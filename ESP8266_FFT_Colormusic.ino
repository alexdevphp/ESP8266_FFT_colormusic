#include "arduinoFFT.h"
#include "user_interface.h"
ADC_MODE(ADC_TOUT);

#include "FastLED.h"
#define NUM_LEDS  62
#define LED_PIN   14
#define CHANNEL A0
CRGB leds[NUM_LEDS];


#define BANDS 17 // Bands count. Yo must change topBands array before change it!
#define NOISE 240  // Used as a crude noise filter, values below this are ignored (70 for 512 samles, 240 - for 1024 samples)

const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
const uint8_t s_divisor = 2; //Divisor for FFT samples
const uint8_t s_multiplify = 1.5; //Multiplifier of volume, default for 512 samples - 1.5, for 1024 - 1
//const uint8_t topBands[BANDS] = {3,4,5,6,7,8,9,10,12,13,16,21,25,31,35,41}; //Starting sample number of each band, for 1024 samples
const uint8_t topBands[BANDS] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 30}; //Starting sample number of each band, for 512 samples


double samplingFrequency = 188550 / s_divisor; //Hz
int bandNum;
unsigned long microseconds;


//Gradient from green to red
DEFINE_GRADIENT_PALETTE(gp_volume) {
  0,    0,    255,  0,  // green
  100,  255,  255,  0,  // yellow
  150,  255,  100,  0,  // orange
  200,  255,  50,   0,  // red
  255,  255,  0,    0   // red
};
CRGBPalette32 gpVolume = gp_volume;


double vReal[samples / s_divisor];
double vImag[samples / s_divisor];
uint8_t adc_clk_div = 8; // ADC working clock = 80M/adc_clk_div, range [8, 32], the recommended value is 8
uint16_t adc_addr[samples]; // point to the address of ADC continuously fast sampling output
arduinoFFT FFT = arduinoFFT(vReal, vImag, samples / s_divisor, samplingFrequency);

int bandValues[BANDS];
int bandPrev[BANDS]; //Previous bands values


void setup()
{
  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  Serial.begin(115200);
}

void loop()
{

  analyzeAudio();

  //Show result on led strip
  for (int i = 0; i < NUM_LEDS / 2; i++) {

    int n = (NUM_LEDS / 2) - i;
    int n2 = (NUM_LEDS / 2) + i;
    byte band = floor(i / ((float)NUM_LEDS / (float)BANDS / (float)2));
    byte bandColor = floor(band * ((float)65000 / (float)NUM_LEDS / (float)2));

    if (bandValues[band] > 0) {
      leds[n] = ColorFromPalette(gpVolume, bandColor, 255);
      leds[n2] = ColorFromPalette(gpVolume, bandColor, 255);
      bandPrev[band] = 255;
    }
    else if (bandPrev[band] == 255) {
      bandPrev[band] = 253;
      leds[n] = ColorFromPalette(gpVolume, bandColor, bandPrev[band]);
      leds[n2] = ColorFromPalette(gpVolume, bandColor, bandPrev[band]);
    }
    else if (bandPrev[band] > 100) {              //Slow down the diode decay
      bandPrev[band] -= 100;
      leds[n] = ColorFromPalette(gpVolume, bandColor, bandPrev[band]);
      leds[n2] = ColorFromPalette(gpVolume, bandColor, bandPrev[band]);
    }
    else {
      leds[n] = CHSV(0, 0, 0);
      leds[n2] = CHSV(0, 0, 0);
    }
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(bandValues[i]);
    Serial.print(",");
  }
  Serial.println();

  FastLED.show();
}



//Write FFT results to bandValues
void analyzeAudio() {

  // Reset bandValues[]
  for (int i = 0; i < NUM_LEDS; i++) {
    bandValues[i] = 0;
  }

  microseconds = micros();
  //Fast read audio samples
  system_adc_read_fast(adc_addr, samples, adc_clk_div);

  for (int i = 0; i < samples / s_divisor; i++) {
    vReal[i] = adc_addr[i] * s_multiplify;
    vImag[i] = 0;
  }

  //Counting new sampling Frequency
  microseconds = micros() - microseconds;
  samplingFrequency = (samples * 1000000) / microseconds / s_divisor;

  // Compute FFT
  arduinoFFT FFT = arduinoFFT(vReal, vImag, samples / s_divisor, samplingFrequency);
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  /*
    for(int i = 0; i < topBands[BANDS-1]; i++) {
      Serial.print(i);
      Serial.print(":");
      Serial.print(vReal[i]);
      Serial.print(",");
    }
    Serial.println();
  */
  // Analyse FFT results
  for (int i = 3; i < topBands[BANDS - 1]; i++) {

    //Increase high freq and decrease low freq
    if (i >= 12) vReal[i] = vReal[i] * 2;
    if (i >= 15) vReal[i] = vReal[i] * 2;
    if (i <= 6) vReal[i] = vReal[i] / 3;

    if (vReal[i] > NOISE) {                    // Add a crude noise filter

      // Defining a cell to add value
      bandNum = 0;
      while (i > topBands[bandNum]) {
        bandNum = bandNum + 1;
      }

      //Fill bandValues with maximum freq values
      if (bandValues[bandNum] < (int)vReal[i]) bandValues[bandNum] = (int)vReal[i];
    }
  }
}
