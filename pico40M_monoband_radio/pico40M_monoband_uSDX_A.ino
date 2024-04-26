/*
  pico 40M monoband radio
  OLED 128x64 VFO+RX_Bandscope Waterfall JR3XNW 2023/10/07
*/

#include <Arduino.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h> // v2.0.2
#include <si5351.h>

//////////////////////////////////////////
//   Global Variables and Definitions   //
//////////////////////////////////////////

#define I_IN 26
#define Q_IN 27
#define PX1 63
#define PY1 32
#define PY2 56
#define SAMPLES 256
#define WFrow 12
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 22
#define SAMPLING_FREQUENCY 50000
#define THRESHOLD 3
#define HISTORY_LENGTH SCREEN_HEIGHT
#define speakerPin 14
#define inputPinI 26
#define inputPinQ 27
#define sampleRate 8000 //8000
#define pwmFrequency 40000  //40000
#define targetAmplitude 0.8 //0.8
#define PIN_IN1 0
#define PIN_IN2 1
#define STEP_BUTTON 2
#define RX_SW 15

Rotary r = Rotary(PIN_IN1, PIN_IN2);

Si5351 si5351;

int mod = 0;

double vReal[SAMPLES];
double vImag[SAMPLES];

double history[HISTORY_LENGTH][SAMPLES];
int historyIndex = 0;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ArduinoFFT<double> FFT;  // v2.0.2 Explicit data types using templates

unsigned long FREQ = 7074000;
unsigned long long FREQ_ULL = 707400000ULL;
unsigned long long pll_freq = 64800000000ULL; //75000000000ULL//65000000000ULL
const long LOW_FREQ = 7000000;
const long HI_FREQ = 7200000;
unsigned long FREQ_OLD = FREQ;
int STEP = 1000;
int phase;

const int numTaps = 31;
const float hilbertCoeffs[numTaps] = {
 -0.001328193331496,  0.000000000000000, -0.002284533986424,
  0.000000000000000, -0.003313586555888,  0.000000000000000,
 -0.004387527896369,  0.000000000000000, -0.005470862958678,
  0.000000000000000, -0.006500296593254,  0.000000000000000,
 -0.007388063473212,  0.000000000000000, -0.008023180743934,
  0.000000000000000, -0.008266448739536,  0.000000000000000,
 -0.007942155320414,  0.000000000000000, -0.006729329308188,
  0.000000000000000, -0.004233803027843,  0.000000000000000,
  0.000000000000000, -0.005656339687371,  0.000000000000000,
 -0.016018448245468,  0.000000000000000, -0.031105779790221,
  0.000000000000000
};

const float lsbFilterCoeffs[numTaps] = {
 -1.57196597e-03, -1.45104171e-03,  1.12478657e-03,  4.46246491e-03,
  2.57714059e-03, -6.99319701e-03, -1.30293770e-02,  7.20713963e-18,
  2.47619628e-02,  2.56472800e-02, -1.88013680e-02, -6.76180323e-02,
 -3.71204042e-02,  1.08307805e-01,  2.91780827e-01,  3.75846238e-01,
  2.91780827e-01,  1.08307805e-01, -3.71204042e-02, -6.76180323e-02,
 -1.88013680e-02,  2.56472800e-02,  2.47619628e-02,  7.20713963e-18,
 -1.30293770e-02, -6.99319701e-03,  2.57714059e-03,  4.46246491e-03,
  1.12478657e-03, -1.45104171e-03, -1.57196597e-03
};

float buffer[numTaps] = {0};
float agcGain = 1.0;

//////////////////////////////////////
// Rotary Encoder Interrupt Routine //
//////////////////////////////////////
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) {
      FREQ = FREQ + STEP;
    } else {
      FREQ = FREQ - STEP;
    }
  }
  FREQ = constrain(FREQ, LOW_FREQ, HI_FREQ);  //Do not exceed the lower and upper limits of VFO
  FREQ_ULL = FREQ * 100ULL;
}

////////////////////////////////
//   STEP Button Processing   //
////////////////////////////////
int stepMode = 0; // 0 for 1000, 1 for 500, 2 for 100

void Fnc_Stp() {
  if (stepMode == 0) {
    stepMode = 1;
    STEP = 500;
  } else if (stepMode == 1) {
    stepMode = 2;
    STEP = 100;
  } else {
    stepMode = 0;
    STEP = 1000;
  }
  
  delay(10);
  
  while (digitalRead(STEP_BUTTON) == LOW) {
    delay(10);
  }
}


//////////////////////////////
//          setup 0         //
//////////////////////////////
void setup() {

  pinMode(25, OUTPUT);    // pico built-in LED
  pinMode(RX_SW, OUTPUT);
  //Serial.begin(115200);

  Wire.begin();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 27008201, 0);  //Reference transmitting frequency of Si5351 //27003411//3593//24999518
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); //Output 2mA approx. 3dBm / 8mA approx. 10dBm
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);

  r.begin();    //Rotary encoder initialization
  attachInterrupt(0, rotary_encoder, CHANGE); //External interrupt setting
  attachInterrupt(1, rotary_encoder, CHANGE);
  pinMode(STEP_BUTTON, INPUT_PULLUP); //STEP_BUTTON Set to input and pull-up

  Freq_Set();

  analogReadResolution(12);  // Set ADC full scale to 12 bits
  u8g2.begin();
  u8g2.setFlipMode(1);   // Set screen orientation, switch between 0 and 1
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();  // The upper left corner is used as the character position reference.
  u8g2.clearBuffer();
  u8g2.drawStr(25, 0, "7MHzVFO SSB RX");
  u8g2.drawStr(42, 10, "BandScope");
  u8g2.drawStr(25, 20, "Waterfall v0.1");
  u8g2.drawStr(48, 30, "JR3XNW");
  u8g2.sendBuffer();
  delay(1000);
}

//////////////////////////////
//           loop 0         //
//////////////////////////////
void loop() {
  digitalWrite(25, HIGH);
  digitalWrite(RX_SW, HIGH);

  /*SAMPLING*/
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;
    vImag[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;
  }

  digitalWrite(25, LOW);
  //When STEP_BUTTON is pressed, change the frequency STEP
  if (digitalRead(STEP_BUTTON) == LOW) {
    Fnc_Stp();
  }

  if (FREQ != FREQ_OLD) {
    Freq_Set();
    FREQ_OLD = FREQ;
  }
  delay(1);

  //FFT
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);  // Use new enum types instead of FFT_WIN_TYP_HAMMING and FFT_FORWARD
  FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);  // Change FFT_REVERSE to FFTDirection::Reverse
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);  // No change

  u8g2.clearBuffer();   //Screen buffer clear
  showS_meter();        //Show Smeter
  showScope();          //Show Spectrum
  displayWaterfall();   //Show Waterfall
  showGraphics();;      //Scale line and other indications
  //showS_meter();        //Show Smeter
  u8g2.sendBuffer();

  delay(10);
}

////////////////////////////
//      frequency set     //
////////////////////////////
void Freq_Set() {
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK1);

  phase = pll_freq / FREQ_ULL + 0.5;  //PLL frequency/transmitter frequency 90° phase difference setting value Rounding
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase); //90°

  //We need to reset the PLL before they will be in phase alignment
  si5351.pll_reset(SI5351_PLLA);

  delay(10);
}

//////////////////////////////
//          setup 1         //
//////////////////////////////
void setup1() {

  pinMode(speakerPin, OUTPUT);
  pinMode(inputPinI, INPUT);
  pinMode(inputPinQ, INPUT);

  analogReadResolution(12);
  analogWriteResolution(8);
  analogWriteFreq(pwmFrequency);
}

////////////////////////////
//         Hilbert        //
////////////////////////////
float applyFilter(float input, const float *coeffs, float *buffer) {
  // Shift elements in the buffer
  for (int i = numTaps - 1; i > 0; i--) {
    buffer[i] = buffer[i - 1];
  }
  buffer[0] = input;
  float output = 0;
  for (int i = 0; i < numTaps; i++) {
    output += buffer[i] * coeffs[i];
  }
  return output;
}

//////////////////////////////
//           loop 1         //
//////////////////////////////
void loop1() {
  float inputI, inputQ;
  float output = 0.0; //initialize output

  inputI = analogRead(inputPinI);
  inputQ = analogRead(inputPinQ);

  inputI = (inputI / 2047.5) - 1.0; //(inputI / 2047.5) - 1.0;
  inputQ = (inputQ / 2047.5) - 1.0;

  //Apply the Hilbert Transform
  float hilbertOutput = applyFilter(output, hilbertCoeffs, buffer);

  output = inputI - inputQ;
  applyFilter(output, lsbFilterCoeffs, buffer);

  // Apply AGC
  float agcOutput = applyAGC(hilbertOutput);
  uint8_t pwmOutput = (uint8_t)((agcOutput + 1.0) * 127.5); //agcOutput + 1.0

  analogWrite(speakerPin, pwmOutput);
  delayMicroseconds(100000 / sampleRate);
}

//////////////////////////////
//    Band scope display    //
//////////////////////////////
void showScope() {  // Spectrum Display
  int d, d1, d2;
  
  for (int xi = 1; xi < 64; xi++) {  // Positive frequency spectrum display
      d1 = barLength(vReal[xi*2]);
      d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi + 64 , PY1 - d, d);  
  }
  for (int xi = 64; xi < 128; xi++) { // Negative frequency spectrum display
      d1 = barLength(vReal[xi*2]);
      d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi - 64 , PY1 - d, d); 
  }
}

////////////////////////////
//      Graph Length      //
////////////////////////////
int barLength(double d) {  // Calculate the length of the graph
  float fy;
  int y;

  fy = 14.0 * (log10(d) + 3.0);  //3.3 14
  y = fy;
  y = constrain(y, 0, 20);  // Cut off upper and lower limits

  return y;
}

///////////////////////////
//         AGC           //
///////////////////////////
float applyAGC(float input) {
  float error = targetAmplitude - abs(input);
  agcGain += 0.002 * error;
  if (agcGain < 0.1) {
    agcGain = 0.1;
  }
  return input * agcGain;
}

//////////////////////////////
//        Water fall        //
//////////////////////////////
void displayWaterfall() {
  // Calculate the magnitudes of the FFT results
  for (int xi = 0; xi < SAMPLES; xi++) {
    double d1 = barLength(vReal[xi]);
    double d2 = barLength(vImag[xi]);
    double d = sqrt(d1 * d1 + d2 * d2);
    // Store the FFT results into the history array
    if(xi < SAMPLES/2){
      history[historyIndex][xi + SAMPLES / 2] = d; 
    }else{
      history[historyIndex][xi - SAMPLES / 2] = d;
    }
  }

  //Increment the history index, looping back to 0 if it reaches the end of the array
  historyIndex = (historyIndex + 1) % HISTORY_LENGTH;

  ////////////////////////////////////
  //        Draw the history        //
  ////////////////////////////////////
  for (int y = 0; y < HISTORY_LENGTH; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      //Skip drawing the pixel at the center of the screen (0 Hz frequency)
      if (x == SCREEN_WIDTH / 2) continue;
      //Retrieve the FFT magnitude from the history
      double magnitude = history[(historyIndex + y) % HISTORY_LENGTH][x * (SAMPLES / SCREEN_WIDTH)];
      //If the magnitude is above the threshold, draw a pixel
      if (magnitude > THRESHOLD) {
        u8g2.drawPixel(x, (SCREEN_HEIGHT + 33) - y - 1);
      }
    }
  }
}

////////////////////////////////
//      on-screen display     //
////////////////////////////////
void showGraphics() {  // Modifying Graphs
  u8g2.drawHLine(0, 33, 128);  // Lower end line for waterfall 55
  u8g2.drawHLine(0, 55, 128);  // Lower end line for waterfall 55
  u8g2.drawFrame(86, 6, 42, 6);
  //u8g2.drawLine(63, 0, 63, 13);

  //Frequency scale (horizontal axis)
  u8g2.drawBox(PX1 - 24, PY2, 2, 2);  // Positive Frequency 10kscale
  u8g2.drawBox(PX1 - 46, PY2, 2, 2);  // Positive Frequency 20kscale

  u8g2.drawBox(PX1, PY2, 2, 2);       // Negative frequency 0kscale
  u8g2.drawBox(PX1 + 22, PY2, 2, 2);  // Negative frequency 10kscale
  u8g2.drawBox(PX1 + 45, PY2, 2, 2);  // Negative frequency 20kscale

  u8g2.setFont(u8g2_font_micro_tr);  // Small font(3x5)
  u8g2.drawStr(9, 58, "-20k");       // Negative frequency display 58 26
  u8g2.drawStr(32, 58, "-10k");

  u8g2.drawStr(63, 58, "0");  // Positive Frequency
  u8g2.drawStr(81, 58, "10k");
  u8g2.drawStr(105, 58, "20k");

  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(120, 0, "Hz");

  //Dummy display for future use
  String freqt;
  u8g2.setFont(u8g2_font_8x13B_tr); //u8g_font_unifont, u8g2_font_t0_16_mr, u8g2_font_mozart_nbp_tr, u8g2_font_t0_15_tf, u8g2_font_crox2hb_tn
  //u8g2.drawStr(2, 1, fff);
  freqt = String (FREQ);
  u8g2.setCursor(2,1);
  u8g2.print(freqt.substring(0,1) + "." + freqt.substring(1,4) + "." + freqt.substring(4));

  //STEP Display
  u8g2.setFont(u8g2_font_micro_tr);  //
  if (STEP == 1000) {
    u8g2.drawStr(100, 0, "1000");
  }
    else if ( STEP == 500) {
      u8g2.drawStr(100, 0, " 500");
    } else {
      u8g2.drawStr(100, 0, " 100");
    }  

}

/////////////////////////////
//      showS_meter        //
/////////////////////////////
void showS_meter() {  //Spectrum Display
  int d, d1, d2;

  for (int xi = 1; xi < 64; xi++) {  //Positive frequency spectrum display
    d1 = barLength(vReal[xi*2]);
    d2 = barLength(vImag[xi*2+1]);
    d = (d1 + d2) * 2.0 ; //1.0
    u8g2.drawBox(86, 6, d, 6);  
  }
}