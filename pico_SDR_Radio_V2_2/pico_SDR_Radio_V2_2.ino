/*
  pico 40M monoband radio V2.2 JR3XNW
  OLED 128x64 VFO+RX_Bandscope & Waterfall
  Improved AGC and increased volume.
  Add a push switch to GPIO3 for volume control.
  Change sampling rate 8000>80000 2024/3/10
  Changed ArduinoFFT version to 2.0.2. 2024/4/25
  Change sampling rate 80000>10000000 2024/9/14
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

#define I_IN 26               // I signal input pins
#define Q_IN 27               // Q signal input pin
#define PX1 63                // Spectral display X-coordinate
#define PY1 32                // Spectral display Y-coordinate
#define PY2 56                // Frequency scale display Y-coordinate
#define SAMPLES 256           // Number of FFT samples
#define WFrow 12              // Number of waterfall rows.
#define SCREEN_WIDTH 128      // Width of OLED displays
#define SCREEN_HEIGHT 22      // Height of OLED display
//#define SAMPLING_FREQUENCY 100000
#define THRESHOLD 3
#define HISTORY_LENGTH SCREEN_HEIGHT
#define speakerPin 16         // Speaker pins
#define inputPinI 26          // I signal input pins
#define inputPinQ 27          // Q signal input pin
#define sampleRate 500000     // sampling rate 80kHz 200kHz 500kHz
#define pwmFrequency 40000    // PWM Frequency
#define targetAmplitude 0.8   // Target amplitude
#define PIN_IN1 0             // Rotary encoder pins
#define PIN_IN2 1             // Rotary encoder pins
#define STEP_BUTTON 2         // Frequency step change button
#define RX_SW 15              // receive switch
#define VOLUME_BUTTON_PIN 3   // volume control button

volatile float volumeMultiplier = 1.0; // 初期音量設定

// Initialisation of the Rotary encoder
Rotary r = Rotary(PIN_IN1, PIN_IN2);

// Initialisation of Si5351
Si5351 si5351;

int mod = 0; // mode

// Array for FFT data
double vReal[SAMPLES];
double vImag[SAMPLES];

// History for spectrum display
double history[HISTORY_LENGTH][SAMPLES];
int historyIndex = 0; // History Index

// Initialization of OLED displays
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Initialization of ArduinoFFT
ArduinoFFT<double> FFT;  // v2.0.2 Explicit data types using templates

// Initial frequency setting
unsigned long FREQ = 7100000;   // Initial Frequency (7.100 MHz)
unsigned long long FREQ_ULL = 710000000ULL;
unsigned long long pll_freq = 80000000000ULL; // PLL frequency setting 648
const long LOW_FREQ = 7000000;  // Lower limit frequency
const long HI_FREQ = 7300000;   // upper frequency
unsigned long FREQ_OLD = FREQ;  // Previous Frequency
int STEP = 1000;                // Frequency Step
int phase;                      // topology

// Coefficients for Hilbert transform
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

// Coefficients for lower waveband filters
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

// Buffer for filter
float buffer[numTaps] = {0};

// AGC gain
float agcGain = 1.0;

//////////////////////////////////////
// Rotary Encoder Interrupt Routine //
//////////////////////////////////////

// Rotary Encoder Processing Functions
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) {
      FREQ = FREQ + STEP; // Clockwise frequency increase
    } else {
      FREQ = FREQ - STEP; // Counterclockwise frequency decrease
    }
  }
  // Set frequencies within constraints
  FREQ = constrain(FREQ, LOW_FREQ, HI_FREQ);
  FREQ_ULL = FREQ * 100ULL;
}

////////////////////////////////
//   STEP Button Processing   //
////////////////////////////////

// Frequency step switching mode
int stepMode = 0; // 0: 1000Hz, 1: 500Hz, 2: 100Hz

// Function to change the frequency step
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

  // If button remains pressed, standby
  while (digitalRead(STEP_BUTTON) == LOW) {
    delay(10);
  }
}

////////////////////////////
//      Frequency Set     //
////////////////////////////

// Function to set frequency
void Freq_Set() {
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK1);

  // Set phase difference of 90° at PLL frequency/transmit frequency
  phase = pll_freq / FREQ_ULL + 0.5;
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase); // 90°

  // Reset PLL and synchronize phase
  si5351.pll_reset(SI5351_PLLA);

  delay(10);
}

////////////////////////////
//         Hilbert        //
////////////////////////////

// Function to apply a filter
float applyFilter(float input, const float *coeffs, float *buffer) {
  // Shift elements in buffer
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
//    Band Scope Display    //
//////////////////////////////

// spectral display function
void showScope() {
  int d, d1, d2;
  
  // Positive frequency spectrum display
  for (int xi = 1; xi < 64; xi++) {
    d1 = barLength(vReal[xi*2]);
    d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi + 64, PY1 - d, d);  
  }

  // Negative frequency spectrum display
  for (int xi = 64; xi < 128; xi++) {
    d1 = barLength(vReal[xi*2]);
    d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi - 64, PY1 - d, d); 
  }
}

////////////////////////////
//      Graph Length      //
////////////////////////////

// Function to compute the length of a graph
int barLength(double d) {
  float fy;
  int y;

  fy = 14.0 * (log10(d) + 3.0); // Amplitude scaling
  y = fy;
  y = constrain(y, 0, 20); // Set upper and lower limits

  return y;
}

///////////////////////////
//         AGC           //
///////////////////////////

// Function to apply AGC
float applyAGC(float input) {
  float error = targetAmplitude - abs(input);
  agcGain += 0.002 * error;
  if (agcGain < 0.1) {
    agcGain = 0.1;
  }
  return input * agcGain;
}

//////////////////////////////
//        Waterfall         //
//////////////////////////////

// Waterfall Display Functions
void displayWaterfall() {
  // Calculate the magnitude of the FFT result
  for (int xi = 0; xi < SAMPLES; xi++) {
    double d1 = barLength(vReal[xi]);
    double d2 = barLength(vImag[xi]);
    double d = sqrt(d1 * d1 + d2 * d2);
    // Stores FFT results in history
    if (xi < SAMPLES/2) {
      history[historyIndex][xi + SAMPLES / 2] = d; 
    } else {
      history[historyIndex][xi - SAMPLES / 2] = d;
    }
  }

  // Increments the history index and sets it back to 0 when it reaches the end
  historyIndex = (historyIndex + 1) % HISTORY_LENGTH;

  ////////////////////////////////////
  //        Draw the history        //
  ////////////////////////////////////
  for (int y = 0; y < HISTORY_LENGTH; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      // Skip drawing at center 0 Hz frequency position
      if (x == SCREEN_WIDTH / 2) continue;
      // FFT amplitude from history
      double magnitude = history[(historyIndex + y) % HISTORY_LENGTH][x * (SAMPLES / SCREEN_WIDTH)];
      // Pixels are drawn when the amplitude exceeds the threshold value
      if (magnitude > THRESHOLD) {
        u8g2.drawPixel(x, (SCREEN_HEIGHT + 33) - y - 1);
      }
    }
  }
}

////////////////////////////////
//      On-Screen Display     //
////////////////////////////////

// Function to display graphics on the screen
void showGraphics() {
  u8g2.drawHLine(0, 33, 128); // Waterfall lower end line
  u8g2.drawHLine(0, 55, 128); // Waterfall lower end line
  u8g2.drawFrame(86, 6, 42, 6);

  // 周波数スケール (横軸)
  u8g2.drawBox(PX1 - 24, PY2, 2, 2); // Positive frequency scale 10k
  u8g2.drawBox(PX1 - 46, PY2, 2, 2); // Positive frequency scale 20k
  u8g2.drawBox(PX1, PY2, 2, 2);      // Negative frequency scale 0k
  u8g2.drawBox(PX1 + 22, PY2, 2, 2); // Negative frequency scale 10k
  u8g2.drawBox(PX1 + 45, PY2, 2, 2); // Negative frequency scale 20k

  u8g2.setFont(u8g2_font_micro_tr); // Small font (3x5)
  u8g2.drawStr(9, 58, "-20k");      // Negative frequency display
  u8g2.drawStr(32, 58, "-10k");
  u8g2.drawStr(63, 58, "0");        // Positive frequency display
  u8g2.drawStr(81, 58, "10k");
  u8g2.drawStr(105, 58, "20k");

  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(120, 0, "Hz");
  u8g2.drawStr(78, 12, "VR");

  // Dummy display (for future use)
  String freqt;
  u8g2.setFont(u8g2_font_8x13B_tr);
  freqt = String(FREQ);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));

  // STEP Display
  u8g2.setFont(u8g2_font_micro_tr);
  if (STEP == 1000) {
    u8g2.drawStr(100, 0, "1000");
  } else if (STEP == 500) {
    u8g2.drawStr(100, 0, " 500");
  } else {
    u8g2.drawStr(100, 0, " 100");
  }
}

/////////////////////////////
//      Show S Meter       //
/////////////////////////////

// S-meter display function
void showS_meter() {
  // Calculation of RMS value
  double sumSquares = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sumSquares += vReal[i] * vReal[i];
  }
  double rms = sqrt(sumSquares / SAMPLES);

  // Signal strength on logarithmic scale
  double signalLevel = 10 * log10(rms + 1e-9); // Add offset to avoid log zero

  // Sメーターの閾値
  const int S_METER_WIDTH = 42;   // S-meter display width
  const double S_MIN_LEVEL = -50; // Minimum Signal Level (dB) -50
  const double S_MAX_LEVEL = 0;   // Max. signal level (dB) 10

  // Mapping signal level to S-meter display range
  int sMeterValue = map(signalLevel, S_MIN_LEVEL, S_MAX_LEVEL, 0, S_METER_WIDTH);
  sMeterValue = constrain(sMeterValue, 0, S_METER_WIDTH);

  // Debug output: RMS value and signal level
  Serial.print("RMS: ");
  Serial.println(rms, 10); // Output with high accuracy
  Serial.print("Signal Level (dB): ");
  Serial.println(signalLevel, 10); // Output with high accuracy
  Serial.print("S Meter Value: ");
  Serial.println(sMeterValue);

  // S-meter display
  u8g2.drawFrame(86, 6, S_METER_WIDTH, 6);  // S-meter frame
  u8g2.drawBox(86, 6, sMeterValue, 6);      // S-meter bar
  
  /*
  // Numerical display of signal level
  u8g2.setFont(u8g2_font_micro_tr);
  char signalStr[10];
  sprintf(signalStr, "%.1f dB", signalLevel);
  u8g2.setCursor(86, 14);
  u8g2.print(signalStr);
  */
}

/////////////////////////////
//      Draw Rectangles    //
////////////////////////////

// Function to draw a volume bar
void drawRectangles() {
  // Calculate the number of squares (up to 10)
  int numberOfRectangles = (volumeMultiplier - 1.0) * 10 + 1;

  // Drawing a rectangle
  for (int i = 0; i < numberOfRectangles; i++) {
    int x = 86 + i * 5;        // X-coordinate
    int y = 13;                // Y-coordinate
    u8g2.drawBox(x, y, 3, 4);  // Drawing a rectangle
  }
}

//////////////////////////////
//          Setup           //
//////////////////////////////

void setup() {
  pinMode(25, OUTPUT);    // Pico built-in LED pins
  pinMode(RX_SW, OUTPUT);
  Wire.begin();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25000550, 0); // Initialization of Si5351
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);

  r.begin();    // Rotary encoder initialization
  attachInterrupt(0, rotary_encoder, CHANGE); // External Interrupt Settings
  attachInterrupt(1, rotary_encoder, CHANGE);
  pinMode(STEP_BUTTON, INPUT_PULLUP);         // Set STEP button to input with pull-up

  Freq_Set();

  analogReadResolution(12); // Set ADC full scale to 12 bits
  u8g2.begin();
  u8g2.setFlipMode(1);  // Set screen orientation
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop(); // Set the character position reference to the upper left corner.
  u8g2.clearBuffer();
  u8g2.drawStr(34, 0, "7MHz SSB RX");
  u8g2.drawStr(41, 10, "BandScope");
  u8g2.drawStr(22, 20, "Waterfall v2.02");
  u8g2.drawStr(48, 30, "JR3XNW");
  u8g2.sendBuffer();
  delay(1000);
}

//////////////////////////////
//           Loop           //
//////////////////////////////

void loop() {
  digitalWrite(25, HIGH);
  digitalWrite(RX_SW, HIGH);

  // サンプリング
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (analogRead(I_IN) / 2047.5) - 1.0;  //vReal[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;
    vImag[i] = (analogRead(Q_IN) / 2047.5) - 1.0;  //vImag[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;
  }

  digitalWrite(25, LOW);
  
  // If STEP button is pressed, change frequency STEP
  if (digitalRead(STEP_BUTTON) == LOW) {
    Fnc_Stp();
  }

  // Reset frequency if frequency is changed
  if (FREQ != FREQ_OLD) {
    Freq_Set();
    FREQ_OLD = FREQ;
  }
  delay(1);

  // FFT processing
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward); // Use new enumerated types
  FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  u8g2.clearBuffer();   // Clear screen buffer
  showS_meter();        // Display S-meter
  showScope();          // Display spectrum
  displayWaterfall();   // View Waterfall
  showGraphics();       // Display scale lines, etc.
  drawRectangles();     // Show volume bar
  u8g2.sendBuffer();

  delay(10);
}

//////////////////////////////
//          Setup 1         //
//////////////////////////////

void setup1() {
  pinMode(speakerPin, OUTPUT);
  pinMode(inputPinI, INPUT);
  pinMode(inputPinQ, INPUT);
  pinMode(VOLUME_BUTTON_PIN, INPUT_PULLUP); // Set volume button pin as input
  
  analogReadResolution(12);
  analogWriteResolution(8);
  analogWriteFreq(pwmFrequency);
}

//////////////////////////////
//           Loop 1         //
//////////////////////////////

void loop1() {
  float inputI, inputQ;
  float output = 0.0; // Output initialization

  inputI = analogRead(inputPinI);
  inputQ = analogRead(inputPinQ);

  inputI = (inputI / 2047.5) - 1.0;
  inputQ = (inputQ / 2047.5) - 1.0;

  // Apply Hilbert transform
  float hilbertOutput = applyFilter(output, hilbertCoeffs, buffer);

  output = inputI - inputQ;
  applyFilter(output, lsbFilterCoeffs, buffer);

  // Apply AGC
  float agcOutput = applyAGC(hilbertOutput);

  uint8_t pwmOutput = (uint8_t)(min((agcOutput + 1.0) * 127.5 * volumeMultiplier, 255.0));
  analogWrite(speakerPin, pwmOutput);

  delayMicroseconds(1000000 / sampleRate);

  // Check to see if the volume buttons have been pressed
  if (digitalRead(VOLUME_BUTTON_PIN) == LOW) {
    volumeMultiplier += 0.05;  // Increase volume by 0.05Increase volume by 0.1
    if (volumeMultiplier > 1.5) {
      volumeMultiplier = 1.0; // reset to 1.0 after 1.9
    }
    // Debounce processing
    delay(200); 
  }
}
