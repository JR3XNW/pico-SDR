/*
  pico VFO + FFT Band Scope OLED128x64 2023/4/24 JR3XNW
  Library to add
  arduinoFFT.h  https://www.arduino.cc/reference/en/libraries/arduinofft/
  Wire.h
  U8g2lib.h     U8glib V2 library for Arduino https:https://github.com/olikraus/U8g2_Arduino
  Rotary.h      Rotary encoder library for Arduino https://github.com/brianlow/Rotary
  Etherkit Si5351 - Arduino Library https://github.com/etherkit/Si5351Arduino
*/

#include <Arduino.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h> // v2.0.2
#include <si5351.h>

#define PIN_IN1 0
#define PIN_IN2 1
#define STEP_BUTTON 2
static int pos = 0;
int rasPicoLED=25;

Rotary r = Rotary(PIN_IN1, PIN_IN2);
Si5351 si5351;

//////////////////////////
// Register set
//////////////////////////
unsigned long FREQ = 7000000; //
unsigned long long FREQ_ULL = 700000000ULL;
unsigned long long pll_freq = 75600000000ULL; //(0.01)

const long LOW_FREQ = 7000000;    // lower frequency limit
const long HI_FREQ = 7200000;     // upper frequency limit
unsigned long FREQ_OLD = FREQ;    // old frequency
int STEP = 1000;                  // STEP(default)

int phase; //90° phase difference setting

//////////////////////////
// Rotary Encoder External Interrupt Processing Routine
//////////////////////////
void rotary_encoder(){
  unsigned char result = r.process();
  if(result){
    if(result == DIR_CW){
      FREQ = FREQ + STEP;
    }else{
    FREQ = FREQ - STEP;
    }
  }
  FREQ = constrain(FREQ,LOW_FREQ,HI_FREQ); //Do not exceed the lower and upper limits of VFO
  FREQ_ULL = FREQ * 100ULL;
}

//////////////////////////
//Processing when STEP SW is pressed
//////////////////////////
void Fnc_Stp()
{
  if(STEP == 10){
    STEP = 1000;
  }
  else{
    STEP /= 10;
  }
  delay(10);
  //Step_Disp(STEP);
  while(digitalRead(STEP_BUTTON) == LOW){
    delay(10);
  }
}

///*************************OLED*****************************///

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); 
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

///***********************GRAPHICS***************************///

#define PX1 63  //Positive frequency screen (Q) origin 62
#define PY1 55  //Bottom edge of spectrum screen 23
#define PY2 56  //24

///**************************FFT*****************************///
#define I_IN 26  //I-Input pins
#define Q_IN 27  //Q-Input pins 

#define SAMPLES 256  //Must be a power of 2
#define WFrow 12

ArduinoFFT<double> FFT;  // v2.0.2 Explicit data types using templates

double vReal[SAMPLES];
double vImag[SAMPLES];

byte DSdata[256];
byte WFdata[WFrow][128];

///*************************core0***************************///

void setup() {

  Wire.begin();                   

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25000658, 0);  //Reference transmitting frequency of Si5351 //27003411//3593
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); //Output 2mA approx. 3dBm / 8mA approx. 10dBm

  r.begin();//Rotary encoder initialization
  attachInterrupt(0,rotary_encoder,CHANGE); //External interrupt setting
  attachInterrupt(1,rotary_encoder,CHANGE);
  pinMode(STEP_BUTTON,INPUT_PULLUP); //STEP_BUTTON Set to input and pull-up
                  
  Freq_Set();
  
  pinMode(rasPicoLED,OUTPUT);      // pico built-in LED
  Serial.begin(115200);

  analogReadResolution(12);  // Set ADC full scale to 12 bits
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();  // The upper left corner is used as the character position reference.
  u8g2.clearBuffer();
  u8g2.drawStr(0, 0, "VFO Band Scope v0.1");
  u8g2.sendBuffer();
  delay(500); 

}

///***********************core0 Main program*************************///
void loop() {
  
  digitalWrite(25, HIGH);  // Built-in LED lights up during sampling 
  /*SAMPLING*/
  for(int i=0; i<SAMPLES; i++)
  {
    vReal[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;  //Arduinoは「0」。
    vImag[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;  //
  }
    digitalWrite(25, LOW);

  if(digitalRead(STEP_BUTTON) == LOW){Fnc_Stp();} //When STEP_BUTTON is pressed, change the frequency STEP
  
  if(FREQ != FREQ_OLD){ 
    Freq_Set();
    //Freq_Disp(FREQ); 
    FREQ_OLD = FREQ;            
  }
  delay(1);

  /*FFT*/
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);  // Use new enum types instead of FFT_WIN_TYP_HAMMING and FFT_FORWARD
  FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);  // Change FFT_REVERSE to FFTDirection::Reverse
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);  // No change
    
  u8g2.clearBuffer();   // Screen buffer clear
  showScope();       // Spectrum Display
  showGraphics();         // Scale line and other indications
  showS_meter();  //Smeter
  u8g2.sendBuffer();    // 

  delay(1);  //Repeat the process every second OR:
    
}

//////////////////////////
// frequency set
//////////////////////////
void Freq_Set(){
  // Set CLK0 and CLK1
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK1);

  phase = pll_freq / FREQ_ULL + 0.5; //PLL frequency/transmitter frequency 90° phase difference setting value Rounding
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase); //90°

  // We need to reset the PLL before they will be in phase alignment
  si5351.pll_reset(SI5351_PLLA);

  delay(10);
}

///******************************core1********************************///
void setup1() {
}

///***********************core1 Main program*************************///
void loop1() {
}

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

void showS_meter() {  // Spectrum Display
  int d, d1, d2;
  
  for (int xi = 1; xi < 64; xi++) {  // Positive frequency spectrum display
    d1 = barLength(vReal[xi*2]);
    d2 = barLength(vImag[xi*2+1]);
    d = (d1 + d2)*1.5 ; //
    u8g2.drawBox(76, 23, d, 5);  
  }
}

int barLength(double d) {  // Calculate the length of the graph
  float fy;
  int y;

  fy = 14.0 * (log10(d) + 3.3);  //
  y = fy;
  y = constrain(y, 0, 16);  // Cut off upper and lower limits 85

  return y;
}

void showGraphics() {  // Modifying Graphs
  // area demarcation line
  u8g2.drawFrame(0, 0, 128, 20);
  u8g2.drawFrame(0, 31, 128, 26);
  u8g2.drawLine(40, 0, 40, 19);

  u8g2.setFont(u8g2_font_5x7_tf);  // 
  u8g2.drawStr(3, 2, "picoSDR");       //
  u8g2.drawStr(3, 10, "JR3XNW>");

  u8g2.setFont(u8g2_font_mozart_nbp_tr);  // Small font(3x5) u8g2_font_6x13_tf
  u8g2.drawStr(0, 21, "STEP"); 
  u8g2.drawStr(62, 21, "SM");

  // Frequency scale (horizontal axis)
  u8g2.drawBox(PX1 - 22, PY2, 2, 2);  // Positive Frequency 10kscale
  u8g2.drawBox(PX1 - 44, PY2, 2, 2);  // Positive Frequency 20kscale

  u8g2.drawBox(PX1, PY2, 2, 2);       // Negative frequency 0kscale
  u8g2.drawBox(PX1 + 22, PY2, 2, 2);  // Negative frequency 10kscale
  u8g2.drawBox(PX1 + 45, PY2, 2, 2);  // Negative frequency 20kscale

  u8g2.setFont(u8g2_font_micro_tr);  // Small font(3x5)
  u8g2.drawStr(11, 58, "-20k");       // Negative frequency display 58
  u8g2.drawStr(34, 58, "-10k");

  u8g2.drawStr(63, 58, "0");  // Positive Frequency
  u8g2.drawStr(81, 58, "10k");
  u8g2.drawStr(105, 58, "20k");
  
  // Dummy display for future use
  String freqt;
  u8g2.setFont(u8g2_font_lubR12_tf); //u8g_font_unifont, u8g2_font_t0_16_mr, u8g2_font_mozart_nbp_tr, u8g2_font_t0_15_tf, u8g2_font_crox2hb_tn
  //u8g2.drawStr(44, 3, fff);
  freqt = String (FREQ);
  u8g2.setCursor(44,3);
  u8g2.print(freqt.substring(0,1) + "." + freqt.substring(1,4) + "." + freqt.substring(4));

  //STEP Display
  u8g2.setFont(u8g2_font_mozart_nbp_tr);  //
  if (STEP == 1000) {
    u8g2.drawStr(26, 21, "1kHz");
  }
    else if ( STEP == 100) {
      u8g2.drawStr(26, 21, "100Hz");
    } else {
      u8g2.drawStr(26, 21, "10Hz");
    }  
}
