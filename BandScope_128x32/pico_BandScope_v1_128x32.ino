/*
  OLED 128x32 Bandscope JR3XNW 2023/3/14
  Library to add
  arduinoFFT.h    https://www.arduino.cc/reference/en/libraries/arduinofft/
  Wire.h
  U8g2lib.h       https://github.com/olikraus/U8g2_Arduino
*/

#include <U8g2lib.h>  // This is for controlling the OLED display
#include <Wire.h>  // This library is needed for I2C communication, which is used by the OLED display
#include "arduinoFFT.h"  // This library is used to perform Fast Fourier Transform

//**************************FFT*****************************
#define I_IN 26  //I-Input pins
#define Q_IN 27  //Q-Input pins
#define MOD_BUTTON 2  //mode switching

#define PX1 63  //Positive frequency screen (Q) origin 62
#define PY1 23  //Bottom edge of spectrum screen 42
#define PY2 24  //56
#define SAMPLES 256  //Must be a power of 2
#define WFrow 12

int mod = 0;

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); 
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLES];
double vImag[SAMPLES];

byte DSdata[256];
byte WFdata[WFrow][128];

void setup() {
  pinMode(25, OUTPUT);      // pico built-in LED
  pinMode(MOD_BUTTON,INPUT_PULLUP); //MODE_SWITCHING Set to input and pull-up
  Serial.begin(115200);

  analogReadResolution(12);  // Set ADC full scale to 12 bits
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();  // The upper left corner is used as the character position reference.
  u8g2.clearBuffer();
  u8g2.drawStr(0, 0, "Band Scope v0.1");
  u8g2.sendBuffer();
  delay(500); 

}
 
void loop() {
    digitalWrite(25, HIGH);  // Built-in LED lights up during sampling 
    if(digitalRead(MOD_BUTTON) == LOW){Mod_Stp();} //When MOD_BUTTON is pressed, change mode

    /*SAMPLING*/
    if ( mod == 0 ) {
      for(int i=0; i<SAMPLES; i++)
      {
        vReal[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;  //Arduinoは「0」。
        vImag[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;  //
      }
      digitalWrite(25, LOW);
    } else {
      for(int i=0; i<SAMPLES; i++)
      {
        vReal[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;  //Arduinoは「0」。
        vImag[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;  //
      }
      digitalWrite(25, LOW);
    }

    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Windowing(vImag, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); 
    FFT.Compute(vReal, vImag, SAMPLES, FFT_REVERSE);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    
    u8g2.clearBuffer();  // Screen buffer clear
    showScope();  // Spectrum Display
    showGraphics();  // Scale line and other indications
    show_mod();  //mode
    u8g2.sendBuffer();  // 

    delay(1);  //Repeat the process every second OR:
    
}

//////////////////////////
//Processing when STEP SW is pressed
//////////////////////////
void Mod_Stp()
{
  if(mod == 0){
    mod = 1;
  }
  else{
    mod = 0;
  }
  delay(10);
  //Step_Disp(STEP);
  while(digitalRead(MOD_BUTTON) == LOW){
    delay(10);
  }
}

//////////////////////////
//Band scope display
//////////////////////////
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

//////////////////////////
//LSB/USB switching
//////////////////////////
void show_mod() {  // Spectrum Display
  u8g2.setFont(u8g2_font_micro_tr);  // Small font(3x5)
  if ( mod == 0 ){
    u8g2.drawStr(0, 0, "LSB");       //
  } else {
    u8g2.drawStr(117, 0, "USB");       //
  }
}

//////////////////////////
//Graph Length
//////////////////////////
int barLength(double d) {  // Calculate the length of the graph
  float fy;
  int y;

  fy = 14.0 * (log10(d) + 3.3);  //3.3 14
  y = fy;
  y = constrain(y, 0, 22);  // Cut off upper and lower limits

  /*For Test*/
  Serial.print(d, 4);
  Serial.print(", ");
  Serial.print(fy);
  Serial.print(", ");
  Serial.println(y);
  
  return y;
}

//////////////////////////
//on-screen display
//////////////////////////
void showGraphics() {  // Modifying Graphs
  // area demarcation line
  //u8g2.drawHLine(0, PY1, 128);  // lower end of the spectrum
  u8g2.drawHLine(0, 23, 128);  // Lower end line for waterfall 55
  //u8g2.drawFrame(0, 0, 128, 13);
  //u8g2.drawLine(63, 0, 63, 13);

  // Frequency scale (horizontal axis)
  u8g2.drawBox(PX1 - 24, PY2, 2, 2);  // Positive Frequency 10kscale
  u8g2.drawBox(PX1 - 46, PY2, 2, 2);  // Positive Frequency 20kscale

  u8g2.drawBox(PX1, PY2, 2, 2);       // Negative frequency 0kscale
  u8g2.drawBox(PX1 + 22, PY2, 2, 2);  // Negative frequency 10kscale
  u8g2.drawBox(PX1 + 45, PY2, 2, 2);  // Negative frequency 20kscale

  u8g2.setFont(u8g2_font_micro_tr);  // Small font(3x5)
  u8g2.drawStr(9, 26, "-20k");       // Negative frequency display 58
  u8g2.drawStr(32, 26, "-10k");

  u8g2.drawStr(63, 26, "0");  // Positive Frequency
  u8g2.drawStr(81, 26, "10k");
  u8g2.drawStr(105, 26, "20k");
  
}
