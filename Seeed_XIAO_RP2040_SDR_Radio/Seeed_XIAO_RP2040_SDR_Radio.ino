/*
  Seeed XIAO RP2040 40M monoband radio V1.2 JR3XNW 2024/8/5
  OLED 128x64 VFO+RX_Bandscope & Waterfall
  Improved AGC and increased volume.
  Add a push switch to GPIO3 for volume control.
  Change sampling rate 8000>80000 2024/3/10
  Changed ArduinoFFT version to 2.0.2. 2024/4/25
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

#define I_IN D0               // I信号入力ピン
#define Q_IN D1               // Q信号入力ピン
#define PX1 63                // スペクトル表示X座標
#define PY1 32                // スペクトル表示Y座標
#define PY2 56                // 周波数スケール表示Y座標
#define SAMPLES 256           // FFTサンプル数
#define WFrow 12              // ウォーターフォールの行数
#define SCREEN_WIDTH 128      // OLEDディスプレイの幅
#define SCREEN_HEIGHT 22      // OLEDディスプレイの高さ
#define SAMPLING_FREQUENCY 100000
#define THRESHOLD 1
#define HISTORY_LENGTH SCREEN_HEIGHT
#define speakerPin D10         // スピーカーのピン 14
#define inputPinI D0          // I信号入力ピン
#define inputPinQ D1          // Q信号入力ピン
#define sampleRate 500000      // サンプリングレート 80000
#define pwmFrequency 44100    // PWM周波数
#define targetAmplitude 0.8   // 目標の振幅
#define PIN_IN1 D7             // ロータリーエンコーダピン0 D6
#define PIN_IN2 D6             // ロータリーエンコーダピン1 D7
#define STEP_BUTTON D8         // 周波数ステップ変更ボタン2
//#define RX_SW 6              // 受信スイッチ 15
#define VOLUME_BUTTON_PIN D9   // 音量調整ボタン3

volatile float volumeMultiplier = 1.0; // 初期音量設定

// Rotaryエンコーダの初期化
Rotary r = Rotary(PIN_IN1, PIN_IN2);

// Si5351の初期化
Si5351 si5351;

int mod = 0; // モード

// FFTデータ用の配列
double vReal[SAMPLES];
double vImag[SAMPLES];

// スペクトル表示用の履歴
double history[HISTORY_LENGTH][SAMPLES];
int historyIndex = 0; // 履歴のインデックス

// OLEDディスプレイの初期化
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE);  //u8g2(U8G2_R0,U8X8_PIN_NONE,5,4)

// ArduinoFFTの初期化
ArduinoFFT<double> FFT;  // v2.0.2 テンプレートを使用した明示的なデータ型

// 初期周波数設定
unsigned long FREQ = 7100000; // 初期周波数 (7.074 MHz)
unsigned long long FREQ_ULL = 710000000ULL;
unsigned long long pll_freq = 75000000000ULL; // PLL周波数設定 648
const long LOW_FREQ = 7000000;  // 下限周波数
const long HI_FREQ = 7200000;   // 上限周波数
unsigned long FREQ_OLD = FREQ;  // 前回の周波数
int STEP = 1000;                // 周波数ステップ
int phase;                      // 位相

// ヒルベルト変換用の係数
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

// 下側波帯フィルタ用の係数
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

// フィルタ用バッファ
float buffer[numTaps] = {0};

// AGCのゲイン
float agcGain = 1.0;

//////////////////////////////////////
// Rotary Encoder Interrupt Routine //
//////////////////////////////////////

// ロータリーエンコーダの処理関数
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) {
      FREQ = FREQ + STEP; // 時計回りで周波数増加
    } else {
      FREQ = FREQ - STEP; // 反時計回りで周波数減少
    }
  }
  // 周波数を制約内に設定
  FREQ = constrain(FREQ, LOW_FREQ, HI_FREQ);
  FREQ_ULL = FREQ * 100ULL;
}

////////////////////////////////
//   STEP Button Processing   //
////////////////////////////////

// 周波数ステップの切り替えモード
int stepMode = 0; // 0: 1000Hz, 1: 500Hz, 2: 100Hz

// 周波数ステップを変更する関数
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

  // ボタンが押されたままの場合は待機
  while (digitalRead(STEP_BUTTON) == LOW) {
    delay(10);
  }
}

////////////////////////////
//      Frequency Set     //
////////////////////////////

// 周波数を設定する関数
void Freq_Set() {
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK1);

  // PLL周波数/送信周波数で90°の位相差を設定
  phase = pll_freq / FREQ_ULL + 0.5;
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase); // 90°

  // PLLをリセットして位相を同期
  si5351.pll_reset(SI5351_PLLA);

  delay(10);
}

////////////////////////////
//         Hilbert        //
////////////////////////////

// フィルタを適用する関数
float applyFilter(float input, const float *coeffs, float *buffer) {
  // バッファ内の要素をシフト
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

// スペクトル表示関数
void showScope() {
  int d, d1, d2;
  
  // 正の周波数スペクトル表示
  for (int xi = 1; xi < 64; xi++) {
    d1 = barLength(vReal[xi*2]);
    d2 = barLength(vImag[xi*2+1]);
    d = sqrt(d1 * d1 + d2 * d2);
    u8g2.drawVLine(xi + 64, PY1 - d, d);  
  }

  // 負の周波数スペクトル表示
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

// グラフの長さを計算する関数
int barLength(double d) {
  float fy;
  int y;

  fy = 14.0 * (log10(d) + 3.0); // 振幅のスケーリング
  y = fy;
  y = constrain(y, 0, 20); // 上限と下限を設定

  return y;
}

///////////////////////////
//         AGC           //
///////////////////////////

// AGCを適用する関数
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

// ウォーターフォール表示関数
void displayWaterfall() {
  // FFTの結果の大きさを計算
  for (int xi = 0; xi < SAMPLES; xi++) {
    double d1 = barLength(vReal[xi]);
    double d2 = barLength(vImag[xi]);
    double d = sqrt(d1 * d1 + d2 * d2);
    // FFTの結果を履歴に格納
    if (xi < SAMPLES/2) {
      history[historyIndex][xi + SAMPLES / 2] = d; 
    } else {
      history[historyIndex][xi - SAMPLES / 2] = d;
    }
  }

  // 履歴インデックスをインクリメントし、末尾に達したら0に戻す
  historyIndex = (historyIndex + 1) % HISTORY_LENGTH;

  ////////////////////////////////////
  //        Draw the history        //
  ////////////////////////////////////
  for (int y = 0; y < HISTORY_LENGTH; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      // 中心の0 Hz周波数位置の描画をスキップ
      if (x == SCREEN_WIDTH / 2) continue;
      // 履歴からFFTの振幅を取得
      double magnitude = history[(historyIndex + y) % HISTORY_LENGTH][x * (SAMPLES / SCREEN_WIDTH)];
      // 振幅が閾値を超えた場合にピクセルを描画
      if (magnitude > THRESHOLD) {
        u8g2.drawPixel(x, (SCREEN_HEIGHT + 33) - y - 1);
      }
    }
  }
}

////////////////////////////////
//      On-Screen Display     //
////////////////////////////////

// 画面上にグラフィックを表示する関数
void showGraphics() {
  u8g2.drawHLine(0, 33, 128); // ウォーターフォールの下端線
  u8g2.drawHLine(0, 55, 128); // ウォーターフォールの下端線
  u8g2.drawFrame(86, 6, 42, 6);

  // 周波数スケール (横軸)
  u8g2.drawBox(PX1 - 24, PY2, 2, 2); // 正の周波数スケール 10k
  u8g2.drawBox(PX1 - 46, PY2, 2, 2); // 正の周波数スケール 20k
  u8g2.drawBox(PX1, PY2, 2, 2);      // 負の周波数スケール 0k
  u8g2.drawBox(PX1 + 22, PY2, 2, 2); // 負の周波数スケール 10k
  u8g2.drawBox(PX1 + 45, PY2, 2, 2); // 負の周波数スケール 20k

  u8g2.setFont(u8g2_font_micro_tr); // 小さなフォント (3x5)
  u8g2.drawStr(9, 58, "-20k");      // 負の周波数表示
  u8g2.drawStr(32, 58, "-10k");
  u8g2.drawStr(63, 58, "0");        // 正の周波数表示
  u8g2.drawStr(81, 58, "10k");
  u8g2.drawStr(105, 58, "20k");

  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(120, 0, "Hz");
  u8g2.drawStr(78, 12, "VR");

  // ダミーディスプレイ（将来的な使用のため）
  String freqt;
  u8g2.setFont(u8g2_font_8x13B_tr);
  freqt = String(FREQ);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));

  // STEP表示
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

// Sメーター表示関数
void showS_meter() {
  // RMS値の計算
  double sumSquares = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sumSquares += vReal[i] * vReal[i];
  }
  double rms = sqrt(sumSquares / SAMPLES);

  // 対数スケールでの信号強度
  double signalLevel = 10 * log10(rms);

  // Sメーターの閾値
  const int S_METER_WIDTH = 42; // Sメーターの表示幅
  const double S_MIN_LEVEL = -50; // 最低信号レベル (dB)
  const double S_MAX_LEVEL = 10; // 最高信号レベル (dB)

  // 信号レベルをSメーターの表示範囲にマッピング
  int sMeterValue = map(signalLevel, S_MIN_LEVEL, S_MAX_LEVEL, 0, S_METER_WIDTH);
  sMeterValue = constrain(sMeterValue, 0, S_METER_WIDTH);

  // Sメーターの表示
  u8g2.drawFrame(86, 6, S_METER_WIDTH, 6); // Sメーターの枠
  u8g2.drawBox(86, 6, sMeterValue, 6); // Sメーターのバー
  /*
  // 信号レベルの数値表示
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

// 音量バーを描画する関数
void drawRectangles() {
  // 四角形の数を計算（最大10個）
  int numberOfRectangles = (volumeMultiplier - 1.0) * 10 + 1;

  // 四角形を描画
  for (int i = 0; i < numberOfRectangles; i++) {
    int x = 86 + i * 5; // X座標
    int y = 13;         // Y座標
    u8g2.drawBox(x, y, 3, 4); // 四角形を描画
  }
}

//////////////////////////////
//          Setup           //
//////////////////////////////

void setup() {
  pinMode(25, OUTPUT);    // Pico内蔵LEDのピン
  pinMode(I_IN, INPUT);
  pinMode(Q_IN, INPUT);
  //pinMode(RX_SW, OUTPUT);
  Wire.begin();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25000632, 0); // Si5351の初期化
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);

  r.begin();    // ロータリーエンコーダの初期化
  attachInterrupt(0, rotary_encoder, CHANGE); // 外部割り込みの設定
  attachInterrupt(1, rotary_encoder, CHANGE);
  pinMode(STEP_BUTTON, INPUT_PULLUP); // STEPボタンをプルアップ付きの入力に設定

  Freq_Set();

  analogReadResolution(12); // ADCのフルスケールを12ビットに設定
  u8g2.begin();
  u8g2.setFlipMode(1); // 画面の向きを設定
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop(); // 文字位置の基準を左上に設定
  u8g2.clearBuffer();
  u8g2.drawStr(25, 0, "7MHzVFO SSB RX");
  u8g2.drawStr(42, 10, "BandScope");
  u8g2.drawStr(25, 20, "Waterfall v0.102");
  u8g2.drawStr(25, 30, "Seed XIAO JR3XNW");
  u8g2.sendBuffer();
  delay(1000);
}

//////////////////////////////
//           Loop           //
//////////////////////////////

void loop() {
  digitalWrite(25, HIGH);
  //digitalWrite(RX_SW, HIGH);

  // サンプリング
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (analogRead(I_IN) - 2048) * 3.3 / 4096.0;
    vImag[i] = (analogRead(Q_IN) - 2048) * 3.3 / 4096.0;
  }

  digitalWrite(25, LOW);
  
  // STEPボタンが押された場合、周波数STEPを変更
  if (digitalRead(STEP_BUTTON) == LOW) {
    Fnc_Stp();
  }

  // 周波数が変更された場合、周波数を再設定
  if (FREQ != FREQ_OLD) {
    Freq_Set();
    FREQ_OLD = FREQ;
  }
  delay(1);

  // FFT処理
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward); // 新しい列挙型を使用
  FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  u8g2.clearBuffer();   // 画面バッファをクリア
  showS_meter();        // Sメーターを表示
  showScope();          // スペクトルを表示
  displayWaterfall();   // ウォーターフォールを表示
  showGraphics();       // スケール線などを表示
  drawRectangles();     // 音量バーを表示
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
  pinMode(VOLUME_BUTTON_PIN, INPUT_PULLUP); // 音量ボタンピンを入力として設定
  
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(pwmFrequency);
}

//////////////////////////////
//           Loop 1         //
//////////////////////////////

void loop1() {
  float inputI, inputQ;
  float output = 0.0; // 出力の初期化

  inputI = analogRead(inputPinI);
  inputQ = analogRead(inputPinQ);

  inputI = (inputI / 2047.5) - 1.0;
  inputQ = (inputQ / 2047.5) - 1.0;

  // ヒルベルト変換を適用
  float hilbertOutput = applyFilter(output, hilbertCoeffs, buffer);

  output = inputI - inputQ;
  applyFilter(output, lsbFilterCoeffs, buffer);

  // AGCを適用
  float agcOutput = applyAGC(hilbertOutput);

  uint16_t pwmOutput = (uint16_t)(min((agcOutput + 1.0) * 2047.5, 4095.0));
  analogWrite(speakerPin, pwmOutput);

  delayMicroseconds(1000000 / sampleRate);
  
  // 音量ボタンが押されたか確認
  if (digitalRead(VOLUME_BUTTON_PIN) == LOW) {
    volumeMultiplier += 0.1;  // 音量を0.1増加
    if (volumeMultiplier > 1.9) {
      volumeMultiplier = 1.0; // 1.9を超えたら1.0にリセット
    }
    // デバウンス処理
    delay(200); 
  }
  
}
