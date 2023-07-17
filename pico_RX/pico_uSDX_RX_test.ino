//2023_5_11 JR3XNW
//pico RX uSDX Filters for each mode + AGC.

const int speakerPin = 14; //Audio output GP14 pico 19-pin
const int inputPinI = 26; //I Signal ADC0 pico31 pin
const int inputPinQ = 27; //Q signal ADC1 pico332 pin
const int sampleRate = 8000; //sample rate
const int pwmFrequency = 30000; //PWM20000
const int pushSwitchPin = 15; //Reception mode change push switch (LSB→USB→CW→AM)
const float targetAmplitude = 0.5;//0.5-0.8

const int numTaps = 31; //Number of filter taps

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

const float usbFilterCoeffs[numTaps] = {
 -1.57196597e-03, -1.45104171e-03,  1.12478657e-03,  4.46246491e-03,
  2.57714059e-03, -6.99319701e-03, -1.30293770e-02,  7.20713963e-18,
  2.47619628e-02,  2.56472800e-02, -1.88013680e-02, -6.76180323e-02,
 -3.71204042e-02,  1.08307805e-01,  2.91780827e-01,  3.75846238e-01,
  2.91780827e-01,  1.08307805e-01, -3.71204042e-02, -6.76180323e-02,
 -1.88013680e-02,  2.56472800e-02,  2.47619628e-02,  7.20713963e-18,
 -1.30293770e-02, -6.99319701e-03,  2.57714059e-03,  4.46246491e-03,
  1.12478657e-03, -1.45104171e-03, -1.57196597e-03
};

const float cwFilterCoeffs[numTaps] = {
 -6.47960382e-04, -1.44397905e-03, -2.70225797e-03, -4.44074475e-03,
 -6.19148454e-03, -6.95915901e-03, -5.37067609e-03,  2.39068677e-18,
  1.02068182e-02,  2.55224470e-02,  4.51695882e-02,  6.72889150e-02,
  8.91803923e-02,  1.07780639e-01,  1.20271316e-01,  1.24672294e-01,
  1.20271316e-01,  1.07780639e-01,  8.91803923e-02,  6.72889150e-02,
  4.51695882e-02,  2.55224470e-02,  1.02068182e-02,  2.39068677e-18,
 -5.37067609e-03, -6.95915901e-03, -6.19148454e-03, -4.44074475e-03,
 -2.70225797e-03, -1.44397905e-03, -6.47960382e-04
};

const float amFilterCoeffs[numTaps] = {
 -1.20126129e-03,  2.04889442e-03, -2.07510535e-03,  4.91080693e-18,
  4.75453597e-03, -9.87450755e-03,  9.95675888e-03, -1.43918829e-17,
 -1.89225390e-02,  3.62143751e-02, -3.46864198e-02,  2.48038628e-17,
  6.84829915e-02, -1.52932377e-01,  2.22972391e-01,  7.50524525e-01,
  2.22972391e-01, -1.52932377e-01,  6.84829915e-02,  2.48038628e-17,
 -3.46864198e-02,  3.62143751e-02, -1.89225390e-02, -1.43918829e-17,
  9.95675888e-03, -9.87450755e-03,  4.75453597e-03,  4.91080693e-18,
 -2.07510535e-03,  2.04889442e-03, -1.20126129e-03
};

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


float buffer[numTaps] = {0};
float agcGain = 1.0;

enum DemodulationMode { LSB, USB, CW, AM };
DemodulationMode currentMode = LSB;

void setup() {
  pinMode(25, OUTPUT);  // pico built-in LED

  pinMode(speakerPin, OUTPUT);
  pinMode(inputPinI, INPUT);
  pinMode(inputPinQ, INPUT);
  pinMode(pushSwitchPin, INPUT_PULLUP);

  analogReadResolution(12);
  analogWriteResolution(8);
  analogWriteFreq(pwmFrequency);
}

float applyHilbertTransform(float input, const float *coeffs, float *buffer) {
  memmove(&buffer[1], &buffer[0], (numTaps - 1) * sizeof(float));
  buffer[0] = input;
  float output = 0;
  for (int i = 0; i < numTaps; i++) {
    output += buffer[i] * coeffs[i];
  }
  return output;
}

float applyAGC(float input) {
  float error = targetAmplitude - abs(input);
  agcGain += 0.001 * error;
  if (agcGain < 0.1) {
    agcGain = 0.1;
  }
  return input * agcGain;
}

float applyLowPassFilter(float input, const float *coeffs, float *buffer) {
  memmove(&buffer[1], &buffer[0], (numTaps - 1) * sizeof(float));
  buffer[0] = input;
  float output = 0;
  for (int i = 0; i < numTaps; i++) {
    output += buffer[i] * coeffs[i];
  }
  return output;
}

void loop() {
  digitalWrite(25, HIGH);  // Built-in LED lights up during sampling

  float inputI, inputQ;
  float output = 0.0; // initialize output 

  inputI = analogRead(inputPinI);
  inputQ = analogRead(inputPinQ);

  inputI = (inputI / 2047.5) - 1.0;
  inputQ = (inputQ / 2047.5) - 1.0;

  if (digitalRead(pushSwitchPin) == LOW) {
    delay(50); // Debounce delay
    currentMode = static_cast<DemodulationMode>((currentMode + 1) % 4);
    while (digitalRead(pushSwitchPin) == LOW); // Wait for switch release
    delay(50); // Debounce delay
  }

  // Apply the Hilbert Transform
  float hilbertOutput = applyFilter(output, hilbertCoeffs, buffer);

  switch (currentMode) {
    case LSB:
      output = inputI - inputQ;
      applyFilter(output, lsbFilterCoeffs, buffer);
      break;
    case USB:
      output = inputI + inputQ;
      applyFilter(output, usbFilterCoeffs, buffer);
      break;
    case CW:
      output = inputI * inputQ;
      applyFilter(output, cwFilterCoeffs, buffer);
      break;
    case AM:
      output = sqrt(inputI * inputI + inputQ * inputQ);
      applyFilter(output, amFilterCoeffs, buffer);
      break;
  }

  // Apply AGC
  float agcOutput = applyAGC(hilbertOutput);
  uint8_t pwmOutput = (uint8_t)((agcOutput + 1.0) * 127.5);

  analogWrite(speakerPin, pwmOutput);
  delayMicroseconds(1000000 / sampleRate);
}
