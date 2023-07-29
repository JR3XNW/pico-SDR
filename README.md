#   Raspberry Pi pico version SDR radio lab.

##  Begin programming SDR radios with Raspberry Pi pico.

I have been enjoying various experiments by building uSDR and thought of adding a band scope to uSDX, but it is not very feasible to add that functionality to ATmega328, so I started to program a band scope as an external device using pico.

I started by experimenting with a small OLED (128x32) to fit the uSDX, designed to take IQ data off the uSDX TriBand board and display it.

##  Development environment

**Arduino IDE 2.1.1**

https://github.com/earlephilhower/arduino-pico
https://www.upesy.com/blogs/tutorials/install-raspberry-pi-pico-on-arduino-ide-software

To compile this firmware, use the https://github.com/earlephilhower/arduino-pico created by Earle F. Philhower, III in order to setup a build chain based on the Arduino IDE. Check for installation and configuration instructions at this https://www.upesy.com/blogs/tutorials/install-raspberry-pi-pico-on-arduino-ide-software.

**Libraries used**
- Arduino.h

- Rotary.h : Rotary encoder library for Arduino https://github.com/brianlow/Rotary

- U8g2lib.h : U8glib V2 library for Arduino https://github.com/olikraus/U8g2_Arduino

- Wire.h

- arduinoFFT.h : arduinoFFT – Arduino https://www.arduino.cc/reference/en/libraries/arduinofft/

- si5351.h : Etherkit Si5351 - Arduino Library https://github.com/etherkit/Si5351Arduino

## BandScope_128x32
**pico_BandScope_v1_128x32.ino**
This program is written in the Arduino programming language and is designed to run on an Raspberry Pi pico board. It uses various libraries, including the U8g2lib library for controlling an OLED display and the arduinoFFT library for performing Fast Fourier Transform (FFT) calculations.

The purpose of the program is to implement a band scope display using an Raspberry Pi pico board and an OLED display. The band scope displays the frequency spectrum of an input signal in real-time. It uses two input pins (I_IN and Q_IN) to read analog input samples from a signal source. The program includes a mode switching functionality using a button connected to the MOD_BUTTON pin.

Here's a brief overview of how the program works:

The necessary libraries are included at the beginning of the program.
Global variables are defined, including pin assignments, sample buffers, and display data arrays.
The setup function is called once at the start of the program. It initializes the pins, sets up the serial communication, initializes the OLED display, and displays an initial message on the screen.
The main loop function, loop(), is executed repeatedly.
Within the loop, it checks if the mode switching button (MOD_BUTTON) is pressed. If so, it calls the Mod_Stp() function to toggle the mode between 0 and 1.
Depending on the current mode, the program reads analog input samples from the designated pins (I_IN and Q_IN) into the sample buffers (vReal and vImag).
The FFT algorithm is applied to the sample buffers using the arduinoFFT library. The FFT is performed in reverse mode (FFT_REVERSE) to obtain the frequency domain representation.
The magnitude of the complex FFT results is calculated and stored in the vReal array.
The OLED display buffer is cleared, and the showScope(), showGraphics(), and show_mod() functions are called to update the display with the spectrum data and other graphical elements.
The display buffer is sent to the OLED display using u8g2.sendBuffer().
A small delay (1 millisecond) is added before repeating the loop to control the update rate.
The Mod_Stp() function is responsible for handling the mode switching functionality. It checks the current mode and toggles it between 0 and 1 when the mode switching button is pressed.

The showScope() function draws the spectrum display on the OLED screen. It calculates the length of the bars representing the spectrum using the barLength() function and draws the bars using the u8g2 library functions.

The show_mod() function displays the current mode (LSB or USB) on the screen.

The showGraphics() function draws the scale lines and other graphical elements on the screen.

Overall, the program continuously reads analog input samples, performs an FFT on the samples, and updates the OLED display with the spectrum data in real-time.

## BandScope_WaterFal128x64
**pico_BandScope_WaterFal_v1_128x64.ino**

This program is designed to control an OLED display and perform a spectrum analysis using a Fast Fourier Transform (FFT). It uses the following libraries:

U8g2lib.h: This library provides functions for controlling the OLED display.
Wire.h: This library enables I2C communication, which is used for communication with the OLED display.
arduinoFFT.h: This library is used to perform the Fast Fourier Transform.
Here's a breakdown of the main components and functionality of the program:

Global Variables and Definitions: This section defines various global variables, including pin assignments, sample sizes, display dimensions, and other parameters.

setup(): This function is called once when the program starts. It initializes the built-in LED, sets the mode switch button as an input with a pull-up resistor, initializes the serial communication, initializes the OLED display, and displays some information on the OLED screen.

loop(): This function is called repeatedly in a loop. It performs the following steps:

Checks if the mode switch button is pressed and calls the Mod_Stp() function to change the mode.
Reads analog input values for I and Q (in-phase and quadrature) signals and stores them in the vReal and vImag arrays.
Performs the FFT on the input signals using the arduinoFFT library.
Updates the OLED display by calling various functions to show the spectrum, waterfall display, mode, and other graphics.
Delays for 1 millisecond before repeating the loop.
Mod_Stp(): This function is called when the mode switch button is pressed. It toggles the mod variable between 0 and 1, representing the LSB (Lower Sideband) and USB (Upper Sideband) modes, respectively.

Display Functions: There are several functions that handle different aspects of the display:

showScope(): This function displays the spectrum on the OLED screen by drawing vertical lines representing the magnitudes of the FFT results.
show_mod(): This function displays the current mode (LSB or USB) on the OLED screen.
barLength(): This function calculates the length of a graph based on its magnitude.
displayWaterfall(): This function displays a waterfall plot on the OLED screen by drawing pixels based on the FFT magnitudes over time.
showGraphics(): This function draws various graphics on the OLED screen, including demarcation lines, frequency scales, and other indications.

## VFO_BandScope
**pico_uSDX_VFO_FFT_04_24.ino** 40M band

**pico_uSDX_VFO_FFT_20M.ino** 20M band

Library Inclusions:

#include <Arduino.h>: This is the main Arduino library that provides basic functions and definitions.

#include <Rotary.h>: A library for interfacing with rotary encoders.

#include <U8g2lib.h>: A library for using OLED displays.

#include <Wire.h>: The Wire library enables communication over I2C.

<arduinoFFT.h>: A library for performing Fast Fourier Transform (FFT) calculations.

<si5351.h>: A library for controlling the Si5351 clock generator.

**Constant Definitions:**

Various constant definitions are provided, including pin assignments, frequency limits, and step values.

**Variable Declarations:**

Global variables are declared, including the frequency values, rotary encoder object, and Si5351 object.

**Rotary Encoder and Button Handling:**

The rotary_encoder() function is an interrupt routine that is triggered when the rotary encoder is rotated. It adjusts the frequency value based on the rotation direction.
The Fnc_Stp() function is called when the step button is pressed. It changes the frequency step value.
These functions are called within the loop() function based on the state of the rotary encoder and step button.

**OLED Display Setup:**

The setup() function initializes the Arduino, sets up the rotary encoder interrupt, and initializes the OLED display.

**Main Loop:**

The loop() function is the main program loop that runs repeatedly.
It samples analog inputs (I_IN and Q_IN) using the analogRead() function and stores the values in arrays vReal and vImag.
The Fnc_Stp() function is called if the step button is pressed.
If the frequency value has changed, the Freq_Set() function is called to update the frequency on the Si5351 clock generator.
The FFT (Fast Fourier Transform) calculations are performed using the arduinoFFT library to convert the time domain signals to frequency domain signals.
The OLED display is updated by calling various functions (showScope(), showGraphics(), showS_meter()) that draw different elements on the display.
There is a delay of 1 millisecond at the end of each iteration of the loop.

**Additional Functions:**

The program includes several functions to handle specific tasks, such as frequency setting (Freq_Set()), displaying the spectrum on the OLED (showScope()), calculating bar lengths for the spectrum (barLength()), and displaying graphics and text on the OLED (showGraphics() and showS_meter()).
Note that this program assumes the availability of specific hardware components, such as a rotary encoder, Si5351 clock generator, and an OLED display. The program interacts with these components to control and display frequency-related information.

## pico_RX
**pico_BandScope_WaterFal_v1_128x64.ino**

The program starts by defining the necessary pin assignments and configuration parameters such as the speaker pin, input pins for I and Q signals, sample rate, PWM frequency, push switch pin, and target amplitude.

**Filter coefficients are defined for various demodulation modes:**

ilbert, LSB (Lower Sideband), USB (Upper Sideband), CW (Continuous Wave), and AM (Amplitude Modulation).

The applyFilter function is defined to apply a finite impulse response (FIR) filter on the input signal. It takes the input, filter coefficients, and a buffer as arguments. The function shifts the elements in the buffer and then performs a convolution operation to compute the filtered output.

The setup function is called to set the pin modes, analog read and write resolutions, and PWM frequency.

The applyHilbertTransform function is defined to apply the Hilbert transform on the input signal using the Hilbert filter coefficients. It performs a similar operation as the applyFilter function but with a different set of coefficients.

The applyAGC function is defined to apply Automatic Gain Control (AGC) on the input signal. It adjusts the gain based on the difference between the target amplitude and the absolute value of the input. The gain is updated gradually to avoid sudden changes.

The applyLowPassFilter function is defined to apply a low-pass filter on the input signal using the provided coefficients. It operates similarly to the applyFilter function.

The loop function is the main program loop that executes repeatedly.

The built-in LED (pin 25) is turned on to indicate the start of the sampling process.

The I and Q input signals are read from the analog pins (inputPinI and inputPinQ) and converted to the range of -1.0 to 1.0.

The push switch pin is checked to determine if the demodulation mode should be changed. If the switch is pressed, the current mode is incremented, and the switch is debounced. The mode is cycled through the four available options: LSB, USB, CW, and AM.

Based on the current demodulation mode, the input signals are processed accordingly using the appropriate filter coefficients. The output is stored in the 'output' variable.

The Hilbert transform is applied to the 'output' signal using the Hilbert filter coefficients. The result is stored in the 'hilbertOutput' variable.

AGC is applied to the 'hilbertOutput' to adjust the signal amplitude, and the resulting value is stored in the 'agcOutput' variable.

The 'agcOutput' value is converted to a PWM output value (0-255) and written to the speaker pin using the analogWrite function.

A delay is introduced to maintain the desired sample rate. The delay duration is calculated based on the reciprocal of the sample rate.

The loop continues, and the process repeats.

Overall, the program reads the I and Q signals, applies the selected demodulation mode, filters the signal using the corresponding filter coefficients, applies AGC, and outputs the resulting audio signal to the speaker. The demodulation mode can be changed by pressing a push switch.

## pico40M_monoband_radio
**pico40M_monoband_radio.ino**

This sketch is a summary of the functionality we have been experimenting with.
Using two cores of pico, we have band scope, waterfall and VFO on core 0 and receive functionality on core 1. In the future, we plan to add a transmit function to core 1.
Right now I am testing reception by connecting it to a direct line mixer that I have built in the past. There are still some problems: the rotary encoder that controls the VFO has jumps in rotation (the VFO lags behind the encoder operation) problems with reception sound, AGC, etc. I will continue to test and try to make it stable.
The variable range of the VFO is from 7.0MHz to 7.2MHz, and the variable amount is switched between 1kz, 500Hz, and 100Hz with a push switch on the rotary encoder.

## pico40M_monoband_uSDX
**pico40M_monoband_uSDX**

This folder contains a sketch to turn a uSDX Atmega328 into a Raspberry Pico, the sketch changes, and a wiring diagram.
The sketch modification is to change the frequency on line 57 to 64800000000ULL since the Si5351's reference frequency is 27MHz. This is an even multiple of the reference frequency (in this case 24 times) in the range of 600 MHz to 900 MHz in order to perform phase control.
Line 143 is the crystal frequency of the Si5351, which is 27000000 Hz.
If the frequency is higher than 7.1MHz, for example 7.101000, multiply the difference 1000 by 4 and add 4000 to 27000000.
27000000+1000x4=27004000.
If the outgoing frequency is lower, subtract it from 27000000. For example, if the outgoing frequency of VFO is 7.099 MHz,
27000000-1000x4=26996000. The frequency of the kanazuzu will not match the frequency once, but it can be matched after repeating a few times.
In the future, I would like to make it possible to match frequencies like uSDX.
One more thing: Currently, the variable amount of VFO (default setting 1KHz) cannot be changed. This is due to the different method of reading the push switch status from uSDX. I will improve it in the future too.
I will disclose the current status though it is still incomplete.

Next is the program for the transmitting part, I want to use the same method as uSDX.

**Circuit Changes in uSDX**

This is a change to replace the Raspberry Pi Pico with an Atmega328.
 I have included the schematic in the pico40M_monoband_uSDX folder, but have changed capacitors C17, C18, C22, and C23.
I experimented with 10n and 100n. 10n has a wider band scope range but a little worse selectivity on reception. 100n has a narrower band scope range but better selectivity.
Another change is to change resistors R10, R11, R12, and R13 to 51k. This change is due to the difference in input impedance between the Atmega328 and the Raspberry Pico.
Finally, we need to connect pins 4 and 27 of the Atmega328; pins 5 and 28 need to be connected.
I are still testing and will include a sketch upgrade in the future.

