#include <MozziGuts.h>
#include <Oscil.h>
#include <LowPassFilter.h>
#include <ADSR.h>
#include <tables/sin2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/square_no_alias2048_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/supersaw2048_int8.h>
#include <tables/random_smooth2048_int8.h>
#include <tables/random_stepped2048_int8.h>

// Oscillators and Filters
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> osc1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> osc2(SIN2048_DATA);

LowPassFilter filter1;
LowPassFilter filter2;

// ADSR Envelopes for amplitude and filters
ADSR ampEnv1(50, 200, 0.7, 300);  // ADSR for amplitude of voice 1
ADSR ampEnv2(50, 200, 0.7, 300);  // ADSR for amplitude of voice 2
ADSR filterEnv1(50, 200, 0.7, 300);  // ADSR for filter 1
ADSR filterEnv2(50, 200, 0.7, 300);  // ADSR for filter 2

// Rungler Variables
uint8_t shiftRegister1 = 0, shiftRegister2 = 0;
int steppedCV1 = 0, steppedCV2 = 0;

// User-selected output options
bool useUSB = false;  // Flag to select USB audio or DAC output

// USB Audio Output (PCM data over serial)
void sendAudioToUSB(int audioData) {
  Serial.write(audioData >> 8);   // Send high byte
  Serial.write(audioData & 0xFF); // Send low byte
}

// External DAC Output
void sendAudioToDAC(int audioData) {
  dacWrite(25, audioData);  // Send to DAC (GPIO 25 for DAC1 on ESP32)
}

// Control Inputs (Potentiometers and Patch Points)
const int potMod1 = 36;    // Modulation amount for voice 1 (GPIO 36)
const int potMod2 = 39;    // Modulation amount for voice 2 (GPIO 39)
const int potWaveShape1 = 34; // Wave shaping for voice 1 (GPIO 34)
const int potWaveShape2 = 35; // Wave shaping for voice 2 (GPIO 35)
const int potTuning1 = 32;  // Coarse tuning for voice 1 (GPIO 32)
const int potTuning2 = 33;  // Coarse tuning for voice 2 (GPIO 33)
const int potFM1 = 26;      // Frequency modulation amount for oscillator 1 (GPIO 26)
const int potFM2 = 27;      // Frequency modulation amount for oscillator 2 (GPIO 27)

// ADSR envelope patch points
const int patchAmpEnv1 = 14;  // External control for amplitude envelope of voice 1 (GPIO 14)
const int patchAmpEnv2 = 12;  // External control for amplitude envelope of voice 2 (GPIO 12)
const int patchFilterEnv1 = 13;  // External control for filter envelope 1 (GPIO 13)
const int patchFilterEnv2 = 15;  // External control for filter envelope 2 (GPIO 15)

// Filter cutoff, resonance, and distortion patch points and pots
const int potCutoff1 = 2;      // Potentiometer for filter cutoff of voice 1 (GPIO 2)
const int potResonance1 = 4;   // Potentiometer for resonance of voice 1 (GPIO 4)
const int potDistortion1 = 16;  // Potentiometer for distortion of voice 1 (GPIO 16)
const int potCutoff2 = 17;      // Potentiometer for filter cutoff of voice 2 (GPIO 17)
const int potResonance2 = 5;    // Potentiometer for resonance of voice 2 (GPIO 5)
const int potDistortion2 = 18;  // Potentiometer for distortion of voice 2 (GPIO 18)

// Rungler control
const int potRunglerSpeed1 = 19;  // Potentiometer for controlling Rungler speed 1 (GPIO 19)
const int potRunglerSpeed2 = 21;  // Potentiometer for controlling Rungler speed 2 (GPIO 21)

void setup() {
  Serial.begin(115200); // Initialize USB serial communication
  startMozzi();         // Initialize Mozzi lib
  osc1.setFreq(440);    // Initial frequency for osc1
  osc2.setFreq(220);    // Initial frequency for osc2
}

void updateControl() {
  // Run Rungler for each oscillator
  steppedCV1 = runglerStep(osc2.next(), shiftRegister1);
  steppedCV2 = runglerStep(osc1.next(), shiftRegister2);
  
  osc1.setFreq(440 + steppedCV1);
  osc2.setFreq(220 + steppedCV2);

  // External tuning patch points with V/Oct
  float tuning1 = (analogRead(potTuning1) > 0) ? calculateVOctTuning(analogRead(potTuning1), 440) : analogRead(potTuning1);
  float tuning2 = (analogRead(potTuning2) > 0) ? calculateVOctTuning(analogRead(potTuning2), 220) : analogRead(potTuning2);

  // Frequency modulation control from potentiometers
  int fmAmount1 = analogRead(potFM1) / 4;
  int fmAmount2 = analogRead(potFM2) / 4;

  // External control for ADSR envelopes
  if (analogRead(patchAmpEnv1) > 0) ampEnv1.setSustain(analogRead(patchAmpEnv1) / 4095.0);
  if (analogRead(patchAmpEnv2) > 0) ampEnv2.setSustain(analogRead(patchAmpEnv2) / 4095.0);
  if (analogRead(patchFilterEnv1) > 0) filterEnv1.setSustain(analogRead(patchFilterEnv1) / 4095.0);
  if (analogRead(patchFilterEnv2) > 0) filterEnv2.setSustain(analogRead(patchFilterEnv2) / 4095.0);

  // Filter cutoff, resonance, and distortion with potentiometer overrides
  int cutoff1 = analogRead(potCutoff1) / 4;
  int resonance1 = analogRead(potResonance1) / 4;
  int distortion1 = analogRead(potDistortion1) / 4;

  int cutoff2 = analogRead(potCutoff2) / 4;
  int resonance2 = analogRead(potResonance2) / 4;
  int distortion2 = analogRead(potDistortion2) / 4;

  // Apply filter modulation via ADSR envelopes and external controls
  filter1.setCutoffFreq(500 + filterEnv1.getOutput() * cutoff1);
  filter2.setCutoffFreq(500 + filterEnv2.getOutput() * cutoff2);

  // Set filter resonance
  filter1.setResonance(resonance1 / 1023.0);
  filter2.setResonance(resonance2 / 1023.0);
}

int runglerStep(int oscillatorOutput, uint8_t &shiftRegister) {
  shiftRegister <<= 1;
  if (oscillatorOutput > 0) shiftRegister |= 1;
  return (shiftRegister & 0b111) * 128;  // Generate stepped CV
}

// Audio output function for DAC or USB
void processAudioOutput(int mixedSignal) {
  if (useUSB) {
    sendAudioToUSB(mixedSignal);  // Send audio via USB
  } else {
    sendAudioToDAC(mixedSignal);  // Send audio via DAC
  }
}

// Update audio output
int updateAudio() {
  int audioSignal1 = osc1.next() * ampEnv1.next();
  int audioSignal2 = osc2.next() * ampEnv2.next();
  int mixedSignal = (audioSignal1 + audioSignal2) / 2;
  
  processAudioOutput(mixedSignal);  // Send audio to the selected output
  return mixedSignal;
}

// Loop function to handle user input for selecting output method
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "USB") {
      useUSB = true;
      Serial.println("Using USB audio output");
    } else if (command == "DAC") {
      useUSB = false;
      Serial.println("Using DAC output");
    }
  }
  audioHook();  // Required for Mozzi to keep processing audio
}