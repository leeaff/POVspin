#include <Arduino.h>
#include "driver/pcnt.h"
#include <math.h>

// -----------------------
// Pin and display config
// -----------------------
#define OUTPUT_BIT0 8
#define OUTPUT_BIT1 7
#define OUTPUT_BIT2 5
#define OUTPUT_BIT3 10

#define ENC_A 32
#define ENC_B 33
#define CPR 2400
#define PCNT_UNIT_USED PCNT_UNIT_0

#define NUM_LEDS 15
#define NUM_COLUMNS 50
#define COLUMN_DURATION_US 500

// -----------------------
// Matrices
// -----------------------
bool displayMatrix[NUM_LEDS][NUM_COLUMNS];
bool incomingMatrix[NUM_LEDS][NUM_COLUMNS];

// -----------------------
// Angle tracking variables
// -----------------------
int16_t lastRawCount = 0;
int32_t totalCount = 0;  // Tracks continuous rotation without wrapping

// -----------------------
// Encoder setup
// -----------------------
void setupEncoder() {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = ENC_A,
    .ctrl_gpio_num  = ENC_B,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_INC,
    .neg_mode       = PCNT_COUNT_DEC,
    .counter_h_lim  = 32767,
    .counter_l_lim  = -32768,
    .unit           = PCNT_UNIT_USED,
    .channel        = PCNT_CHANNEL_0
  };
  
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT_USED);
  pcnt_counter_clear(PCNT_UNIT_USED);
  pcnt_counter_resume(PCNT_UNIT_USED);
}

// -----------------------
// POV display helpers
// -----------------------
void outputBinaryValue(uint8_t ledNumber) {
  if (ledNumber > NUM_LEDS) ledNumber = 0;
  digitalWrite(OUTPUT_BIT0, ledNumber & 0x01);
  digitalWrite(OUTPUT_BIT1, (ledNumber & 0x02) >> 1);
  digitalWrite(OUTPUT_BIT2, (ledNumber & 0x04) >> 2);
  digitalWrite(OUTPUT_BIT3, (ledNumber & 0x08) >> 3);
}

void displayColumn(int columnIndex, unsigned long durationUs) {
  if (columnIndex < 0 || columnIndex >= NUM_COLUMNS) return;
  
  unsigned long startMicros = micros();
  unsigned long endMicros = startMicros + durationUs;
  
  while (micros() < endMicros) {
    for (int led = 0; led < NUM_LEDS; led++) {
      if (displayMatrix[led][columnIndex]) {
        outputBinaryValue(led);
        delayMicroseconds(5);
      }
      if (micros() >= endMicros) break;
    }
  }
  outputBinaryValue(0);
}

float getCurrentAngle() {
  int16_t rawCount = 0;
  pcnt_get_counter_value(PCNT_UNIT_USED, &rawCount);
  
  // Track accumulated count to handle wrapping
  int16_t delta = rawCount - lastRawCount;
  
  // Handle counter wrapping (shouldn't happen often with 16-bit, but just in case)
  if (delta > 16384) {
    delta -= 32768;
  } else if (delta < -16384) {
    delta += 32768;
  }
  
  totalCount += delta;
  lastRawCount = rawCount;
  
  // Apply calibration
  int32_t calibratedCount = (int32_t)(totalCount * 5.911);
  
  // Calculate angle within 0-360 range
  float angle = fmod((float)calibratedCount, (float)CPR) * (360.0 / CPR);
  if (angle < 0) angle += 360.0;
  
  return angle;
}

// -----------------------
// Serial reception
// -----------------------
String rowBuffer = "";
int rowsReceived = 0;
bool newMatrixReady = false;

void processRow(String rowData, int rowIndex) {
  int col = 0;
  int start = 0;
  for (int i = 0; i <= rowData.length(); i++) {
    if (i == rowData.length() || rowData[i] == ',') {
      int bit = rowData.substring(start, i).toInt();
      if (col < NUM_COLUMNS) incomingMatrix[rowIndex][col] = (bit == 1);
      col++;
      start = i + 1;
    }
  }
}

// -----------------------
// Setup
// -----------------------
void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("READY");
  
  // Output pins
  pinMode(OUTPUT_BIT0, OUTPUT);
  pinMode(OUTPUT_BIT1, OUTPUT);
  pinMode(OUTPUT_BIT2, OUTPUT);
  pinMode(OUTPUT_BIT3, OUTPUT);
  outputBinaryValue(0);
  
  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  setupEncoder();
  
  // Initialize lastRawCount
  pcnt_get_counter_value(PCNT_UNIT_USED, &lastRawCount);
  totalCount = 0;
  
  // Initialize matrices
  for (int r = 0; r < NUM_LEDS; r++)
    for (int c = 0; c < NUM_COLUMNS; c++)
      displayMatrix[r][c] = incomingMatrix[r][c] = false;
  
  // Test pattern
  for (int c = 0; c < NUM_COLUMNS; c++) {
    displayMatrix[14][c] = true;
  }
  
  Serial.println("POV Display Ready - Stable Angle Tracking Enabled");
}

// -----------------------
// Main loop
// -----------------------
void loop() {
  // Non-blocking serial reception
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processRow(rowBuffer, rowsReceived);
      rowBuffer = "";
      rowsReceived++;
      if (rowsReceived >= NUM_LEDS) {
        newMatrixReady = true;
        rowsReceived = 0;
      }
    } else {
      rowBuffer += c;
    }
  }
  
  // Apply new matrix if fully received
  if (newMatrixReady) {
    for (int r = 0; r < NUM_LEDS; r++)
      for (int c = 0; c < NUM_COLUMNS; c++)
        displayMatrix[r][c] = incomingMatrix[r][c];
    newMatrixReady = false;
    Serial.println("Matrix updated");
  }
  
  // Display current column
  float angle = getCurrentAngle();
  int columnIndex = (int)(angle / 360.0 * NUM_COLUMNS);
  if (columnIndex >= NUM_COLUMNS) columnIndex = NUM_COLUMNS - 1;
  
  displayColumn(columnIndex, COLUMN_DURATION_US);
}