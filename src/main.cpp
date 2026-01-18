#include <Arduino.h>
#include "driver/pcnt.h"
#include <math.h>

// Binary output pins (these go to your external LED decoder circuit)
#define OUTPUT_BIT0 8   // LSB - Change to your actual output pins
#define OUTPUT_BIT1 7   // Middle bit
#define OUTPUT_BIT2 5   // MSB

// Encoder Pins
#define ENC_A 32
#define ENC_B 33
#define CPR 2400
#define PCNT_UNIT_USED PCNT_UNIT_0

// POV Display Configuration
#define NUM_LEDS 8  // Your external circuit controls 8 LEDs (3 bits = 8 combinations)
#define NUM_COLUMNS 360  // Number of angular positions (1 degree resolution)
#define COLUMN_DURATION_US 500  // Time to display each column in microseconds

// Display matrix: 8 virtual LEDs × NUM_COLUMNS
// Each value 0-7 represents which LED should be on
uint8_t displayMatrix[NUM_COLUMNS];

int16_t lastCount = 0;
unsigned long lastTime = 0;

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

// Output 3-bit value to control which LED is on
// ledNumber: 0-7 (which of the 8 LEDs to turn on)
void outputBinaryValue(uint8_t ledNumber) {
  if (ledNumber > 7) ledNumber = 0;
  
  // Extract bit 0, bit 1, and bit 2
  bool bit0 = ledNumber & 0x01;  // Least significant bit
  bool bit1 = (ledNumber & 0x02) >> 1;  // Middle bit
  bool bit2 = (ledNumber & 0x04) >> 2;  // Most significant bit
  
  // Output to pins
  digitalWrite(OUTPUT_BIT0, bit0);
  digitalWrite(OUTPUT_BIT1, bit1);
  digitalWrite(OUTPUT_BIT2, bit2);
}

// Load a pattern matrix into the display matrix
// inputMatrix: array of NUM_COLUMNS values (0-7), each representing which LED to turn on at that angle
void loadMatrix(const uint8_t inputMatrix[NUM_COLUMNS]) {
  for (int i = 0; i < NUM_COLUMNS; i++) {
    displayMatrix[i] = inputMatrix[i];
  }
}

// Display one column value for specified duration
// columnIndex: which column (0 to NUM_COLUMNS-1)
// durationUs: how long to display this column in microseconds
void displayColumn(int columnIndex, unsigned long durationUs) {
  if (columnIndex < 0 || columnIndex >= NUM_COLUMNS) return;
  
  // Get which LED should be on for this column
  uint8_t ledValue = displayMatrix[columnIndex];
  
  // Output the binary value
  outputBinaryValue(ledValue);
  
  // Hold for the specified duration
  delayMicroseconds(durationUs);
}

float getCurrentAngle() {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_USED, &count);
  
  count = (int16_t)(count * 5.911);  // Apply your calibration factor
  
  float angle = fmod((float)count, (float)CPR) * (360.0 / CPR);
  if (angle < 0) angle += 360.0;
  
  return angle;
}

void setup() {
  // Serial.begin(115200);
  Serial.begin(9600);
  delay(1000);
  
  // Setup binary output pins (3 pins for 8 LEDs)
  pinMode(OUTPUT_BIT0, OUTPUT);
  pinMode(OUTPUT_BIT1, OUTPUT);
  pinMode(OUTPUT_BIT2, OUTPUT);
  digitalWrite(OUTPUT_BIT0, LOW);
  digitalWrite(OUTPUT_BIT1, LOW);
  digitalWrite(OUTPUT_BIT2, LOW);
  
  // Setup encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  setupEncoder();
  
  // Create example test pattern
  uint8_t testPattern[NUM_COLUMNS];
  
  // Pattern 1: 8 segments (45 degrees each)
  for (int col = 0; col < NUM_COLUMNS; col++) {
    if (col < 45) {
      testPattern[col] = 0;  // LED 0 (binary 000)
    } else if (col < 90) {
      testPattern[col] = 1;  // LED 1 (binary 001)
    } else if (col < 135) {
      testPattern[col] = 2;  // LED 2 (binary 010)
    } else if (col < 180) {
      testPattern[col] = 3;  // LED 3 (binary 011)
    } else if (col < 225) {
      testPattern[col] = 4;  // LED 4 (binary 100)
    } else if (col < 270) {
      testPattern[col] = 5;  // LED 5 (binary 101)
    } else if (col < 315) {
      testPattern[col] = 6;  // LED 6 (binary 110)
    } else {
      testPattern[col] = 7;  // LED 7 (binary 111)
    }
  }
  
  // Load the test pattern into display matrix
  loadMatrix(testPattern);
  
  lastTime = millis();
  Serial.println("POV Display Ready - Motor should be spinning at ~5 Hz");
  Serial.println("Using 3-bit output (8 LEDs): BIT0=" + String(OUTPUT_BIT0) + 
                 ", BIT1=" + String(OUTPUT_BIT1) + ", BIT2=" + String(OUTPUT_BIT2));
}

void loop() {
  // // Get current angle
  // float angle = getCurrentAngle();
  
  // // Convert angle to column index (0 to NUM_COLUMNS-1)
  // int columnIndex = (int)angle;
  // if (columnIndex >= NUM_COLUMNS) columnIndex = NUM_COLUMNS - 1;
  
  // // Display the current column
  // displayColumn(columnIndex, COLUMN_DURATION_US);

  // if (Serial.available()) {
  //   String data = Serial.readStringUntil('\n');
  //   Serial.println("Received:");
  //   Serial.println(data);

  //   // parse matrix here
  // }
  
  // Debug output (comment out for better performance)
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 500) {
  //   Serial.print("Angle: ");
  //   Serial.print(angle, 1);
  //   Serial.print("° | Column: ");
  //   Serial.print(columnIndex);
  //   Serial.print(" | LED value: ");
  //   Serial.println(displayMatrix[columnIndex]);
  //   lastPrint = millis();
  // }

  // Check for incoming serial data to update matrix
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();  // Remove whitespace
    
    Serial.println("Received: " + data);
    
    // Parse matrix data
    // Expected format: comma-separated values, 360 numbers (0-7)
    // Example: "0,1,2,3,4,5,6,7,0,1,2,..."
    
    uint8_t newPattern[NUM_COLUMNS];
    int colIndex = 0;
    int startPos = 0;
    
    // Parse comma-separated values
    for (int i = 0; i <= data.length(); i++) {
      if (i == data.length() || data[i] == ',') {
        if (colIndex >= NUM_COLUMNS) break;
        
        String valueStr = data.substring(startPos, i);
        valueStr.trim();
        int value = valueStr.toInt();
        
        // Validate value is 0-7
        if (value < 0) value = 0;
        if (value > 7) value = 7;
        
        newPattern[colIndex] = (uint8_t)value;
        colIndex++;
        startPos = i + 1;
      }
    }
    
    // If we received valid data, load it
    if (colIndex == NUM_COLUMNS) {
      loadMatrix(newPattern);
      Serial.println("Matrix updated successfully!");
    } else {
      Serial.print("Error: Expected 360 values, got ");
      Serial.println(colIndex);
    }
  }
  
  // Get current angle
  float angle = getCurrentAngle();
  
  // Convert angle to column index (0 to NUM_COLUMNS-1)
  int columnIndex = (int)angle;
  if (columnIndex >= NUM_COLUMNS) columnIndex = NUM_COLUMNS - 1;
  
  // Display the current column
  displayColumn(columnIndex, COLUMN_DURATION_US);
  
  // Debug output (comment out for better performance)
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 500) {
  //   Serial.print("Angle: ");
  //   Serial.print(angle, 1);
  //   Serial.print("° | Column: ");
  //   Serial.print(columnIndex);
  //   Serial.print(" | LED value: ");
  //   Serial.println(displayMatrix[columnIndex]);
  //   lastPrint = millis();
  // }
}