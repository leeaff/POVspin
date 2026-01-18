#include <Arduino.h>
#include "driver/pcnt.h"
#include <math.h>

// Binary output pins (these go to your external LED decoder circuit)
#define OUTPUT_BIT0 8   // Change to your actual output pins
#define OUTPUT_BIT1 7   // Change to your actual output pins
#define OUTPUT_BIT2 5   // Third bit for 8 LEDs

// Encoder Pins
#define ENC_A 32
#define ENC_B 33
#define CPR 2400
#define PCNT_UNIT_USED PCNT_UNIT_0

// POV Display Configuration
#define NUM_LEDS 8  // Your external circuit controls 8 LEDs (3 bits = 2^3 = 8)
#define NUM_COLUMNS 100  // Number of angular positions (1 degree resolution)
#define COLUMN_DURATION_US 500  // Time to display each column in microseconds

// Display matrix: 8 rows (LEDs) × NUM_COLUMNS
// matrix[row][column] where row = LED number (0-7), column = angle position (0-359)
bool displayMatrix[NUM_LEDS][NUM_COLUMNS];

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

// Set the display matrix from input
// inputMatrix: 8 rows × NUM_COLUMNS array
void setDisplayMatrix(bool inputMatrix[NUM_LEDS][NUM_COLUMNS]) {
  for (int row = 0; row < NUM_LEDS; row++) {
    for (int col = 0; col < NUM_COLUMNS; col++) {
      displayMatrix[row][col] = inputMatrix[row][col];
    }
  }
}

// Output 3-bit value to control which LED is on
// ledNumber: 0-7 (which of the 8 LEDs to turn on)
void outputBinaryValue(uint8_t ledNumber) {
  if (ledNumber > 7) ledNumber = 0;
  
  // Extract bit 0, bit 1, and bit 2
  bool bit0 = ledNumber & 0x01;        // Least significant bit
  bool bit1 = (ledNumber & 0x02) >> 1; // Middle bit
  bool bit2 = (ledNumber & 0x04) >> 2; // Most significant bit
  
  // Output to pins
  digitalWrite(OUTPUT_BIT0, bit0);
  digitalWrite(OUTPUT_BIT1, bit1);
  digitalWrite(OUTPUT_BIT2, bit2);
}

// Display one column using multiplexing
// columnIndex: which column (0 to NUM_COLUMNS-1)
// durationUs: how long to display this column in microseconds
void displayColumn(int columnIndex, unsigned long durationUs) {
  if (columnIndex < 0 || columnIndex >= NUM_COLUMNS) return;
  
  unsigned long startMicros = micros();
  unsigned long endMicros = startMicros + durationUs;
  
  // Calculate how many iterations we can do
  // We want to cycle through all 8 LEDs multiple times for persistence of vision
  int iterationsPerLED = max(1, (int)(durationUs / (NUM_LEDS * 10))); // ~10us per LED minimum
  
  while (micros() < endMicros) {
    // Cycle through all 8 LEDs
    for (int led = 0; led < NUM_LEDS; led++) {
      if (displayMatrix[led][columnIndex]) {
        // This LED should be ON at this column
        outputBinaryValue(led);
        delayMicroseconds(10); // Brief pulse
      }
      
      // Check if time is up
      if (micros() >= endMicros) break;
    }
  }
  
  // Turn off all LEDs after column display
  outputBinaryValue(0);
  digitalWrite(OUTPUT_BIT0, LOW);
  digitalWrite(OUTPUT_BIT1, LOW);
  digitalWrite(OUTPUT_BIT2, LOW);
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
  Serial.begin(9600);
  delay(1000);
  
  // Setup binary output pins
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
  
  // Initialize display matrix to all off
  for (int row = 0; row < NUM_LEDS; row++) {
    for (int col = 0; col < NUM_COLUMNS; col++) {
      displayMatrix[row][col] = false;
    }
  }
  
  // Example: Create a test pattern matrix and load it
  bool myPattern[NUM_LEDS][NUM_COLUMNS];
  

  // Initialize to all off
  for (int row = 0; row < NUM_LEDS; row++) {
    for (int col = 0; col < NUM_COLUMNS; col++) {
      myPattern[row][col] = false;
    }
  }

  // Set your pattern - example: LED 0 and LED 3 on from 0-180°
  for (int col = 0; col < 100; col++) {
    myPattern[0][col] = true;  // LED 0 ON
    myPattern[1][col] = false;  // LED 0 ON
    myPattern[2][col] = true;  // LED 0 ON
    myPattern[3][col] = false;  // LED 0 ON
  }

  // Load it into the display
  setDisplayMatrix(myPattern);
    
  lastTime = millis();
  Serial.println("POV Display Ready - Motor should be spinning at ~5 Hz");
  Serial.println("Using 8 LEDs with 3 output bits");
  Serial.println("Output pins: BIT0=" + String(OUTPUT_BIT0) + ", BIT1=" + String(OUTPUT_BIT1) + ", BIT2=" + String(OUTPUT_BIT2));
}

void loop() {
  if (Serial.available()) {
    Serial.println("Receiving matrix data...");

    bool newPattern[NUM_LEDS][NUM_COLUMNS] = { false };

    // // Initialize pattern to all false
    // for (int r = 0; r < NUM_LEDS; r++) {
    //   for (int c = 0; c < NUM_COLUMNS; c++) {
    //     newPattern[r][c] = false;
    //   }
    // }

    int rowsReceived = 0;

    while (rowsReceived < NUM_LEDS) {
      unsigned long startWait = millis();
      while (!Serial.available() && (millis() - startWait) < 1000) {
        delay(10);
      }

      if (!Serial.available()) {
        Serial.println("Timeout waiting for data");
        break;
      }

      String rowData = Serial.readStringUntil('\n');
      rowData.trim();
      if (rowData.length() == 0) continue;

      Serial.print("Row ");
      Serial.print(rowsReceived);
      Serial.print(": ");
      Serial.println(rowData);

      int col = 0;
      int start = 0;

      for (int i = 0; i <= rowData.length(); i++) {
        if (i == rowData.length() || rowData[i] == ',') {
          int bit = rowData.substring(start, i).toInt();
          if (col < NUM_COLUMNS) {
            newPattern[rowsReceived][col] = (bit == 1);
          }
          col++;
          start = i + 1;
        }
      }

      rowsReceived++;
    }

    setDisplayMatrix(newPattern);
  }
  // Get current angle
  float angle = getCurrentAngle();
  
  // Convert angle to column index (0 to NUM_COLUMNS-1)
  int columnIndex = (int)angle;
  if (columnIndex >= NUM_COLUMNS) columnIndex = NUM_COLUMNS - 1;
  
  // Display the current column
  displayColumn(columnIndex, COLUMN_DURATION_US);
  
  // Debug output (uncomment to see status)
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 500) {
  //   Serial.print("Angle: ");
  //   Serial.print(angle, 1);
  //   Serial.print("° | Column: ");
  //   Serial.println(columnIndex);
  //   lastPrint = millis();
  // }
}

