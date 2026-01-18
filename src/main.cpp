#include <Arduino.h>
#include "driver/pcnt.h"
#include <math.h>

#define OUTPUT_BIT0 8  
#define OUTPUT_BIT1 7   
#define OUTPUT_BIT2 5 

#define ENC_A 32
#define ENC_B 33
#define CPR 2400
#define PCNT_UNIT_USED PCNT_UNIT_0

#define NUM_LEDS 8  
#define NUM_COLUMNS 360 
#define COLUMN_DURATION_US 500  

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

void setDisplayMatrix(bool inputMatrix[NUM_LEDS][NUM_COLUMNS]) {
  for (int row = 0; row < NUM_LEDS; row++) {
    for (int col = 0; col < NUM_COLUMNS; col++) {
      displayMatrix[row][col] = inputMatrix[row][col];
    }
  }
}

void outputBinaryValue(uint8_t ledNumber) {
  if (ledNumber > 7) ledNumber = 0;
  
  bool bit0 = ledNumber & 0x01;        // Least significant bit
  bool bit1 = (ledNumber & 0x02) >> 1; // Middle bit
  bool bit2 = (ledNumber & 0x04) >> 2; // Most significant bit
  
  // Output to pins
  digitalWrite(OUTPUT_BIT0, bit0);
  digitalWrite(OUTPUT_BIT1, bit1);
  digitalWrite(OUTPUT_BIT2, bit2);
}

void displayColumn(int columnIndex, unsigned long durationUs) {
  if (columnIndex < 0 || columnIndex >= NUM_COLUMNS) return;
  
  unsigned long startMicros = micros();
  unsigned long endMicros = startMicros + durationUs;
  
  int iterationsPerLED = max(1, (int)(durationUs / (NUM_LEDS * 10))); 
  
  while (micros() < endMicros) {
    for (int led = 0; led < NUM_LEDS; led++) {
      if (displayMatrix[led][columnIndex]) {
        outputBinaryValue(led);
        delayMicroseconds(10); 
      }
      
      if (micros() >= endMicros) break;
    }
  }
  
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
  for (int col = 0; col < 360; col++) {
    myPattern[0][col] = true;  // LED 0 ON
    myPattern[1][col] = true;  // LED 0 ON
    myPattern[2][col] = true;  // LED 0 ON
    myPattern[3][col] = true;  // LED 0 ON
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

    int MATRIX_COLS = 100;

    bool newPattern[8][MATRIX_COLS];

    // Initialize pattern to all false
    for (int r = 0; r < 8; r++) {
      for (int c = 0; c < MATRIX_COLS; c++) {
        newPattern[r][c] = false;
      }
    }

    int rowsReceived = 0;

    while (rowsReceived < 8) {
      unsigned long startWait = millis();
      while (!Serial.available() && (millis() - startWait) < 1000) {
        delay(10)
      }

      if (!Serial.available()) {
        Serial.println("Timeout waiting for data");
        break;
      }

      String rowData = Serial.readStringUntil('\n');
      rowData.trim();

      if (rowData.length() == 0) continue;

      int colIndex = 0;
      int startPos = 0;

      for (int i = 0; i <= rowData.length(); i++) {
        if (i == rowData.length() || rowData[i] == ',')
      }
    }
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

