#include <Adafruit_DotStar.h>

// Connections
const int detPin = A2;           // Analog signal from photodetector
const int laserPin = 7;          // Signal to laser MOSFET
const int ledPin = LED_BUILTIN;  // Built-in LED

// Resistor values
const float R1 = 6710.0; // Ohm
const float R2 = 3260.0; // Ohm

// Threshold and timeout
const float THRESHOLD = 3;     // Voltage threshold (V)
const unsigned long TIMEOUT = 20000; // Max measurement time in ms (20 seconds)

// State variables
bool measuring = false;
bool streaming = false;
bool laserOn = false;
bool belowThreshold = false; // Tracks current signal state
unsigned long startTime = 0;

// LED initialisation (todo: change strip to dot possible?)
Adafruit_DotStar strip(1, 8, 6, DOTSTAR_RGB);

void setup() {
  // Set up laser
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW); // Laser initially off

  // Set up analog read
  analogReadResolution(12);    // 12-bit ADC

  // Set up LED
  strip.begin(); // Initialize pins for output
  strip.setBrightness(100);
  strip.show();  // Turn all LEDs off ASAP

  // Set up serial connection
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect (works only on native USB)
  }

  delay(500); // Give the serial monitor time to attach (esp. Uno/Nano)
  Serial.println("Commands:");
  Serial.println("  'start'  - Begin measuring with laser and dip detection");
  Serial.println("  'stream' - Output signal continuously");
  Serial.println("  'stop'   - Stop measuring/streaming and turn off laser");
}


void loop() {
  // Handle Serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("start") && !measuring) {
      startTime = millis();
      digitalWrite(laserPin, HIGH);
      strip.setPixelColor(0, 255, 0, 0);
      strip.show();
      Serial.println("Laser ON. Measuring started...");
      delay(100); // Wait for laser to stabilize
      measuring = true;
      streaming = false;
      laserOn = true;
      belowThreshold = false; // Reset state
    }
    else if (command.equalsIgnoreCase("stream") && !streaming) {
      digitalWrite(laserPin, HIGH);
      strip.setPixelColor(0, 255, 0, 0);
      strip.show();
      Serial.println("Laser ON. Streaming started...");
      streaming = true;
      measuring = false;
      laserOn = true;
    }
    else if (command.equalsIgnoreCase("stop")) {
      measuring = false;
      streaming = false;
      laserOn = false;
      digitalWrite(laserPin, LOW);
      strip.setPixelColor(0, 0, 0, 0);
      strip.show();
      Serial.println("Laser OFF. Operation stopped.");
    }
  }

  // Read and convert signal
  int val = analogRead(detPin); // 0–4095
  float voltage = (val / 4095.0) * 3.3;
  float signalVoltage = voltage * ((R1 + R2) / R2);

  // Stream mode: just print the signal continuously
  if (streaming) {
    Serial.println(signalVoltage, 3);
    delay(10); // Optional: limit output speed

    // Falling edge: signal drops below threshold
    if (!belowThreshold && signalVoltage < THRESHOLD) {
      Serial.print("DROPLET!!");
      strip.setPixelColor(0, 0, 255, 0);
      strip.show();
      belowThreshold = true;
    }

    // Rising edge: signal rises back above or equal to threshold
    else if (belowThreshold && signalVoltage >= THRESHOLD) {
      Serial.println("...and it's gone.");
      strip.setPixelColor(0, 255, 0, 0);
      strip.show();
      belowThreshold = false;
    }

    return;
  }

  // Measurement mode: edge detection
  if (measuring) {
    unsigned long now = millis();

    // Stop if timeout exceeded
    if (now - startTime > TIMEOUT) {
      measuring = false;
      laserOn = false;
      digitalWrite(laserPin, LOW);
      strip.setPixelColor(0, 0, 0, 0);
      strip.show();
      Serial.println("Laser OFF. Timeout reached. Measurement ended.");
      return;
    }

    // Falling edge: signal drops below threshold
    if (!belowThreshold && signalVoltage < THRESHOLD) {
      Serial.print("DROPLET!!");
      strip.setPixelColor(0, 0, 255, 0);
      strip.show();
      belowThreshold = true;
    }

    // Rising edge: signal rises back above or equal to threshold
    else if (belowThreshold && signalVoltage >= THRESHOLD) {
      Serial.println("...and it's gone.");
      strip.setPixelColor(0, 255, 0, 0);
      strip.show();
      belowThreshold = false;
    }
  }
}
