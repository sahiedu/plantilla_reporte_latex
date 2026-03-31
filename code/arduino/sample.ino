/* -------------------------------------------------------
   Arduino Style Test (Minimal Directives)
   Contains:
   - Basic Arduino functions
   - A couple of #defines
   - One #include
   - LED toggle + analog read
------------------------------------------------------- */

#include <Arduino.h>     // Basic Arduino library

#define LED_PIN 9        // LED output pin
#define POT_PIN A0       // Potentiometer analog input pin

unsigned long lastTime = 0;
const unsigned long interval = 500;   // LED toggle interval in ms

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);

    Serial.println("Arduino minimal directive test started.");
}

void loop() {
    // Toggle LED every 500 ms
    if (millis() - lastTime >= interval) {
        toggleLED();
        lastTime = millis();
    }

    // Read analog value
    int value = analogRead(POT_PIN);
    Serial.print("Analog: ");
    Serial.println(value);

    delay(50);
}

// Toggle LED state
void toggleLED() {
    static bool state = LOW;
    state = !state;
    digitalWrite(LED_PIN, state ? HIGH : LOW);
}
