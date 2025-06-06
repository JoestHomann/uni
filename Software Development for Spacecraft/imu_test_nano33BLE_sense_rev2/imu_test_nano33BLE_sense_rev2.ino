#include <Arduino_BMI270_BMM150.h>

const int ledPin = LED_BUILTIN;
const float motionThreshold = 1.2;  // alles über ~1g (Erdbeschl.) gilt als Bewegung
unsigned long lastBlink = 0;
bool ledState = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Versuche, die IMU zu initialisieren
  if (!IMU.begin()) {
    Serial.println("IMU konnte nicht initialisiert werden!");
    while (1);  // Stoppe das Programm
  }

  Serial.println("IMU erfolgreich initialisiert.");
  pinMode(ledPin, OUTPUT);
}

void loop() {
  float ax, ay, az;

  // Wenn neue Beschleunigungsdaten verfügbar sind
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    // Betrag der Beschleunigung (inkl. Gravitation)
    float aTotal = sqrt(ax * ax + ay * ay + az * az);

    Serial.print("a_total = ");
    Serial.println(aTotal);

    if (aTotal > motionThreshold) {
      // Bewegung erkannt → LED blinkt (alle 100ms)
      if (millis() - lastBlink > 100) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastBlink = millis();
      }
    } else {
      // Keine Bewegung → LED aus
      digitalWrite(ledPin, LOW);
    }
  }

  delay(20);  // Kurze Pause, damit der Sensor Zeit hat
}
