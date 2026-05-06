#include <Arduino.h>
#include "portaSerial.h"

// Pinos: TX=17, RX=16
// Para testar com 1 ESP32: conecte o pino 17 ao pino 16 com um fio (loopback)
static portaSerial* mySerial = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Porta Serial por Software ===");

    mySerial = new portaSerial(17, 16, 9600);

    Serial.println("Enviando 'A'...");
    mySerial->envia('A');

    delay(20);   // tempo para o byte chegar no loopback

    Serial.println("Enviando \"Oi mundo\"...");
    mySerial->envia("Oi mundo");
}

void loop() {
    if (mySerial->disponivel()) {
        char c = mySerial->le();
        Serial.print("Recebido: '");
        Serial.print(c);
        Serial.println("'");
    }
    delay(10);
}
