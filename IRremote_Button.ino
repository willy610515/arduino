#include <Arduino.h>

/* Defines of GPIO_Interrupt.ino */
const byte ledPin = 2;       // Builtin-LED pin
const byte interruptPin = 0; // BOOT/IO0 button pin
volatile byte state = LOW;

/* Defines of SendAndReceive.ino */
// select only NEC and the universal decoder for pulse width or pulse distance protocols
#define DECODE_NEC          // Includes Apple and Onkyo
#define DECODE_DISTANCE     // in case NEC is not received correctly

#include "PinDefinitionsAndMore.h"
 
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), IRsender, CHANGE);
}

void loop() {
  Serial.println("IR receiving...");
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK); // Specify send pin and enable feedback LED at default feedback LED pin

    pinMode(STATUS_PIN, OUTPUT);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols (&Serial);
    Serial.print(F("at pin "));
#if defined(ARDUINO_ARCH_STM32) || defined(ESP8266)
    Serial.println(IR_RECEIVE_PIN_STRING);
#else
    Serial.println(IR_RECEIVE_PIN);
#endif

    Serial.print(F("Ready to send IR signals at pin "));
#  if defined(IR_SEND_PIN_STRING)
    Serial.print(IR_SEND_PIN_STRING);
#  else
    Serial.print(IR_SEND_PIN);
#  endif
    Serial.print(F(" on press of button at pin "));
    Serial.println(SEND_BUTTON_PIN);
}

void IRsender() {
  Serial.println("IR sending...");
   if (aIRDataToSend->receivedIRData.protocol == UNKNOWN /* i.e. raw */) {
        // Assume 38 KHz
        IrSender.sendRaw(aIRDataToSend->rawCode, aIRDataToSend->rawCodeLength, 38);

        Serial.print(F("Sent raw "));
        Serial.print(aIRDataToSend->rawCodeLength);
        Serial.println(F(" marks or spaces"));
    } else {

        /*
         * Use the write function, which does the switch for different protocols
         */
        IrSender.write(&aIRDataToSend->receivedIRData, DEFAULT_NUMBER_OF_REPEATS_TO_SEND);

        Serial.print(F("Sent: "));
        printIRResultShort(&Serial, &aIRDataToSend->receivedIRData);
    }
}
