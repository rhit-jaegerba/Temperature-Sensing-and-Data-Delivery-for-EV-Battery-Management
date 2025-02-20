// SPI SLAVE (ARDUINO)
// SPI COMMUNICATION BETWEEN TWO ARDUINO
// CIRCUIT DIGEST
// Pramoth.T

#include <SPI.h>

volatile boolean received;
volatile byte Slavereceived, Slavesend;

void setup() {
  Serial.begin(9600);

  pinMode(MISO, OUTPUT);         // Sets MISO as OUTPUT (Have to Send data to Master IN)

  SPCR |= _BV(SPE);              // Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt();         // Interrupt ON is set for SPI communication
}

ISR(SPI_STC_vect) {              // Interrupt routine function
  Slavereceived = SPDR;          // Value received from master is stored in variable Slavereceived
  received = true;               // Sets received as True
  SPDR = 25;a
}

void loop() {
  if (received) {                // Logic to SET LED ON or OFF depending on value received from master
    Serial.println(Slavereceived);
    received = false;  
  }
}
