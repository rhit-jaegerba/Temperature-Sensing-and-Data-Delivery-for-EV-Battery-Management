// SPI MASTER (ARDUINO)
// SPI COMMUNICATION BETWEEN TWO ARDUINO
// CIRCUIT DIGEST

#include <SPI.h>  // Library for SPI 


void setup() {
  Serial.begin(9600);                 // Starts Serial Communication at Baud Rate 9600
  
  SPI.begin();                        // Begins the SPI communication
  SPI.setClockDivider(SPI_CLOCK_DIV8);// Sets clock for SPI communication at 2MHz (16MHz/8)
  digitalWrite(SS, HIGH);             // Sets SlaveSelect as HIGH (disconnects master from slave)
}

void loop() {
  byte Mastersend, Masterreceive;

  digitalWrite(SS, LOW);              // Starts communication with the slave
  delay(5);
  Mastersend = 129;
  Masterreceive = SPI.transfer(Mastersend); // Sends Mastersend value to slave and receives value from slave
  Serial.println(Masterreceive);

  digitalWrite(SS, HIGH);
  delay(1000);

}
