#include <Wire.h>

const int i2cAddress = 0x08;    // Set your I2C device address here
const byte dataToSend = 0xAB;   // Set the data byte you want to send here

void setup() {
  Wire.begin();                 // Initialize I2C
  Serial.begin(9600);           // Initialize Serial communication
  Serial.print("Sending data 0x");
  Serial.print(dataToSend, HEX);
  Serial.print(" to address 0x");
  Serial.println(i2cAddress, HEX);
}

void loop() {
  /* Commented out to focus on just requesting A byte from slave device
  // Send the data to the specified I2C address
  Wire.beginTransmission(i2cAddress);  // Start communication with I2C device
  Wire.write(dataToSend);              // Send the byte data
  Wire.endTransmission();              // End communication
  Serial.println("Data sent.");
  */

  
  // Request 1 byte from the I2C device
  Wire.requestFrom(i2cAddress, 1);     // Request 1 byte from the I2C device

  // Check if the device has sent any data back
  if (Wire.available()) {
    byte receivedData = Wire.read();   // Read the byte sent back from the slave
    Serial.print("Received response:");
    Serial.println(receivedData); // Print the received data in hexadecimal format
  } else {
    Serial.println("No response received.");
  }

  delay(50); // Delay between transmissions for testing
}
