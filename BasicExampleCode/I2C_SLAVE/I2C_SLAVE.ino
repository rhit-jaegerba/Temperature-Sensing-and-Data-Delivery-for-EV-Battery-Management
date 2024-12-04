#include <Wire.h>

const int I2C_SLAVE_ADDRESS = 0x08;  // Define the I2C address of this slave device

void setup() {
  Serial.begin(9600);                // Initialize Serial communication for printing
  Wire.begin(I2C_SLAVE_ADDRESS);     // Join I2C bus as a slave with the given address
  Wire.onReceive(receiveEvent);      // Register the receive event handler
  Wire.onRequest(requestEvent);      // Register the request event handler (optional, in case the master requests data)
  
  Serial.println("I2C Slave Initialized");
  pinMode(SDA, INPUT);  // Set the pin as an input without pull-up or pull-down resistor
  pinMode(SCL, INPUT);  // Set the pin as an input without pull-up or pull-down resistor
  //Wire.setClock(400000);
}

void loop() {
  // Main loop can remain empty since we handle events using interrupts
  delay(100); // Just a small delay to allow for events to trigger
  //Serial.println("0x24");
  //Wire.write(0x24);
}

// Function to handle data received from the I2C master
void receiveEvent(int numBytes) {
  Serial.println("Packet Received");
  Serial.println(numBytes);
  if (numBytes > 0) {
    while (Wire.available()) {
      byte receivedData = Wire.read();   // Read one byte of incoming data
      Serial.print("Received from master: ");
      Serial.println(receivedData, HEX);  // Print in HEX format for better clarity
    }
    
    // Respond with 0x24 to the master
    Wire.write(0x24);  // Send 0x24 back to the master
    //Serial.println("Responding with: 0x24");
  }
  else {
    //Serial.println("No data received");
    Wire.write(0x24);
  }
  //Serial.println("");
  
}

// Function to handle requests from the I2C master (optional, if the master requests data from the slave)
void requestEvent() {
  // Send the same response (0x24) when the master requests data
  Wire.write(0x24);  // Send 0x24 back to the master
  //Serial.println("Sending 0x24 to master upon request");
}
