#include <RiseTelemetry.h>



// Set the address of the NRF24L01 module
byte address[6] = {0x01, 0x02, 0x03, 0x04, 0x05};

// Initialize an instance of the RiseNRF24L01 library on pins 9 and 10 with the specified address
RiseNRF24L01 nrf24l01(9, 10, address);

// Create an instance of the MyData struct to hold the received data
MyData myData;

void setup() {
  // Initialize the serial port for debugging
  Serial.begin(9600);

  // Initialize the NRF24L01 module
  nrf24l01.begin();
}

void loop() {
  // Check if there is any data available on the NRF24L01 module and if the sender's address matches the expected address
  if (nrf24l01.receive(&myData, sizeof(myData)) && nrf24l01.isAddressMatched(address)) {
    // If data is available and the sender's address matches, print the received data to the serial port
    Serial.print("Received data: ");
    Serial.print(myData.x);
    Serial.print(", ");
    Serial.print(myData.y);
    Serial.print(", ");
    Serial.println(myData.z);
  }

  // Generate some random data to send
  myData.x = random(10);
  myData.y = random(10);
  myData.z = random(100) / 10.0;

  // Send the data to the other NRF24L01 module with the specified address
  nrf24l01.send(&myData, sizeof(myData), address);

  // Wait for 1 second before sending more data
  delay(1000);
}

