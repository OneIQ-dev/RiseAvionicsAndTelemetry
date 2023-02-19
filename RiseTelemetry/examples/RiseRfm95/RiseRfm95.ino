
#include <RiseTelemetry.h>

RFM95Communication rfm95Communication(MY_ADDRESS);

void setup() {
  Serial.begin(9600);
  rfm95Communication.setup();
}

void loop() {
  // Create a sensor reading to send
  SensorReading reading;
  reading.temperature = 25;
  reading.humidity = 50;
  reading.lightLevel = 1000;

  // send it
  rfm95Communication.send(OTHER_ADDRESS, reading);
  delay(1000);

  // this struct will be updated to be the received struct
  SensorReading receivedReading;

  // after the receive function this uint_8 will contain the address of the sender of the received packet
  uint8_t fromAddress;
  if (rfm95Communication.receive(receivedReading, fromAddress, OTHER_ADDRESS)) {
    Serial.print("Received sensor reading from address ");
    Serial.print(fromAddress);
    Serial.print(": ");
    Serial.print(receivedReading.temperature);
    Serial.print(" ");
    Serial.print(receivedReading.humidity);
    Serial.print(" ");
    Serial.println(receivedReading.lightLevel);
  }
}
