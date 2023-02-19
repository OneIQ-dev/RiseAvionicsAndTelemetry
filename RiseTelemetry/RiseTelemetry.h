#ifndef RISETELEMETRY_H
#define RISETELEMETRY_H

#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>

// Define a custom struct called MyData with three members: x, y, and z
struct MyData {
  int x;
  int y;
  float z;
};

class RiseNRF24L01 {
  private:
    RF24 radio;
    byte address[6];


  public:
    RiseNRF24L01(byte ce_pin, byte cs_pin, byte* address) {
      radio = RF24(ce_pin, cs_pin);
      memcpy(this->address, address, sizeof(this->address));
    }

    void begin() {
      radio.begin();
      radio.setPALevel(RF24_PA_LOW);
      radio.openWritingPipe(address);
      radio.openReadingPipe(1, address);
      radio.startListening();
    }

    bool send(void* data, uint8_t size, byte* dest_address) {
      radio.stopListening();
      radio.openWritingPipe(dest_address);
      
      bool result = radio.write(data, size);
      radio.openWritingPipe(address);
      radio.startListening();
      return result;
    }

    bool receive(void* data, uint8_t size) {
      if (radio.available()) {
        radio.read(data, size);
        return true;
      }
      return false;
    }

    bool isAddressMatched(byte* expected_address) {
      uint8_t num_pipes = radio.getDynamicPayloadSize();
      for (uint8_t i = 0; i < num_pipes; i++) {
          uint64_t pipe_address = 0;
          radio.openReadingPipe(i, pipe_address);
          byte pipe_address_bytes[6];
          memcpy(pipe_address_bytes, &pipe_address, sizeof(pipe_address));
          if (memcmp(pipe_address_bytes, expected_address, sizeof(address)) == 0) {
            return true;
          }
        }
        return false;
    }
};

#include <RH_RF95.h>
#include <AESLib.h>
#include <RHGenericDriver.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// LoRa Parameters
#define LORA_SPREADING_FACTOR 12
#define LORA_BANDWIDTH 125000
#define LORA_CODING_RATE 5


// defining the adresses of the devices, these two should be reversed on the sketches of GCC and FC 
#define MY_ADDRESS 1
#define OTHER_ADDRESS 2

// Define a struct to represent the data that we want to send
struct SensorReading {
  uint16_t temperature;
  uint16_t humidity;
  uint16_t lightLevel;
};

class RFM95Communication {
  public:
    RFM95Communication(uint8_t address);
    void setup();
    void send(uint8_t toAddress, const SensorReading& reading);
    bool receive(SensorReading& reading, uint8_t& fromAddress, uint8_t desiredAddress);

  private:
    uint8_t address_;
    RH_RF95 rf95_;
    uint8_t receiveBuffer_[RH_RF95_MAX_MESSAGE_LEN];
    // initialising the key and initialisization vector for AES-256
    byte key[32] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    byte iv[32] = {0x8E, 0x5D, 0xF7, 0x0F, 0x2D, 0x3F, 0x11, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09};
};

// constructor to define private variables
RFM95Communication::RFM95Communication(uint8_t address)
  : address_(address), rf95_(RFM95_CS, RFM95_INT) {}

void RFM95Communication::setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95_.init()) {
    while (1);
  }

  rf95_.setFrequency(915.0);
  rf95_.setTxPower(23, false);
  rf95_.setSpreadingFactor(LORA_SPREADING_FACTOR);
  rf95_.setSignalBandwidth(LORA_BANDWIDTH);
  rf95_.setCodingRate4(LORA_CODING_RATE);
}

void RFM95Communication::send(uint8_t toAddress, const SensorReading& reading) {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

  // Copy the SensorReading struct to the buffer
  memcpy(buf+1, &reading, sizeof(reading));
  buf[0] = toAddress;

  // Encrypt the buffer using AES-256
  AES aes;
  aes.do_aes_encrypt(buf+1, sizeof(reading), buf+1, key, 256, iv);

  // CSMA/CA algorithm
  while (1) {
    // Wait for a random time interval
    unsigned long waitTime = random(0, 50);
    delay(waitTime);

    // Check if the channel is clear
    if (!rf95_.isChannelActive()) {
      // Transmit the packet
      rf95_.send(buf, sizeof(reading)+1);
      rf95_.waitPacketSent();
      break;
    }
  }
}



bool RFM95Communication::receive(SensorReading& reading, uint8_t& fromAddress, uint8_t desiredAddress) {
  while (1) {
    // Wait for a random time interval with maximum 50 ms
    unsigned long waitTime = random(0, 50);
    delay(waitTime);

    if (rf95_.available()) {
      uint8_t payloadSize = sizeof(receiveBuffer_);
      if (rf95_.recv(receiveBuffer_, &payloadSize)) {
        if (payloadSize == sizeof(SensorReading) && rf95_.headerFrom() == desiredAddress) {

          // Decrypt the received data using AES-256
          AES aes;
          aes.do_aes_decrypt(receiveBuffer_+1, sizeof(SensorReading), receiveBuffer_+1, key, 256, iv);

          // Copy the decrypted data to the SensorReading struct
          memcpy(&reading, receiveBuffer_+1, sizeof(SensorReading));
          fromAddress = rf95_.headerFrom();
          return true;
        }
      }
    }
    
    // Check if the channel is clear
    if (!rf95_.isChannelActive()) {
      // If the channel is clear but there is no packet available, return false
      return false;
    }
  }
}




#endif
