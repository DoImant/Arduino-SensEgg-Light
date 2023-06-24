//////////////////////////////////////////////////////////////////////////////
/// @file main.cpp
/// @author Kai R. (you@domain.com)
/// @brief Receive the data determined by the SensEgg sensor and output on the serial interface in JSON format.
/// This program can be used as a serial gateway. E.g. to process the data in ioBroker on a Raspberry Pi..
///
/// @date 2022-05-22
/// @version 1.0
///
//////////////////////////////////////////////////////////////////////////////

// Attention - use current RF24 library from TMRh20 !!!
// https://github.com/nRF24/RF24
#include "RF24.h"
#include <ArduinoJson.h>

constexpr uint8_t SENSOR_ID_MIN {201};
constexpr uint8_t SENSOR_ID_MAX {205};
constexpr uint8_t RF24_MAX_PIPES {6};
constexpr uint8_t RF24_CHANNEL {110};

constexpr uint8_t CE_PIN {8};
constexpr uint8_t CSN_PIN {7};

constexpr float DIVISOR {100.0};
constexpr float DIVISOR_BAT {1000.0};
constexpr uint8_t JSON_MEMORY {100};

struct SensorData   // with nRF24 max. 32Byte are allowed as packet
{
  uint16_t Sensor_ID;   // Unique ID of the sender
  int16_t BME_Temp;
  uint16_t BME_Humi;
  uint32_t BME_Druck;
  uint16_t Vcc;
  uint16_t ON_time;
  int16_t NTC_Temp;
  int16_t Option1;
  int16_t Option2;
  int16_t Option3;
};
SensorData payload = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//______________________ ID  T  H  P  V ms T2  O  O  O __

RF24 radio(CE_PIN, CSN_PIN);   // define  nRF24-Radio-Object
// byte addresses[][6] = {"1RF24", "2RF24"};   // Addresses for transmitters and receivers
//                             ME       Others
const byte addresses[][6] = {"0Base", "1SEgg", "2SEgg", "3SEgg", "4SEgg", "5SEgg"};

bool radioInit = false;

// Json Dokument
// Format: // {"SENSOR_ID":200,"data":{"BME_T":"12.3","BME_H":"55.7","BME_P":"1019","SNE_BATT":"2.987","NTC_T":"22.1"}}
StaticJsonDocument<JSON_MEMORY> doc;

void setup() {
  Serial.begin(38400);

  if (radio.begin()) {
    radioInit = true;
    radio.setChannel(RF24_CHANNEL);   // Transmit channel (between 1 - 125), lower channels often occupied by WIFI
    radio.setPayloadSize(sizeof(payload));
    radio.setAutoAck(true);   // Automatic acknowledgement of receipt
    // Define transmit power (RF24_PA_MAX), (RF24_PA_HIGH), (RF24_PA_LOW), (RF24_PA_MIN)
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);   // Define transmission speed (RF24_250kbps) =
                                       // wider range but not compatible with RF24L01 (without '+')

    for (uint8_t i = 0; i < RF24_MAX_PIPES; i++) { radio.openReadingPipe(i, addresses[i]); }
    radio.startListening();   // Start radio reception
  } else {
    JsonObject sensor = doc.to<JsonObject>();
    sensor["ERROR"] = "Empfänger nicht bereit";
    serializeJson(sensor, Serial);
    Serial.print("\n");
  }
}

void loop() {
  uint8_t pipe;
  if (radioInit && radio.available(&pipe))   // Wenn Daten vorhanden sind
  {
    // Serial.println(pipe);
    radio.read(&payload, sizeof(payload));   // Lese alle Daten in das Array ein
    //******************************************
    if (payload.Sensor_ID >= SENSOR_ID_MIN && payload.Sensor_ID < SENSOR_ID_MAX)   // SE_light (T, H, P, V, ms)
    {
      JsonObject sensor = doc.to<JsonObject>();
      sensor["SENSOR_ID"] = payload.Sensor_ID;
      JsonObject data = sensor.createNestedObject("data");
      data["BME_T"] = static_cast<float>(payload.BME_Temp / DIVISOR);
      data["BME_H"] = static_cast<float>(payload.BME_Humi / DIVISOR);
      data["BME_P"] = static_cast<float>(payload.BME_Druck / DIVISOR);
      data["SNE_BATT"] = static_cast<float>(payload.Vcc / DIVISOR_BAT);
      data["NTC_T"] = static_cast<float>(payload.NTC_Temp / DIVISOR);

      serializeJson(sensor, Serial);
      Serial.print("\n");
    }
    //******************************************
    delay(10);
  }
}