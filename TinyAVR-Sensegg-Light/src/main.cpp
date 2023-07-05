//////////////////////////////////////////////////////////////////////////////
/// @file main.cpp
///
/// @author of Software base: SpaghettiCode
/// @https://www.arduinoforum.de/arduino-Thread-SensEgg-light-FunkSensor-ATtiny814-nRF24-BME280-NTC
///
/// @author Code changes and adjustments made by Kai R.
///
/// @brief Measure temperature, air pressure and humidity. The determined values are transmitted
/// via radio.
///
/// @date 2023-05-22
/// @version 1.0
///
//////////////////////////////////////////////////////////////////////////////

#include <avr/sleep.h>
#include <TinyBME280.h>   // https://github.com/technoblogy/tiny-bme280
#include "RF24.h"         // https://github.com/nRF24/RF24
// Attention - use current RF24 library from TMRh20 !!!

// #define SENSOR_ID via the build_flags in the platormio.ini file
// Set basic data per sensor here
#if (SENSOR_ID == 201)
//constexpr float NTC_R {10006.0};                      // Measured resistance for the NTC temperature sensor
constexpr float NTC_R {10080.0};                      // Measured resistance for the NTC temperature sensor
constexpr rf24_pa_dbm_e RF24_SENDPWR {RF24_PA_MAX};   // Define transmit power (RF24_PA_MAX(3)),
                                                      // (RF24_PA_HIGH(2)), (RF24_PA_LOW(1)), (RF24_PA_MIN(0))
uint8_t addresses[][6] = {"0Base", "1SEgg"};          // Addresses for sender and receiver;
#elif (SENSOR_ID == 202)
//constexpr float NTC_R {10003.0};
constexpr float NTC_R {10120.0};
constexpr rf24_pa_dbm_e RF24_SENDPWR {RF24_PA_LOW};
uint8_t addresses[][6] = {"0Base", "2SEgg"};
#elif (SENSOR_ID == 203)
constexpr float NTC_R {9989.0};
constexpr rf24_pa_dbm_e RF24_SENDPWR {RF24_PA_MIN};
uint8_t addresses[][6] = {"0Base", "3SEgg"};
#elif (SENSOR_ID == 204)
// constexpr float NTC_R {10015.0};
constexpr float NTC_R {9990.0};
constexpr rf24_pa_dbm_e RF24_SENDPWR {RF24_PA_LOW};
uint8_t addresses[][6] = {"0Base", "4SEgg"};
#elif (SENSOR_ID == 205)
constexpr float NTC_R {10025.0};
constexpr rf24_pa_dbm_e RF24_SENDPWR {RF24_PA_LOW};
uint8_t addresses[][6] = {"0Base", "5SEgg"};
#endif

constexpr uint16_t BAT_THRESHOLD {2400};            // Battery voltage limit ok or not
constexpr uint32_t REF_VOLTAGE {1500UL * 1024UL};   // 1.500V * 1024 in mV
constexpr uint8_t WAKEUP_COUNTER_MAX {15};

constexpr uint32_t BME280_I2C_SPEED {400000};
constexpr uint8_t BME280_I2C_ADDRESS {0x76};

constexpr uint8_t SEND_RETRIES_MAX {5};
constexpr uint8_t NOCC_MAX {5};   // Maximum "no Communication Counter"
constexpr uint8_t RF_CHANNEL {110};

constexpr uint8_t RF_POWER_MAX {3};          // Test Auto_Low_Powering ( 3 = max. / 0 = min.)
constexpr uint8_t RF_ERROR_BLINK_NUM {10};   // LED flashing number if transmitter was not detected/initialized
constexpr uint8_t RF_RETRY_DELAY {5};        // How long to wait between each retry, in multiples of 250us
constexpr uint8_t RF_RETRY_COUNT {4};        // How many retries before giving up, max 15

constexpr uint16_t NTC_ERROR_VAL {9990};
constexpr uint32_t NTC_MIN_VALID_VAL {1000};     // If valid measured value > -40°C
constexpr uint32_t NTC_MAX_VALID_VAL {200000};   // If valid measured value < 99°C
constexpr float KELVIN_ZERO_DEGREES_CELSIUS {273.15};

constexpr uint8_t LED_GN {PIN_PB2};
constexpr uint8_t LED_RT {PIN_PB3};
constexpr uint8_t CSN_PIN {PIN_PA4};
constexpr uint8_t CE_PIN {PIN_PA5};
constexpr uint8_t NTC_Pin {PIN_PA6};   // for NTC temperature measurement
constexpr uint8_t NTC_PWR {PIN_PA7};   // Power pin for R - NTC - voltage divider
constexpr uint8_t UPDI_PIN {11};

constexpr float R {NTC_R};               // R1-NTC resistance = 9.959k (measured ntc temperature) + Ri_Pin (30-40 Ohm)
constexpr float c1 {0.8998597028E-03};   // NTC-Parameter for:
constexpr float c2 {2.4938774720E-04};   // TS-NTC-103 from B+B Sensors,
constexpr float c3 {2.0107102340E-07};   //

uint8_t sentCounter {0};   // Counts confirmed telegrams for Auto_Low_Powering
uint8_t noSentCounter;
uint8_t wakeupCounter;     // 15x = approx.60s transmission cycle
uint8_t NoCC {0};          // No Communication Counter
uint32_t startZeit;        // for measurement sketch runtime
uint8_t messwertID {0};    // Substitute variable for on-Time (22.0ms --> 16.5ms)
uint16_t ntcTemperatur;    // Substitute variable for NTC temperature
float rawADC {0};          // for NTC temperature measurement
float rT, logRt, kelvin;   // for NTC temperature measurement

RF24 radio(CE_PIN, CSN_PIN);   // define nRF24 radio object

struct SensorData   // with nRF24 max. 32Byte are allowed as packet, 20 Bytes used (32Bit boundaries possible)
{
  uint16_t Sensor_ID;   // Unique ID of the sender
  int16_t BME_Temp;
  uint32_t BME_Druck;
  uint16_t BME_Humi;
  uint16_t Vcc;
  uint16_t ON_time;   // Switch-on time, will be sent with the next transmission
  int16_t NTC_Temp;
  int16_t Option1;
  int16_t Option2;
};
SensorData payload = {SENSOR_ID, 22, 999, 50, 3, 0, 0, 0, 0};
//____________________    ID     T    P   H   V ms T2  O  O

void ADCSetup_Vcc() {
  VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;                     // 1.5V Reference
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV4_gc;   // 1.250MHz clock @ 5MHz
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;                      // Measure INTREF
  ADC0.CTRLA = ADC_ENABLE_bm;                              // Single, 10-bit
}

int Vcc_messen() {
  uint16_t adc_reading = 0;
  for (uint8_t i = 0; i < 2; i++)   // 1. = dummy read!
  {
    delayMicroseconds(25);
    ADC0.COMMAND = ADC_STCONV_bm;              // Start conversion
    while (ADC0.COMMAND & ADC_STCONV_bm) {};   // Wait for completion
    adc_reading = ADC0.RES;                    // ADC conversion result
  }
  return REF_VOLTAGE / adc_reading;   // 1.500V * 1024 in mV / adc_reading
}

void RTC_init() {
  while (RTC.STATUS > 0) {
    ;   // Wait for all register to be synchronized
  }
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;      // OSCULP32K --> DIV32 --> 1.024kHz
  RTC.PITINTCTRL = RTC_PI_bm;            // PIT Interrupt: enabled
  RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc   // RTC Clock Cycles 4.096 --> ca. 4.096ms
                 | RTC_PITEN_bm;         // Enable PIT counter: enabled
}

void NTC_messen() {
  digitalWriteFast(NTC_PWR, HIGH);   // Activate voltage divider NTC
  rawADC = analogRead(PIN_PA6);      //
  digitalWriteFast(NTC_PWR, LOW);    // Switch off voltage divider NTC
  rT = R * (rawADC / (1024 - rawADC));
}

void Temp_NTC_rechnen() {
  if (rT > NTC_MIN_VALID_VAL && rT < NTC_MAX_VALID_VAL)   // If valid measured value (< 99°C oder > -40°C)
  {
    // Steinhart-Hart equation for temperature curve NTC / ~ 0.8ms @ 5MHz
    logRt = log(rT);
    kelvin = (1.0 / (c1 + c2 * logRt + c3 * logRt * logRt * logRt));
    payload.NTC_Temp =
        static_cast<int>((kelvin - KELVIN_ZERO_DEGREES_CELSIUS) * 100.0);   // kelvin >> *Celsius, 2 decimal places
  } else payload.NTC_Temp = NTC_ERROR_VAL;                                  // 99.9*C as error message for NTC
}

void blinken_RT() {
  digitalWriteFast(LED_RT, HIGH);
  delayMicroseconds(2400);
  digitalWriteFast(LED_RT, LOW);
}

void blinken_GN() {
  digitalWriteFast(LED_GN, HIGH);
  delayMicroseconds(800);
  digitalWriteFast(LED_GN, LOW);
}

ISR(RTC_PIT_vect) {
  // Clear interrupt flag by writing '1' (required)
  RTC.PITINTFLAGS = RTC_PI_bm;
}

void setup() {
  pinMode(NTC_PWR, OUTPUT);
  pinMode(LED_RT, OUTPUT);
  pinMode(LED_GN, OUTPUT);
  pinMode(UPDI_PIN, INPUT_PULLUP);   // unused Pin UPDI

  RTC_init();                                // Initialize the RTC timer
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);       // Set sleep mode to POWER DOWN mode
  sleep_enable();                            // Enable sleep mode, but not yet
  Wire.begin();                              // Wire.h is in the megaTinyCore!
  Wire.usePullups();                         // In addition to the 10k from the BME280 ???
  Wire.setClock(BME280_I2C_SPEED);           // 22ms --> 16.5ms ;-)
  BME280setI2Caddress(BME280_I2C_ADDRESS);   // the BME280 has 0x76 or 0x77

  if (radio.begin())   // Starte Sender
  {
    radio.setChannel(RF_CHANNEL);   // Transmit channel (between 1 - 125), lower channels often occupied by WIFI
    radio.setPayloadSize(sizeof(payload));   // Send only as much data as necessary
    radio.setAutoAck(true);                  // Automatic confirmation on receipt data
    radio.setPALevel(RF24_SENDPWR);          // Define transmit power
    radio.setDataRate(RF24_250KBPS);         // Define transmission speed RF24_1MBPS, (RF24_250KBPS) = wider range
                                             //  but not compatible with RF24L01 (without '+')
    radio.setRetries(RF_RETRY_DELAY,
                     RF_RETRY_COUNT);   // Attempts in the interval of RF_RETRY_DELAY * 250µs the data RF_RETRY_COUNT4x
                                        // to send repeatedly if no confirmation arrived from the recipient

    // Open the pipes to send and receive these are defined by the address in the array (6 byte)
    radio.openWritingPipe(addresses[1]);      // here is sent
    radio.openReadingPipe(1, addresses[0]);   // here is received
    radio.stopListening();                    // Set transmitter to transmit mode

    radio.powerDown();   // Disable radio after sending
  } else {               // If transmitter was not detected/initialized, 10x LED flashing
    for (uint8_t i = 0; i < RF_ERROR_BLINK_NUM; i++) {
      blinken_RT();
      delay(198);
    }
  }
  blinken_GN();
}

void loop() {
  sleep_cpu();   // Sleep, wait for an interrupt
  wakeupCounter++;
  // if (wakeupCounter == 2 || wakeupCounter == 6 || wakeupCounter == 10 || wakeupCounter == 14) {
  //   if (wakeupCounter == 2) { rawADC = 0; }
  //   NTC_messen();
  // }
  if (wakeupCounter >= WAKEUP_COUNTER_MAX) {   // 15 x 4.096ms --> ca. 61s
    wakeupCounter = 0;
    noSentCounter++;                                              // counts voluntary non-sending (max. 4x)
    if (NoCC < NOCC_MAX || noSentCounter >= SEND_RETRIES_MAX) {   // if 5x was not received,
                                                                  // then send only every 5 minutes!
      startZeit = micros();
      noSentCounter = 0;
      ADC0.CTRLA |= ADC_ENABLE_bm;   // ADC Switch on
      NTC_messen();
      Temp_NTC_rechnen();
      ADCSetup_Vcc();
      payload.Vcc = Vcc_messen();
      analogReference(VDD);
      ADC0.CTRLA &= ~ADC_ENABLE_bm;             // Switch off ADC
      BME280setup();                            // BME280 wake
      payload.BME_Temp = BME280temperature();   // BME supplies four-digit Temperature (2 decimal points)
      payload.BME_Humi = BME280humidity();
      payload.BME_Druck = BME280pressure();   // Air pressure conversion to NN on the target system
      BME280sleep();
      radio.powerUp();   // Start Radio (nRF24)
      if (!radio.write(&payload, sizeof(payload))) {
        NoCC++;                                // No Communication Counter
        NoCC = constrain(NoCC, 0, NOCC_MAX);   // NoCC % NOCC_MAX
        blinken_RT();
      } else {
        NoCC = 0;   // wenn gesendet wurde, Zähler auf 0
        // if (payload.Vcc >= BAT_THRESHOLD) blinken_GN();   // sent, Ubatt i.O.
        // if (payload.Vcc < BAT_THRESHOLD) blinken_RT();    // Replace battery soon!
        (payload.Vcc >= BAT_THRESHOLD) ? blinken_GN() : blinken_RT();
      }
      radio.powerDown();                                                   // Disable radio after sending
      payload.ON_time = static_cast<int>((micros() - startZeit) * 0.01);   // in ms, one decimal place
    }
  }
}
