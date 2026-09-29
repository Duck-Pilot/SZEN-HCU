/*
  SZEngine HY1 "Lightning" control software
  Written by Ábel Radvánszki, Electronics department lead, 2026
  
  May this software, together with those written by my colleagues, 
  drive our car from the test track to the finish line!

  "Just read the code bro"
*/

/*
  Some resources for the lost soul trying to make sense of the nonsense
  https://enepaq.com/wp-content/uploads/2025/02/Communication-Protocols-%E2%80%93-Battery-Management-System-BMS-Tiny-BMS-Enepaq.pdf
*/

#include <Arduino.h>
#include <ASM330LHHSensor.h>
#include <ACAN2515.h>
#include <ACAN_ESP32.h>
#include <esp_task_wdt.h>

// Communication 
  // I2C for ASM330 gyro
    #define sda 35
    #define scl 36
  // SPI for MCP2515 CAN
    #define cs 10
    #define mosi 11
    #define sclk 12
    #define miso 13
    #define mcp_int 48
  // UART for BMS
    #define bms_tx 14
    #define bms_rx 21
  // CAN for controlling vesc
    #define hy_can_tx 38
    #define hy_can_rx 39

// Inputs
  // Cell temperature
    #define temp_s_1 4
    #define temp_s_2 5
    #define temp_s_3 6
    #define temp_s_4 7
  // Output sense for precharge
    #define hsc_out_s 15
  // Shutdown circuit sense
    #define sdc_s 17
  // Fan power input sense
    #define hsc_12v_s 47
  // Driver input sensors
    #define brake_s_1 8 // unused
    #define brake_s_2 9 // unused
    #define steering_s_1 1
    #define steering_s_2 2

// Outputs
  // BMS turn on signal
    #define bms_ign_out 16
  // Enable relay for contactor
    #define contactor_en_out 18
  // Cooling fan control
    #define fan_pwm_out 40
  // Mux for switching the active temperature sensor
    #define mux_1_out 42
    #define mux_2_out 41

// Watchdog
  #define WDT_TIMEOUT_S 1


//------------- Variables --------------
  // Cell temperatures [°C/10]
    uint16_t cell_temp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool temp_error[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t overtemp_timer[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t prechg_t = 0;
    uint16_t avg_cell_t = 0;
    float resistance[4] = {0, 0, 0, 0};
    bool temperature_ok = 0;
  // counts bad sensors (improbable reading), resets after every measurement cycle 
    uint8_t temp_error_counter = 0;
  // Cell voltages [mV]
    uint16_t cell_voltage[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t voltage_error[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool bms_ok = 0;
  // Driver inputs [%]
    int16_t steering = 0;
    bool clutch = 0;
    bool ALS_active = 0;
  // Voltage sense [V/10]
    uint16_t hsc_output = 0;
    uint16_t hsc_12V = 0;
    uint16_t sdc_voltage = 0;
  // Current sense [A/10]
    float battery_current = 0;
    union {
      uint8_t raw[4];
      float f;
    } hsc_current;
  // Current error timeout counters
    uint8_t overcurrent_timer = 0;
    uint8_t undercurrent_timer = 0;
  // VESC data
    // Current [A/10]
      int16_t vesc_left_current = 0;
      int16_t vesc_right_current = 0;
    // Voltage [V/10]
      uint16_t vesc_left_voltage = 0;
      uint16_t vesc_right_voltage = 0;
    // Speed [ERPM]
      int32_t vesc_left_erpm = 0;
      int32_t vesc_right_erpm = 0;
      int32_t vesc_left_rpm_target = 0;
      int32_t vesc_right_rpm_target = 0;
    // Temperature [°C/2]
      uint8_t vesc_left_temp = 0;
      uint8_t vesc_right_temp = 0;
      uint8_t motor_left_temp = 0;
      uint8_t motor_right_temp = 0;
  // Electrical outputs
      bool bms_ign = 0;
      bool last_bms_ign = 0;
      bool contactor_en = 0;
      bool last_contactor_en = 0;
      uint8_t fan_pwm = 0;
      uint16_t max_duty = 0;
      uint8_t mux_channel = 0;
  // CAN outputs
      ACAN2515 mcp_can(cs, SPI, mcp_int);
      uint16_t vesc_left_current_target = 0;
      uint16_t vesc_right_current_target = 0;

  // Gyro data

  // TinyBMS uart
    //uint8_t output_data[] = {0xAA, 0x03, 0x00, 0x05, 0x00, 0x05, 0x8C, 0x13};
    uint8_t read_cell_voltage[8] = {0xAA, 0x03, 0x00, 0x06, 0x00, 0x0A, 0x3C, 0x17};
    uint8_t read_battery_current[4] = {0xAA, 0x15, 0xBE, 0xDF};
    uint8_t read_bms_state[4] = {0xAA, 0x18, 0x7F, 0x1A};
    uint8_t reset_tinybms[5] = {0xAA, 0x02, 0x05, 0x90, 0x83};
    //uint8_t dummy_data[6] = {0xAA, 0x02, 0x05};
    uint8_t input_data[] = {};
    uint16_t tinyBMS_state = 0x00;
  // TinyBMS timeout counters
    uint8_t bms_v_timeout_counter = 0;
    uint8_t bms_c_timeout_counter = 0;
    uint8_t bms_s_timeout_counter = 0;

  // MaxxECU data
    uint16_t ecu_rpm = 0;
    uint8_t ecu_gear = 0;
    uint8_t ecu_speed = 0;
    uint16_t ecu_apps = 0;
    uint16_t ecu_brake_pressure = 0;

    bool engine_running = 0;
    uint8_t engine_state_counter = 0;
    uint16_t ecu_rpm_timeout_counter = 0;
    uint16_t ecu_gear_timeout_counter = 0;
    uint16_t ecu_apps_timeout_counter = 0;
  // Functional variables
    uint8_t adc_cont_pins[] = {hsc_out_s, sdc_s, hsc_12v_s, brake_s_1, brake_s_2, steering_s_1, steering_s_2};
    static const uint32_t mcp_freq = 10000000;
  // CRC lookup table for TinyBMS UART
    const static uint16_t crcTable[256] = {
      0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
      0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
      0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
      0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
      0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
      0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
      0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
      0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
      0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
      0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
      0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
      0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
      0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
      0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
      0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
      0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
      0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
      0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
      0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
      0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
      0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
      0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
      0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
      0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
      0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
      0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
      0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
      0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
      0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
      0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
      0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
      0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
  
  // Task handles
    TaskHandle_t safety_task;        // core 0
    TaskHandle_t CAN_input_task;     // core 0
    TaskHandle_t analog_input_task;  // core 0
    TaskHandle_t VESC_output_task;   // core 1
    TaskHandle_t dash_output_task;   // core 1
    TaskHandle_t fan_control_task;   // core 1
  
  ASM330LHHSensor Gyro(&Wire, 0x6A);
  
uint16_t CRC16(const uint8_t* data, uint16_t length) {
  // CRC generation for TinyBMS, stolen from the communication protocols doc
  uint8_t tmp;
  uint16_t crcWord = 0xFFFF;
  while (length--) {
    tmp = *data++ ^ crcWord;
    crcWord >>= 8;
    crcWord ^= crcTable[tmp];
  }
  return crcWord;
}


bool reset_BMS() {
  // clear the RX buffer
    while (Serial2.available()) {
      Serial2.read();
    }
  // request bms reset
    Serial2.write(reset_tinybms, 5);
  // wait for response
    while (!Serial2.available()) {
      vTaskDelay(1);
    }
  // check first 3 bytes 0xAA, 0x01, 0x02
    uint8_t uartReceived[3] = {};
    Serial2.readBytes(uartReceived, 3);
    if (uartReceived[0] != 0xAA) {
      return false;}
    if (uartReceived[1] != 0x01) {
      return false;}
    if (uartReceived[2] != 0x02) {
      return false;}
  // generate CRC for the message that was received
    uint8_t uartCRC[2] = {};
    Serial2.readBytes(uartCRC, 2);
    if (CRC16(uartReceived, 3) != (uartCRC[0] | (uartCRC[1]<<8))) {
      return false;}
    delay(10000);
  return true;
}

void initialise() {
  // Setup outputs
    pinMode(bms_ign_out,      OUTPUT);
    pinMode(contactor_en_out, OUTPUT);
    pinMode(mux_1_out,        OUTPUT);
    pinMode(mux_2_out,        OUTPUT);
    digitalWrite(bms_ign_out,      0);
    digitalWrite(contactor_en_out, 0);
    digitalWrite(mux_1_out,        1);
    digitalWrite(mux_2_out,        1);
  // Setup pwm
    ledcSetup(0, 5000, 8);
    ledcAttachPin(fan_pwm_out, 0);
    ledcWrite(0, 0);
  // Setup analog for temperatures
    pinMode(temp_s_1, INPUT);
    pinMode(temp_s_2, INPUT);
    pinMode(temp_s_3, INPUT);
    pinMode(temp_s_4, INPUT);
    analogSetPinAttenuation(temp_s_1, ADC_6db);
    analogSetPinAttenuation(temp_s_2, ADC_6db);
    analogSetPinAttenuation(temp_s_3, ADC_6db);
    analogSetPinAttenuation(temp_s_4, ADC_6db);
  // Setup analog for other inputs
    pinMode(hsc_out_s, INPUT);
    pinMode(sdc_s, INPUT);
    pinMode(hsc_12v_s, INPUT);
    pinMode(steering_s_1, INPUT);
    pinMode(steering_s_2, INPUT);
    analogReadResolution(10);
  // Setup Gyro (I2C)
    Wire.begin(sda, scl);
    Wire.setClock(100000);
    Wire.setTimeOut(100);
    Gyro.begin();
    Gyro.Enable_X();
    Gyro.Enable_G();
  // Serial for debugging
    Serial.begin(115200);
  // Setup MCP2515 CAN (SPI)
    SPI.begin(sclk, miso, mosi);
    ACAN2515Settings mcp_settings(mcp_freq, 500000UL);
    mcp_settings.mRequestedMode = ACAN2515Settings::NormalMode;
    mcp_can.begin(mcp_settings, [] {mcp_can.isr();} );
  // Setup ESP32 CAN
    ACAN_ESP32_Settings esp_can_settings(500000UL);
    esp_can_settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
    esp_can_settings.mRxPin = GPIO_NUM_39;
    esp_can_settings.mTxPin = GPIO_NUM_38;
    ACAN_ESP32::can.begin(esp_can_settings);
  // Setup UART for TinyBMS
    Serial2.begin(115200, SERIAL_8N1, bms_rx, bms_tx, false);
    reset_BMS();
    Serial2.end();
    delay(100);
    Serial2.begin(115200, SERIAL_8N1, bms_rx, bms_tx, false);

}

void switch_mux(uint8_t channel) {
  switch (channel) {
    case 0:
      digitalWrite(mux_1_out, LOW);
      digitalWrite(mux_2_out, LOW);
      break;
    case 1:
      digitalWrite(mux_1_out, HIGH);
      digitalWrite(mux_2_out, LOW);
      break;
    case 2:
      digitalWrite(mux_1_out, HIGH);
      digitalWrite(mux_2_out, HIGH);
      break;
    case 3:
      digitalWrite(mux_1_out, LOW);
      digitalWrite(mux_2_out, HIGH);
      break;
  }
  return;
}

void printVoltages() {
  Serial.print(cell_voltage[0]);
  Serial.print(" ");
  Serial.print(cell_voltage[1]);
  Serial.print(" ");
  Serial.print(cell_voltage[2]);
  Serial.print(" ");
  Serial.print(cell_voltage[3]);
  Serial.print(" ");
  Serial.print(cell_voltage[4]);
  Serial.print(" ");
  Serial.print(cell_voltage[5]);
  Serial.print(" ");
  Serial.print(cell_voltage[6]);
  Serial.print(" ");
  Serial.print(cell_voltage[7]);
  Serial.print(" ");
  Serial.print(cell_voltage[8]);
  Serial.print(" ");
  Serial.println(cell_voltage[9]);
  return;
}

void printTemperatures() {
  Serial.print(cell_temp[0]);
  Serial.print(" ");
  Serial.print(cell_temp[1]);
  Serial.print(" ");
  Serial.print(cell_temp[2]);
  Serial.print(" ");
  Serial.print(cell_temp[3]);
  Serial.print(" ");
  Serial.print(cell_temp[4]);
  Serial.print(" ");
  Serial.print(cell_temp[5]);
  Serial.print(" ");
  Serial.print(cell_temp[6]);
  Serial.print(" ");
  Serial.print(cell_temp[7]);
  Serial.print(" ");
  Serial.print(cell_temp[8]);
  Serial.print(" ");
  Serial.print(cell_temp[9]);
  Serial.print(" ");
  Serial.print(cell_temp[10]);
  Serial.print(" ");
  Serial.print(cell_temp[11]);
  Serial.print(" ");
  Serial.print(cell_temp[12]);
  Serial.print(" ");
  Serial.print(cell_temp[13]);
  Serial.print(" ");
  Serial.print(cell_temp[14]);
  Serial.print(" ");
  Serial.println(cell_temp[15]);
  return;
}

void printCurrentAndStatus() {
  Serial.print(battery_current),
  Serial.print(" ");
  Serial.println(tinyBMS_state >> 8, HEX);
}

bool update_cell_voltage() {
  // clear the RX buffer
    while (Serial2.available()) {
      Serial2.read();
    }
  // request cell voltage data
    Serial2.write(read_cell_voltage, 8);
  // wait for response
    while (!Serial2.available()) {
      vTaskDelay(1);
    }
  // check first 3 bytes 0xAA, 0x03, PL length
    uint8_t uartReceived[23] = {};
    Serial2.readBytes(uartReceived, 23);
    if (uartReceived[0] != 0xAA) {
      return false;}
    if (uartReceived[1] != 0x03) {
      return false;}
    uint8_t PL = uartReceived[2];
  // read the CRC bytes
    uint8_t uartCRC[2] = {};
    Serial2.readBytes(uartCRC, 2);    
  // generate CRC for the message that was received
    if(CRC16(uartReceived, 23) != (uartCRC[0] | (uartCRC[1]<<8))) {
      return false;}
  // process the received data
    cell_voltage[0] = uartReceived[4] | (uartReceived[3]<<8);
    cell_voltage[1] = uartReceived[6] | (uartReceived[5]<<8);
    cell_voltage[2] = uartReceived[8] | (uartReceived[7]<<8);
    cell_voltage[3] = uartReceived[10] | (uartReceived[9]<<8);
    cell_voltage[4] = uartReceived[12] | (uartReceived[11]<<8);
    cell_voltage[5] = uartReceived[14] | (uartReceived[13]<<8);
    cell_voltage[6] = uartReceived[16] | (uartReceived[15]<<8);
    cell_voltage[7] = uartReceived[18] | (uartReceived[17]<<8);
    cell_voltage[8] = uartReceived[20] | (uartReceived[19]<<8);
    cell_voltage[9] = uartReceived[22] | (uartReceived[21]<<8);
  return true;
}

bool update_battery_current() {
  // clear the RX buffer
    while (Serial2.available()) {
      Serial2.read();
    }
  // request battery current data
    Serial2.write(read_battery_current, 4);
  // wait for response
    while (!Serial2.available()) {
      vTaskDelay(1);
    }
  // check first 2 bytes 0xAA, 0x15
    uint8_t uartReceived[6] = {};
    Serial2.readBytes(uartReceived, 6);
    if (uartReceived[0] != 0xAA) {
      return false;}
    if (uartReceived[1] != 0x15) {
      return false;}
  // read the CRC bytes
    uint8_t uartCRC[2] = {};
    Serial2.readBytes(uartCRC, 2);
  // generate CRC for the message that was received
    if(CRC16(uartReceived, 6) != (uartCRC[0] | (uartCRC[1]<<8))) {
      return false;}
  // process the received data
    uint32_t temp = uartReceived[2] | (uartReceived[3]<<8) | (uartReceived[4]<<16) | (uartReceived[5]<<24);
    battery_current = *(float*)&temp;    
  return true;
}

bool update_bms_state() {
  // clear the RX buffer
    while (Serial2.available()) {
      Serial2.read();
    }
  // request bms active state
    Serial2.write(read_bms_state, 4);
  // wait for response
    while (!Serial2.available()) {
      vTaskDelay(1);
    }
  // check first 2 bytes 0xAA, 0x18
    uint8_t uartReceived[4] = {};
    Serial2.readBytes(uartReceived, 4);
    if (uartReceived[0] != 0xAA) {
      return false;}
    if (uartReceived[1] != 0x18) {
      return false;}
  // read the CRC bytes
    uint8_t uartCRC[2] = {};
    Serial2.readBytes(uartCRC, 2);
  // generate CRC for the message that was received
    if(CRC16(uartReceived, 4) != (uartCRC[0] | (uartCRC[1]<<8))) {
      return false;}
  // process the received data
    tinyBMS_state = uartReceived[3] | (uartReceived[2]<<8);
    // 0x91 charging
    // 0x92 fully charged
    // 0x93 discharging
    // 0x96 regeneration
    // 0x97 idle
    // 0x9B fault
  return true;
}



uint16_t resistance_temp(float resistance) {
  // Steinhart-Hart equation for NTC thermistor
  float steinhart;
  steinhart = resistance / 10000;     // (R/Ro)
  steinhart = log(steinhart);          // ln(R/Ro)
  steinhart /= 3984;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15);    // + (1/To)
  steinhart = 1.0 / steinhart;         // Invert
  steinhart -= 273.15;                 // convert to °C
  steinhart = 15.433*steinhart-155.82;    // correction *shrug* + 10x °C
  return uint16_t(steinhart);
}

bool check_cell_temp() {
  // switch the mux channel
    mux_channel++;
    if (mux_channel > 3) {
      mux_channel = 0;}
    switch_mux(mux_channel); 
  // read the 4 resistances
    resistance[0] = float((8052000/(analogReadMilliVolts(temp_s_1)+1))-2610);
    resistance[1] = float((8052000/(analogReadMilliVolts(temp_s_2)+1))-2610);
    resistance[2] = float((8052000/(analogReadMilliVolts(temp_s_3)+1))-2610);
    resistance[3] = float((8052000/(analogReadMilliVolts(temp_s_4)+1))-2610);
    
  // convert the resistances to temperatures and store them in the correct variables
    for (uint8_t i = 0; i < 4; i++) {
      uint16_t temp = resistance_temp(resistance[i]);
      uint8_t cell = (mux_channel*4)+i;
      // check the plausability of the temperature data
      if (temp > 1600 || temp < 100) {temp_error[cell] = 1;}
      else {
        temp_error[cell] = 0;
        // check if the cell is overtemperature
        if (temp > 600) {overtemp_timer[cell] += 1;}
        else {overtemp_timer[cell] = 0;}
        // cell_temp[cell] = temp;
      }
      cell_temp[cell] = temp;
    }
    temp_error_counter = 0;
    for (uint8_t i = 0; i < 16; i++) {
      if(overtemp_timer[i] > 4) {return false;}
      if(temp_error[i]) {temp_error_counter++;}
    }
    if (temp_error_counter > 2) {return false;}
  return true;
}

bool check_bms_data() {
  // cycle runs 16 times every second
  // returns false if communication is lost or parameters are outside the allowed range for over 1s

  // update voltages from TinyBMS
    if (!update_cell_voltage()) {bms_v_timeout_counter++;}
    else {bms_v_timeout_counter = 0;}
    if (bms_v_timeout_counter > 16) {return false;}
  // update current from TinyBMS
    if (!update_battery_current()) {bms_c_timeout_counter++;}
    else {bms_c_timeout_counter = 0;}
    if (bms_c_timeout_counter > 16) {return false;}
  // update state from TinyBMS
    if (!update_bms_state()) {bms_s_timeout_counter++;}
    else {bms_s_timeout_counter = 0;}
    if (bms_s_timeout_counter > 16) {return false;}

  // check if the BMS is in a fault state
    if (tinyBMS_state == 0x9B) {return false;}

  // check if any cell voltage is outside the limits
    for (uint8_t i = 0; i<10; i++) {
      if (cell_voltage[i] > 42000 || cell_voltage[i] < 25000) {voltage_error[i]++;}
      else {voltage_error[i] = 0;}
      if (voltage_error[i] > 16) {return false;}
    }
  // check if current is over 560A
  // timeout for overcurrent is 3s (datasheet)
    if (battery_current > 560) {overcurrent_timer++;}
    else {overcurrent_timer = 0;}
    if (overcurrent_timer > 48) {return false;}

  // check if charge current is over 60A
    if (battery_current < -60) {undercurrent_timer++;}
    else {undercurrent_timer = 0;}
    if (undercurrent_timer > 16) {return false;}
  
  return true;
}

void send_cell_temp_can() {
  // send the first 4 temperatures
    CANMessage display_temp_1;
    display_temp_1.id = 0x600;
    display_temp_1.len = 8;
    display_temp_1.data16[0] = cell_temp[0];
    display_temp_1.data16[1] = cell_temp[1];
    display_temp_1.data16[2] = cell_temp[2];
    display_temp_1.data16[3] = cell_temp[3];
    mcp_can.tryToSend(display_temp_1);
  // send the second 4 temperatures
    CANMessage display_temp_2;
    display_temp_2.id = 0x601;
    display_temp_2.len = 8;
    display_temp_2.data16[0] = cell_temp[4];
    display_temp_2.data16[1] = cell_temp[5];
    display_temp_2.data16[2] = cell_temp[6];
    display_temp_2.data16[3] = cell_temp[7];
    mcp_can.tryToSend(display_temp_2);
  // send the third 4 temperatures
    CANMessage display_temp_3;
    display_temp_3.id = 0x602;
    display_temp_3.len = 8;
    display_temp_3.data16[0] = cell_temp[8];
    display_temp_3.data16[1] = cell_temp[9];
    display_temp_3.data16[2] = cell_temp[10]; 
    display_temp_3.data16[3] = prechg_t;
    mcp_can.tryToSend(display_temp_3);
  // send the fourth 4 temperatures
    CANMessage display_temp_4;
    display_temp_4.id = 0x603;
    display_temp_4.len = 8;
    display_temp_4.data16[0] = cell_temp[12];
    display_temp_4.data16[1] = cell_temp[13];
    display_temp_4.data16[2] = cell_temp[14]; 
    display_temp_4.data16[3] = cell_temp[15];
    mcp_can.tryToSend(display_temp_4);
}

void send_cell_voltage_can() {
  // send the first 4 voltages
    CANMessage display_voltage_1;
    display_voltage_1.id = 0x610;
    display_voltage_1.len = 8;
    display_voltage_1.data16[0] = cell_voltage[0];
    display_voltage_1.data16[1] = cell_voltage[1];
    display_voltage_1.data16[2] = cell_voltage[2];
    display_voltage_1.data16[3] = cell_voltage[3];
    mcp_can.tryToSend(display_voltage_1);
  // send the second 4 voltages
    CANMessage display_voltage_2;
    display_voltage_2.id = 0x611;
    display_voltage_2.len = 8;
    display_voltage_2.data16[0] = cell_voltage[4];
    display_voltage_2.data16[1] = cell_voltage[5];
    display_voltage_2.data16[2] = cell_voltage[6];
    display_voltage_2.data16[3] = cell_voltage[7];
    mcp_can.tryToSend(display_voltage_2);
  // send the last 2 voltages, output voltage and battery current
    CANMessage display_voltage_3;
    display_voltage_3.id = 0x612;
    display_voltage_3.len = 8;
    display_voltage_3.data16[0] = cell_voltage[8];
    display_voltage_3.data16[1] = cell_voltage[9];
    display_voltage_3.data16[2] = hsc_output;
    display_voltage_3.data16[3] = battery_current;
    mcp_can.tryToSend(display_voltage_3);
}

void send_battery_diag_can() {
  // send diag data about the battery
    CANMessage diag_1;
    diag_1.id = 0x620;
    diag_1.len = 8;
    diag_1.data[0] = temp_error[0] | temp_error[1] << 1 | temp_error[2] << 2 | temp_error[3] << 3 | temp_error[4] << 4 | 
                     temp_error[5] << 5 | temp_error[6] << 6 | temp_error[7] << 7;
    diag_1.data[1] = temp_error[8] | temp_error[9] << 1 | temp_error[10] << 2 | temp_error[11] << 3 | temp_error[12] << 4 | 
                     temp_error[13] << 5 | temp_error[14] << 6 | temp_error[15] << 7;
    diag_1.data[2] = temp_error_counter;
    diag_1.data[3] = bms_v_timeout_counter;
    diag_1.data[4] = bms_c_timeout_counter;
    diag_1.data[5] = bms_s_timeout_counter;
    diag_1.data[6] = fan_pwm;
    diag_1.data[7] = temperature_ok | bms_ok << 1 | contactor_en << 2 | bms_ign << 3 | engine_running << 4;
    mcp_can.tryToSend(diag_1);
}

void send_hybrid_diag_can() {
  // send diag data about the hybrid system
    CANMessage diag_2;
    diag_2.id = 0x621;
    diag_2.len = 8;
    diag_2.data[0] = 0;
    mcp_can.tryToSend(diag_2);
}

void send_vesc_can() {
  // send the requested current to left VESC ID 79
    CANMessage vesc_left_current_msg;
    vesc_left_current_msg.id = 0x014F;
    vesc_left_current_msg.len = 8;
    vesc_left_current_msg.ext = true;
    vesc_left_current_msg.data16[0] = vesc_left_current_target;
    ACAN_ESP32::can.tryToSend(vesc_left_current_msg);
  // send the requested current to right VESC ID 108
    CANMessage vesc_right_current_msg;
    vesc_right_current_msg.id = 0x016C;
    vesc_right_current_msg.len = 8;
    vesc_right_current_msg.ext = true;
    vesc_right_current_msg.data16[0] = vesc_right_current_target;
    ACAN_ESP32::can.tryToSend(vesc_right_current_msg);
}

void send_vesc_can_rpm() {
  // only used for testing and breaking in the gears
  // send the requested rpm to left VESC ID 79 (hex 4F)
    CANMessage vesc_left_rpm_msg;
    vesc_left_rpm_msg.id = 0x034F;
    vesc_left_rpm_msg.len = 8;
    vesc_left_rpm_msg.ext = true;
    vesc_left_rpm_msg.data[0] = vesc_left_rpm_target >> 24;
    vesc_left_rpm_msg.data[1] = (vesc_left_rpm_target & 0xFFFFFF) >>16;
    vesc_left_rpm_msg.data[2] = (vesc_left_rpm_target & 0xFFFF) >>8;
    vesc_left_rpm_msg.data[3] = (vesc_left_rpm_target & 0xFF);
    //vesc_left_rpm_msg.data_s32[0] = vesc_left_rpm_target;
    ACAN_ESP32::can.tryToSend(vesc_left_rpm_msg);
  // send the requested rpm to right VESC ID 108 (hex 6C)
    CANMessage vesc_right_rpm_msg;
    vesc_right_rpm_msg.id = 0x036C;
    vesc_right_rpm_msg.len = 8;
    vesc_right_rpm_msg.ext = true;
    vesc_right_rpm_msg.data[0] = vesc_right_rpm_target >> 24;
    vesc_right_rpm_msg.data[1] = (vesc_right_rpm_target & 0xFFFFFF) >>16;
    vesc_right_rpm_msg.data[2] = (vesc_right_rpm_target & 0xFFFF) >>8;
    vesc_right_rpm_msg.data[3] = (vesc_right_rpm_target & 0xFF);
    //vesc_right_rpm_msg.data_s32[0] = vesc_right_rpm_target;
    ACAN_ESP32::can.tryToSend(vesc_right_rpm_msg);
}

void safety_function(void *parameter) {
  TickType_t xLastRanSafety;
  xLastRanSafety = xTaskGetTickCount();
  TickType_t xSafetyFrequency = 62 / portTICK_PERIOD_MS; // around 16 Hz is 62
  esp_task_wdt_add(NULL);
  for (;;) {
  // feed the watchdog
    esp_task_wdt_reset();
  // check the battery parameters
    temperature_ok = check_cell_temp();
    bms_ok = check_bms_data();
    contactor_en = temperature_ok && bms_ok;
  // change enable output state if it changed
    if(contactor_en != last_contactor_en) {
      digitalWrite(contactor_en_out, contactor_en);
      last_contactor_en = contactor_en;}
  // output voltages and temperatures for debug
  
  // check MaxxECU if the engine is running
  // set the engine_running variable high/low after a 5 second delay as a filter
    if(!engine_running && (ecu_rpm > 1500)) {
      engine_state_counter++;
      if (engine_state_counter > 80) {engine_running = true; engine_state_counter = 0;}
    }
    if(engine_running && (ecu_rpm < 1500)) {
      engine_state_counter++;
      if (engine_state_counter > 80) {engine_running = false; engine_state_counter = 0;}
    }
  // check PDU hybrid enable switch (input x)
    // not implemented I guess
    bms_ign = engine_running;
    bms_ign = 1;
  // change bms ignition output state if it changed
    if(bms_ign != last_bms_ign) {
      digitalWrite(bms_ign_out, bms_ign);
      last_bms_ign = bms_ign;}
    // print data for debug
    vTaskDelayUntil(&xLastRanSafety, xSafetyFrequency);
  }
}

void CAN_input_function(void *parameter) {
  TickType_t xLastRanCANInput;
  xLastRanCANInput = xTaskGetTickCount();
  TickType_t xCANInputFrequency = 6 / portTICK_PERIOD_MS; // around 150 Hz
  for (;;) {
    if (mcp_can.available()) {
      CANMessage MCP_receive_msg;
      mcp_can.receive(MCP_receive_msg);
      switch(MCP_receive_msg.id) {
        case 0x520:   // RPM
          ecu_rpm = MCP_receive_msg.data16[0];
          ecu_rpm_timeout_counter = 0;
          break;
        case 0x543:   // Gear, Speed
          ecu_gear = MCP_receive_msg.data[0];
          ecu_speed = MCP_receive_msg.data[1];
          ecu_gear_timeout_counter = 0;
          break;
        case 0x630:   // APPS, Brake pressure
          ecu_apps = MCP_receive_msg.data16[0];
          ecu_brake_pressure = MCP_receive_msg.data16[1];
          ecu_apps_timeout_counter = 0;
          break;
        // PDU inputs?
      }
    }

    if (ACAN_ESP32::can.available()) {
      CANMessage ESP_receive_msg;
      ACAN_ESP32::can.receive(ESP_receive_msg);
      switch(ESP_receive_msg.id) {
        case 0x094F: // Status 1, 79  left
          break;
        case 0x096C: // Status 1, 108 right
          break;
      }
    }
    ecu_rpm_timeout_counter++;
    ecu_gear_timeout_counter++;
    ecu_apps_timeout_counter++;
    if (ecu_rpm_timeout_counter  > 166) {ecu_rpm = 0;}
    if (ecu_gear_timeout_counter > 166) {ecu_gear = 0; ecu_speed = 0;}
    if (ecu_apps_timeout_counter > 166) {ecu_apps = 0; ecu_brake_pressure = 0;}


    vTaskDelayUntil(&xLastRanCANInput, xCANInputFrequency);
  }
}

void analog_input_function(void *parameter) {
  for (;;) {
    // steering 1
    // steering 2
    // output voltage     56k, 3.3k divider
      hsc_output = analogReadMilliVolts(hsc_out_s) * 179.7;
    // sdc       9.1k, 3.3k divider
      sdc_voltage = analogReadMilliVolts(sdc_s) * 37.6;
    // HY_12V    9.1k, 3.3k divider
      //hsc_12V = analogReadMilliVolts(hsc_12v_s) * 37.6;
    vTaskDelay(10);
  }
}

void VESC_output_function(void *parameter) {
  for (;;) {
    vTaskDelay(10);
  }
}

void dash_output_function(void *parameter) {
  for (;;) {
    printVoltages();
    printTemperatures();
    printCurrentAndStatus();
    vTaskDelay(200);
  }
}

void fan_control_function(void *parameter) {
  for (;;) {
    //max_duty = 120.0 / float(hsc_12V) * 255.0;
    //if (max_duty > 255) {max_duty = 255;}
    max_duty = 210;
    // calculate average cell temperature
    avg_cell_t = 0;
    for (uint8_t i = 0; i < 16; i++) {
      avg_cell_t += cell_temp[i];
    }
    avg_cell_t /= 16;
    // calculate duty cycle based on temperature
    fan_pwm = ((float(avg_cell_t) / 371.0) - 0.52 ) * max_duty;
    if (fan_pwm < 100) {fan_pwm = 0;}
    if (fan_pwm > max_duty) {fan_pwm = max_duty;}

    ledcWrite(0, fan_pwm);
    vTaskDelay(1000);
  }
}

void setup() {
  // put your setup code here, to run once:
    initialise();
  // Setup watchdog timer for critical tasks
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
  // setup tasks
    // safety (core 0)
      xTaskCreatePinnedToCore(
        safety_function,            // Task function
        "safety",                   // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &safety_task,               // Task handle
        0                           // Core 0
      );
    // CAN inputs (core 0)
    /*
      xTaskCreatePinnedToCore(
        CAN_input_function,         // Task function
        "CAN input",                // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &CAN_input_task,            // Task handle
        0                           // Core 0
      );
    */
    // analog inputs (core 0) 
    /*
      xTaskCreatePinnedToCore(
        analog_input_function,      // Task function
        "analog input",             // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &analog_input_task,         // Task handle
        0                           // Core 0
      );
    */
    // VESC CAN outputs (core 1)
    /*
      xTaskCreatePinnedToCore(
        VESC_output_function,       // Task function
        "VESC output",              // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &VESC_output_task,          // Task handle
        1                           // Core 1
      );
    */
    // dash/telemetry CAN outputs (core 1)
    
      xTaskCreatePinnedToCore(
        dash_output_function,       // Task function
        "dash output",              // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &dash_output_task,          // Task handle
        1                           // Core 1
      );
    
    // cooling fan control (core 1)
    
      xTaskCreatePinnedToCore(
        fan_control_function,       // Task function
        "fan control",              // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &fan_control_task,          // Task handle
        1                           // Core 1
      );
    
  
}

void loop() {
    vTaskDelete(NULL); // Deletes the loopTask, where we're going we don't need it
}

//void loop() {
  // put your main code here, to run repeatedly:
    //int32_t acceleration[3] = {};
    //int32_t angular_rate[3] = {};
    //Gyro.Get_X_Axes(acceleration);
    //Gyro.Get_G_Axes(angular_rate);
    //Serial.println(CRC16(dummy_data, 3), HEX);
    //uint8_t output_data[4] = {read_cell_voltage[0], read_cell_voltage[1], 0x7E, 0xD9};
    //output_data = CRC16(read_cell_voltage);
    //Serial2.write(output_data, 4);
    //uint32_t most = millis();
    //while (millis() < most +100){
    //  Serial.println(Serial2.read(), HEX);
    //  delay(25);
    //}
    //check_bms_data();
    //check_cell_temp();
    //printVoltages();
    //printTemperatures();

  
    
    //delay(1000);
    
//}


