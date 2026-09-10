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

#include <ASM330LHHSensor.h>
#include <ACAN2515.h>
#include <ACAN_ESP32.h>

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


//------------- Variables --------------
  // Cell temperatures [°C/2]
    uint8_t cell_t_1 = 0;
    uint8_t cell_t_2 = 0;
    uint8_t cell_t_3 = 0;
    uint8_t cell_t_4 = 0;
    uint8_t cell_t_5 = 0;
    uint8_t cell_t_6 = 0;
    uint8_t cell_t_7 = 0;
    uint8_t cell_t_8 = 0;
    uint8_t cell_t_9 = 0;
    uint8_t cell_t_10 = 0;
    uint8_t cell_t_11 = 0;
    uint8_t prechg_t_12 = 0;
    uint8_t cell_t_13 = 0;
    uint8_t cell_t_14 = 0;
    uint8_t cell_t_15 = 0;
    uint8_t cell_t_16 = 0;
    uint8_t avg_cell_t = 0;
  // Overtemperature counters 
  // incremented after every overtemp measurement, turn off the contactor if it's over 4 (>1 second) 
    uint8_t overtemp_counter0 = 0;
    uint8_t overtemp_counter1 = 0;
    uint8_t overtemp_counter2 = 0;
    uint8_t overtemp_counter3 = 0;
  // counts bad sensors (improbable reading), resets after every measurement cycle 
    uint8_t temp_error_counter = 0;
  // Cell voltages [mV]
    uint16_t cell_v_1 = 0;
    uint16_t cell_v_2 = 0;
    uint16_t cell_v_3 = 0;
    uint16_t cell_v_4 = 0;
    uint16_t cell_v_5 = 0;
    uint16_t cell_v_6 = 0;
    uint16_t cell_v_7 = 0;
    uint16_t cell_v_8 = 0;
    uint16_t cell_v_9 = 0;
    uint16_t cell_v_10 = 0;
  // Voltage error timeout counters
    uint8_t overvoltage_timeout_counter = 0;
    uint8_t undervoltage_timeout_counter = 0;
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
    uint8_t overcurrent_timeout_counter = 0;
    uint8_t undercurrent_timeout_counter = 0;
  // VESC data
    // Current [A/10]
      int16_t vesc_left_current = 0;
      int16_t vesc_right_current = 0;
    // Voltage [V/10]
      uint16_t vesc_left_voltage = 0;
      uint16_t vesc_right_voltage = 0;
    // Speed [ERPM]
      uint16_t vesc_left_erpm = 0;
      uint16_t vesc_right_erpm = 0;
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

}

void switch_mux(uint8_t channel) {
  if((channel & 0x01) != 0) {
    digitalWrite(mux_1_out, HIGH);
  } else {digitalWrite(mux_1_out, LOW);}
  if((channel & 0x02) != 0) {
    digitalWrite(mux_2_out, HIGH);
  } else {digitalWrite(mux_2_out, LOW);}
  return;
}

// CRC generation for TinyBMS, stolen from the communication protocols doc
uint16_t CRC16(const uint8_t* data, uint16_t length) {
  uint8_t tmp;
  uint16_t crcWord = 0xFFFF;
  while (length--) {
    tmp = *data++ ^ crcWord;
    crcWord >>= 8;
    crcWord ^= crcTable[tmp];
  }
  return crcWord;
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
    cell_v_1 = uartReceived[4] | (uartReceived[3]<<8);
    cell_v_2 = uartReceived[6] | (uartReceived[5]<<8);
    cell_v_3 = uartReceived[8] | (uartReceived[7]<<8);
    cell_v_4 = uartReceived[10] | (uartReceived[9]<<8);
    cell_v_5 = uartReceived[12] | (uartReceived[11]<<8);
    cell_v_6 = uartReceived[14] | (uartReceived[13]<<8);
    cell_v_7 = uartReceived[16] | (uartReceived[15]<<8);
    cell_v_8 = uartReceived[18] | (uartReceived[17]<<8);
    cell_v_9 = uartReceived[20] | (uartReceived[19]<<8);
    cell_v_10 = uartReceived[22] | (uartReceived[21]<<8);
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

uint8_t resistance_temp(float resistance) {
  // Steinhart-Hart equation for NTC thermistor
  float steinhart;
  steinhart = resistance / 10000;     // (R/Ro)
  steinhart = log(steinhart);          // ln(R/Ro)
  steinhart /= 3984;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15);    // + (1/To)
  steinhart = 1.0 / steinhart;         // Invert
  steinhart -= 273.15;                 // convert to °C
  return uint8_t(steinhart);
}

bool check_cell_temp() {
  // switch the mux channel
    mux_channel++;
    if (mux_channel > 3) {
      mux_channel = 0;}
    switch_mux(mux_channel); 
  // read the 4 resistances
    float resistance_1 = float((8052000/(analogReadMilliVolts(temp_s_1)+1))-2440);
    float resistance_2 = float((8052000/(analogReadMilliVolts(temp_s_2)+1))-2440);
    float resistance_3 = float((8052000/(analogReadMilliVolts(temp_s_3)+1))-2440);
    float resistance_4 = float((8052000/(analogReadMilliVolts(temp_s_4)+1))-2440);
  // convert the resistances to temperatures and store them in the correct variables
    switch(mux_channel) {
      case 0:
        cell_t_1 = resistance_temp(resistance_1);
        cell_t_2 = resistance_temp(resistance_2);
        cell_t_3 = resistance_temp(resistance_3);
        cell_t_4 = resistance_temp(resistance_4);
      // check if any temperature is over 60°C
        if (cell_t_1 > 60 || cell_t_2 > 60 || cell_t_3 > 60 || cell_t_4 > 60) {overtemp_counter0++;}
        else {overtemp_counter0 = 0;}
        if (overtemp_counter0 > 4) {return false;}
      // reset the error counter 
        temp_error_counter = 0;
      // check if any temperature is over 160°C, will be treated as a bad sensor
        if (cell_t_1 > 160) {temp_error_counter++;}
        if (cell_t_2 > 160) {temp_error_counter++;}
        if (cell_t_3 > 160) {temp_error_counter++;}
        if (cell_t_4 > 160) {temp_error_counter++;}
      // check if any temperature is under 10°C, will be treated as a bad sensor
        if (cell_t_1 < 10) {temp_error_counter++;}
        if (cell_t_2 < 10) {temp_error_counter++;}
        if (cell_t_3 < 10) {temp_error_counter++;}
        if (cell_t_4 < 10) {temp_error_counter++;}
      // if there are more than 3 bad sensors disable the battery (12 working sensors are required)
        if (temp_error_counter > 3) {return false;}
        break;
      case 1:
        cell_t_5 = resistance_temp(resistance_1);
        cell_t_6 = resistance_temp(resistance_2);
        cell_t_7 = resistance_temp(resistance_3);
        cell_t_8 = resistance_temp(resistance_4);
      // check if any temperature is over 60°C
        if (cell_t_5 > 60 || cell_t_6 > 60 || cell_t_7 > 60 || cell_t_8 > 60) {overtemp_counter1++;}
        else {overtemp_counter1 = 0;}
        if (overtemp_counter1 > 4) {return false;}
        break;
      case 2:
        cell_t_9 = resistance_temp(resistance_1);
        cell_t_10 = resistance_temp(resistance_2);
        cell_t_11 = resistance_temp(resistance_3);
        prechg_t_12 = resistance_temp(resistance_4);
      // check if any temperature is over 60°C
        if (cell_t_9 > 60 || cell_t_10 > 60 || cell_t_11 > 60 || prechg_t_12 > 60) {overtemp_counter2++;}
        else {overtemp_counter2 = 0;}
        if (overtemp_counter2 > 4) {return false;}
        break;
      case 3:
        cell_t_13 = resistance_temp(resistance_1);
        cell_t_14 = resistance_temp(resistance_2);
        cell_t_15 = resistance_temp(resistance_3);
        cell_t_16 = resistance_temp(resistance_4);
      // check if any temperature is over 60°C
        if (cell_t_13 > 60 || cell_t_14 > 60 || cell_t_15 > 60 || cell_t_16 > 60) {overtemp_counter3++;}
        else {overtemp_counter3 = 0;}
        if (overtemp_counter3 > 4) {return false;}
        break;
    }  
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

  // check if any cell voltage is over 4.2V
    if (cell_v_1 > 42000 || cell_v_2 > 42000 || cell_v_3 > 42000 || cell_v_4 > 42000 || cell_v_5 > 42000 
      || cell_v_6 > 42000 || cell_v_7 > 42000 || cell_v_8 > 42000 || cell_v_9 > 42000 || cell_v_10 > 42000) {overvoltage_timeout_counter++;}
    else {overvoltage_timeout_counter = 0;}
    if (overvoltage_timeout_counter >16) {return false;}

  // check if any cell voltage is under 2.5V
    if (cell_v_1 < 25000 || cell_v_2 < 25000 || cell_v_3 < 25000 || cell_v_4 < 25000 || cell_v_5 < 25000 
      || cell_v_6 < 25000 || cell_v_7 < 25000 || cell_v_8 < 25000 || cell_v_9 < 25000 || cell_v_10 < 25000) {undervoltage_timeout_counter++;}
    else {undervoltage_timeout_counter = 0;}
    if (undervoltage_timeout_counter >16) {return false;}

  // check if current is over 560A
  // timeout for overcurrent is 3s (datasheet)
    if (battery_current > 560) {overcurrent_timeout_counter++;}
    else {overcurrent_timeout_counter = 0;}
    if (overcurrent_timeout_counter > 48) {return false;}

  // check if charge current is over 60A
    if (battery_current < -60) {undercurrent_timeout_counter++;}
    else {undercurrent_timeout_counter = 0;}
    if (undercurrent_timeout_counter > 16) {return false;}
  
  return true;
}

void send_cell_temp_can() {
  // send the first 8 temperatures
    CANMessage display_temp_1;
    display_temp_1.id = 0x600;
    display_temp_1.len = 8;
    display_temp_1.data[0] = cell_t_1;
    display_temp_1.data[1] = cell_t_2;
    display_temp_1.data[2] = cell_t_3;
    display_temp_1.data[3] = cell_t_4;
    display_temp_1.data[4] = cell_t_5;
    display_temp_1.data[5] = cell_t_6;
    display_temp_1.data[6] = cell_t_7;
    display_temp_1.data[7] = cell_t_8;
    mcp_can.tryToSend(display_temp_1);
  // send the second 8 temperatures
    CANMessage display_temp_2;
    display_temp_2.id = 0x601;
    display_temp_2.len = 8;
    display_temp_2.data[0] = cell_t_9;
    display_temp_2.data[1] = cell_t_10;
    display_temp_2.data[2] = cell_t_11;
    display_temp_2.data[3] = prechg_t_12;
    display_temp_2.data[4] = cell_t_13;
    display_temp_2.data[5] = cell_t_14;
    display_temp_2.data[6] = cell_t_15;
    display_temp_2.data[7] = cell_t_16; 
    mcp_can.tryToSend(display_temp_2);
}

void send_cell_voltage_can() {
  // send the first 4 voltages
    CANMessage display_voltage_1;
    display_voltage_1.id = 0x610;
    display_voltage_1.len = 8;
    display_voltage_1.data16[0] = cell_v_1;
    display_voltage_1.data16[1] = cell_v_2;
    display_voltage_1.data16[2] = cell_v_3;
    display_voltage_1.data16[3] = cell_v_4;
    mcp_can.tryToSend(display_voltage_1);
  // send the second 4 voltages
    CANMessage display_voltage_2;
    display_voltage_2.id = 0x611;
    display_voltage_2.len = 8;
    display_voltage_2.data16[0] = cell_v_5;
    display_voltage_2.data16[1] = cell_v_6;
    display_voltage_2.data16[2] = cell_v_7;
    display_voltage_2.data16[3] = cell_v_8;
    mcp_can.tryToSend(display_voltage_2);
  // send the last 2 voltages, output voltage and battery current
    CANMessage display_voltage_3;
    display_voltage_3.id = 0x612;
    display_voltage_3.len = 8;
    display_voltage_3.data16[0] = cell_v_9;
    display_voltage_3.data16[1] = cell_v_10;
    display_voltage_3.data16[2] = hsc_output;
    display_voltage_3.data16[3] = battery_current;
    mcp_can.tryToSend(display_voltage_3);
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

void safety_function(void *parameter) {
  TickType_t xLastRanSafety;
  xLastRanSafety = xTaskGetTickCount();
  TickType_t xSafetyFrequency = 62 / portTICK_PERIOD_MS; // around 16 Hz
  for (;;) {
  // check the battery parameters
    contactor_en = check_cell_temp() && check_bms_data();
  // change enable output state if it changed
    if(contactor_en != last_contactor_en) {
      digitalWrite(contactor_en_out, contactor_en);
      last_contactor_en = contactor_en;}
  // output voltages and temperatures for debug
    Serial.print(cell_v_1);
    Serial.print(" ");
    Serial.print(cell_v_2);
    Serial.print(" ");
    Serial.print(cell_v_3);
    Serial.print(" ");
    Serial.print(cell_v_4);
    Serial.print(" ");
    Serial.print(cell_v_5);
    Serial.print(" ");
    Serial.print(cell_v_6);
    Serial.print(" ");
    Serial.print(cell_v_7);
    Serial.print(" ");
    Serial.print(cell_v_8);
    Serial.print(" ");
    Serial.print(cell_v_9);
    Serial.print(" ");
    Serial.println(cell_v_10);
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
    vTaskDelay(10);
  }
}

void fan_control_function(void *parameter) {
  for (;;) {
    max_duty = 120.0 / float(hsc_12V) * 255.0;
    if (max_duty > 255) {max_duty = 255;}
    avg_cell_t = 0;
    vTaskDelay(10);
  }
}

void setup() {
  // put your setup code here, to run once:
    initialise();
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
      xTaskCreatePinnedToCore(
        CAN_input_function,         // Task function
        "CAN input",                // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &CAN_input_task,            // Task handle
        0                           // Core 0
      );
    // analog inputs (core 0)
      xTaskCreatePinnedToCore(
        analog_input_function,      // Task function
        "analog input",             // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &analog_input_task,         // Task handle
        0                           // Core 0
      );
    // VESC CAN outputs (core 1)
      xTaskCreatePinnedToCore(
        VESC_output_function,       // Task function
        "VESC output",              // Task name
        10000,                      // Stack size (bytes)
        NULL,                       // Parameters
        1,                          // Priority
        &VESC_output_task,          // Task handle
        1                           // Core 1
      );
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
  // put your main code here, to run repeatedly:
    //int32_t acceleration[3] = {};
    //int32_t angular_rate[3] = {};
    //Gyro.Get_X_Axes(acceleration);
    //Gyro.Get_G_Axes(angular_rate);
    /*Serial.print(acceleration[0]);
    Serial.print(" ");
    Serial.print(acceleration[1]);
    Serial.print(" ");
    Serial.println(acceleration[2]);*/
    //Serial.println(CRC16(dummy_data, 3), HEX);
    //uint8_t output_data[4] = {read_cell_voltage[0], read_cell_voltage[1], 0x7E, 0xD9};
    //output_data = CRC16(read_cell_voltage);
    //Serial2.write(output_data, 4);
    //uint32_t most = millis();
    //while (millis() < most +100){
    //  Serial.println(Serial2.read(), HEX);
    //  delay(25);
    //}


  

  // print data for debug
    

    Serial.print(cell_t_1);
    Serial.print(" ");
    Serial.print(cell_t_2);
    Serial.print(" ");
    Serial.print(cell_t_3);
    Serial.print(" ");
    Serial.print(cell_t_4);
    Serial.print(" ");
    Serial.print(cell_t_5);
    Serial.print(" ");
    Serial.print(cell_t_6);
    Serial.print(" ");
    Serial.print(cell_t_7);
    Serial.print(" ");
    Serial.print(cell_t_8);
    Serial.print(" ");
    Serial.print(cell_t_9);
    Serial.print(" ");
    Serial.print(cell_t_10);
    Serial.print(" ");
    Serial.print(cell_t_11);
    Serial.print(" ");
    Serial.print(prechg_t_12);
    Serial.print(" ");
    Serial.print(cell_t_13);
    Serial.print(" ");
    Serial.print(cell_t_14);
    Serial.print(" ");
    Serial.print(cell_t_15);
    Serial.print(" ");
    Serial.println(cell_t_16);

    Serial.print(battery_current);
    Serial.print(" ");
    Serial.println(tinyBMS_state>>8, HEX);
    delayMicroseconds(62500);
    
}
