/*!
 * @file motionWake.ino
 * @brief Set the accelerometer interrupt wake-up threshold. In the low-power mode, if the accelerometer of any x, y, z axis reaches the threshold,
 * the interrupt output pin INT of the sensor will generate an interrupt signal. Only accelerometer can work normally in low power consumption mode.
 *
 * @n connection table in SPI
 * ---------------------------------------------------------------------------------------------------------------------
 *  Sensor pin  |                      MCU                          | ESP32 | ESP8266 |    M0   | micro:bit | Mega2560 |
 *    FSY       | not connected, floating                           |   X   |    X    |    X    |     X     |     X    |
 *    INT       | connected to the external interrupt IO pin of MCU | 2/D9  |  2/D5   |    2    |     P9    |     2    |
 *    CS        | connected to the IO pin of MCU                    | 5/D8  |  5/D6   |    5    |     P8    |     5    |
 *    SDO       | connected to miso of mcu'spi                      |19/MISO|  MISO   |    MI   |  P14/MISO |  50/MISO |
 *    SDI       | connected to mosi of mcu'spi                      |23/MOSI|  MOSI   |    MO   |  P15/MOSI |  51/MOSI |
 *    SCK       | connected to sck of mcu'spi                       |18/SCK |   SCK   |   SCK   |  P13/SCK  |  52/SCK  |
 *    GND       | GND                                               |  GND  |   GND   |   GND   |    GND    |    GND   |
 *    3V3/VCC   | 3V3/VCC                                           |  3V3  |   3V3   |   3V3   |    3V3    |    5V    |
 * ---------------------------------------------------------------------------------------------------------------------
 *
 * @n connection table in IIC
 * -------------------------------------------------------------------------------------------------------------------
 * sensor pin |                         MCU                       | ESP32 | ESP8266 |    M0   | micro:bit | Mega2560 |
 *    FSY     | not connected, floating                           |   X   |    X    |    X    |     X     |     X    |
 *    INT     | connected to the external interrupt IO pin of MCU | 2/D9  |  2/D5   |    2    |     P9    |     2    |
 *    SDA     | connected to SDA of mcu'iic                       | 21/SDA|   SDA   |   SDA   |  P20/SDA  |  20/SDA  |
 *    SCL     | connected to scl of mcu'iic                       | 22/SCL|   SCL   |   SCL   |  P19/SCL  |  21/SCL  |
 *    GND     | GND                                               |  GND  |   GND   |   GND   |    GND    |    GND   |
 *    3V3/VCC | 3V3/VCC                                           |  3V3  |   3V3   |   3V3   |    3V3    |    5V    |
 * -------------------------------------------------------------------------------------------------------------------
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @data  2021-06-01
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_ICG20660L
 */
#include "DFRobot_ICG20660L.h"


#if defined(ARDUINO_BBC_MICROBIT)
#define CS_PIN      8                      //The CS pin of sensor which is connected to the 8 digital io pin of micro:bit can also be connected to other pin.
#define INT_PIN     9                      //The INT pin of sensor which is connected to the 8 digital io pin of micro:bit can also be connected to other pin.
#else
#define CS_PIN      5                      //The CS pin of sensor which is connected to the 5 digital io pin of MCU can also be connected to other pin.
#define INT_PIN     2                      //The INT pin of sensor which is connected to the 2 digital io pin of MCU can also be connected to other pin.
#endif
/**
 * @brief The constructor of the ICG20660L sensor, using IIC communication.
 * @param addr: 7-bit IIC address, controlled by SDO pin.
 * @n     IIC_ADDR_SDO_H or 0x69:  SDO pull high.(default)
 * @n     IIC_ADDR_SDO_L or 0x68:  SDO pull down.
 * @param pWire:   TwoWire class pointer.
 */
DFRobot_ICG20660L_IIC icg(/*addr=*/IIC_ADDR_SDO_H, &Wire);
/**
 * @brief The constructor of the ICG20660L sensor, using SPI communication.
 * @param csPin: SPI chip select pin, connected to IO pin of MCU.
 * @param spi: SPIClass class pointer.
 */
//DFRobot_ICG20660L_SPI icg(/*csPin=*/CS_PIN, &SPI);
bool irqFlag = false;

void fun(){
    irqFlag = true;
}
void setup() {
  Serial.begin(115200);
  while(!Serial){                                                     //Waiting for USB Serial COM port to open.
  }
  
  Serial.print("Initialization sensor...");
/**
 * @brief Initialize the sensor. After initialization, all sensors are turned off, and the corresponding configuration needs to be turned on through enableSensor. 
 * @param mode: Enum variable,from eDataReadMode_t, configure to read sensor data from FIFO or register?
 * @n     eRegMode: Read sensor data from registers.
 * @n     eFIFOMode: Read data from 512-byte FIFO. Note: Read from FIFO, accelerometer, gyroscope and temperature must all be enabled, 
* @n and the internal sampling rate must be configured to be consistent. (This demo doesn’t support)
 * @return status:
 * @n      0 :   Initialization success.
 * @n      -1:   Interface initialization failed(IIC or SPI).
 * @n      -2:   Failed to read the device ID, the ID is not 0x91
 */
  while(icg.begin(/*mode=*/icg.eRegMode) != 0){
      Serial.println("failed. Please check the hardware connection.");
      delay(1000);
      Serial.print("Initialization sensor...");
  }
  Serial.println("done.");
  
  Serial.print("ICG20660L Device ID: 0x");
  Serial.println(icg.readID(), HEX);
  
/**
 * @brief Enable sensor, including Accel of xyz axis, Gyro of xyz, temperature. 
 * @param bit: 8-bit byte data. Each bit represents enabling a function bit, as shown in the following table:
 * @n -------------------------------------------------------------------------------------------------------------------
 * @n |       bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
 * @n -------------------------------------------------------------------------------------------------------------------
 * @n |     reserve     |    reserve   | eAccelAxisX | eAccelAxisY | eAccelAxisZ | eGyroAxisX | eGyroAxisY | eGyroAxisZ |
 * @n |                                |            eAccelAxisXYZ                |           eGyroAxisXYZ               |
 * @n |                                |                                eAxisAll                                        |
 * @n -------------------------------------------------------------------------------------------------------------------
 * @n   bit0:  Z-axis of gyro and temperature.
 * @n   bit1:  Y-axis of gyro and temperature.
 * @n   bit2:  X-axis of gyro and temperature.
 * @n   bit3:  Z-axis of acceleration.
 * @n   bit4:  Z-axis of acceleration.
 * @n   bit5:  Z-axis of acceleration.
 * @n   bit6:  reserve.
 * @n   bit7:  reserve.
 * @n Note: Enabling any axis of the gyroscope will automatically enable the on-board temperature sensor.
 * @n   eGyroAxisZ: The bit0 of the bit, enable gyro's z axis and temperature.
 * @n   eGyroAxisY: The bit1 of the bit, enable gyro's y axis and temperature.
 * @n   eGyroAxisX: The bit2 of the bit, enable gyro's X axis and temperature.
 * @n   eAccelAxisZ: The bit3 of the bit, enable accel's z axis.
 * @n   eAccelAxisY: The bit4 of the bit, enable Accel's y axis.
 * @n   eAccelAxisX: The bit5 of the bit, enable Accel's X axis.
 * @n   eGyroAxisXYZ or eGyroAxisX|eGyroAxisY|eGyroAxisZ: The bit0/bit1/bit2 of the bit, enable gyro's xyz axis and temperature.
 * @n   eAccelAxisXYZ or eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit3/bit4/bit5 of the bit, enable Accel's xyz axis.
 * @n   eAxisAll or eGyroAxisX|eGyroAxisY|eGyroAxisZ|eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit, enable temperature, Accel's and gyro's xyz axis. 
 */
  icg.enableSensor(icg.eAccelAxisXYZ);
  //icg.enableSensor(icg.eAccelAxisX|icg.eAccelAxisY|icg.eAccelAxisZ);
/**
 * @brief Config of accel's full scale, dlpf bandwidth and internal sample rate. 
 * @param scale  The full scale of accel, unit: g(1g = 9.80665 m/s²).
 * @n     eFSR_A_2G:  The full scale range is ±2g.
 * @n     eFSR_A_4G:  The full scale range is ±4g.
 * @n     eFSR_A_8G:  The full scale range is ±8g.
 * @n     eFSR_A_16G:  The full scale range is ±16g.
 * @param bd  Set 3-db bandwidth.
 * @n     eAccel_DLPF_5_1KHZ or 0: When the signal is less than or equal to 5Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
 * @n     eAccel_DLPF_10_1KHZ or 1: When the signal is less than or equal to 10Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
 * @n     eAccel_DLPF_21_1KHZ or 2: When the signal is less than or equal to 21Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
 * @n     eAccel_DLPF_44_1KHZ or 3: When the signal is less than or equal to 44Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
 * @n     eAccel_DLPF_99_1KHZ or 4: When the signal is less than or equal to 99Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
 * @n     eAccel_DLPF_218_1KHZ or 5: When the signal is less than or equal to 218Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Support low power consumption mode
 * @n     eAccel_DLPF_420_1KHZ or 6: When the signal is less than or equal to 420Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Support low power consumption mode
 * @n     eAccel_DLPF_1046_4KHZ or 7: When the signal is less than or equal to 1046Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 4KHz. Support low power consumption mode
 * @n     eAccel_DLPF_55_1KHZ or 8: When the signal is less than or equal to 55Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Only support low power consumption mode.
 * @n     eAccel_DLPF_110_1KHZ or 9: When the signal is less than or equal to 110Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz. Only support low power consumption mode
 * @n Note: When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO, 
 * @n the internal sampling rate of the gyroscope and accelerometer must be the same.
 * @param odr: Set the frequency of waking up the chip to take a sample of accel data – the low power accel Output Data Rate.
 * @n     eODR_125Hz or 9:    The low power accel Output Data Rate: 125Hz
 * @n     eODR_250Hz or 10:   The low power accel Output Data Rate: 250Hz
 * @n     eODR_500Hz or 11:   The low power accel Output Data Rate: 500Hz
 * @param lowPowerFlag:  Whether to configure the Acceleration to low power mode.
 * @n     true:          Enter low power mode.
 * @n     false:         Not configure the Acceleration to low power mode.(default)
 */
  icg.configAccel(icg.eFSR_A_16G, icg.eAccel_DLPF_1046_4KHZ, icg.eODR_500Hz, true);
/**
 * @brief Set sample rate divider. 
 * @param div  Sample rate divider, the range is 0~255.
 * @n     Sampling rate = internal sampling rate/(div+1)
 * @n Note: If the accelerometer configuration is in low power consumption mode, that is, the formal parameter lowPowerFlag of the configAccel function is true, 
 * @n the sampling rate must match the output rate of the formal parameter odr of configAccel, as shown in the following table:
 * @n ----------------------------------------------------------------------------
 * @n |                           configAccel                    |  setSampleDiv  |
 * @n ----------------------------------------------------------------------------|
 * @n |            bd             |      odr      | lowPowerFlag |      div       |
 * @n ----------------------------------------------------------------------------|
 * @n |            X              |       X       |    false     |      0~255     |
 * @n ----------------------------------------------------------------------------|
 * @n |                           |  eODR_125Hz   |    true      |        7       |
 * @n |                           |-----------------------------------------------|
 * @n |bd of supporting low power consumption mode|  eODR_250Hz   |    true      |        3       |
 * @n |                           |-----------------------------------------------|
 * @n |                           |  eODR_500Hz   |    true      |        1       |
 * @n |---------------------------------------------------------------------------|
 */
  icg.setSampleDiv(1);
/**
 * @brief Set the level polarity of the INT pin when the accelerometer sensor is triggered to wake up the motion interrupt.
 * @param polarity: the level signal of the sensor INT pin when the wake-up motion is triggered
 * @n     HIGH:  The initial signal of the INT pin is LOW. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to HIGH. 
 * @n Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
 * @n     LOW:   The initial signal of the INT pin is HIGH. When an accelerometer wake-up motion occurs, the level signal of the INT pin will change to LOW. 
 * @n Then the readINTStatus function needs to be called to clear the signal and restore the initial signal.
 * @n Note: After triggering the accelerometer wake-up motion, if the readINTStatus function is not called to clear the sign, 
 * @n the INT pin will always maintain the level polarity when the motion is triggered.
 */
  icg.setINTPinMotionTriggerPolarity(/*polarity =*/LOW);
/**
 * @brief Set the threshold value for the Wake on Motion Interrupt for accelerometer. 
 * @param level: WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg
 * @n     level = 0~255
 * @return Actul WoM thresholds, unit : g   re_value = (level * 3.9)/1000 g
 */
  icg.setWakeOnMotionThresholdForAccel(100);/*rate = 100*3.9/1000 g = 0.39g*/

  pinMode(INT_PIN,INPUT);
/**
 * @brief Enable the external interrupt pin of MCU. 
 * @param pin:   The external pin of MCU.
 * @n     Mega2560:  The external pin is 2, 3, 21, 20, 19, 18.
 * @n     microbit:  The external pin is 0~20(P0-P20)
 * @n     ESP32, ESP8266, M0:    The external pin is all digital pin and analog pin.
 * @param fun: Pointer to guide interrupt service function.
 * @param mode:  Interrupt trigger mode.
 * @n     LOW:     Low level trigger.
 * @n     HIGH:    HIGH level trigger
 * @n     RISING:  Rising edge trigger
 * @n     FALLING: Falling edge trigger
 * @n     CHANGE:  Double edge transition trigger
 */
  attachInterrupt(/*pin=*/digitalPinToInterrupt(INT_PIN),/*fun=*/fun,/*mode =*/FALLING);
}

void loop() {
  uint8_t status;
  if(irqFlag || (digitalRead(INT_PIN) == icg.getINTPinMotionTriggerPolarity())){
      irqFlag = false;
      /**
         * @brief Read interrupt status register, and clear INT pin's interrupt signal. 
         * @return Interrupt status register value.
         * @n  INT_STATUS register：addr:0x3A,acess:rw
         * @n  ------------------------------------------------------------------------------------
         * @n  |     b7    |    b6     |    b5     |      b4        | b3 | b2 | b1 |      b0      |
         * @n  ------------------------------------------------------------------------------------
         * @n  |             WOM_XYZ_INT           | FIFO_OFLOW_INT |     rsv      | DATA_RDY_INT |
         * @n  ------------------------------------------------------------------------------------
         * @n  DATA_RDY_INT  : This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read.
         * @n  rsv           : reserve
         * @n  FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow has been generated. The bit clears to 0 after the register has been read.
         * @n  WOM_XYZ_INT   : These bits automatically set to a non-zero number when the X-axis, Y-axis or Z-axis of accelerometer which trigger WOM(wake on motion) 
         * @n                  interrupt. Cleared on Read.
       */
      status = icg.readINTStatus();
      if(status & ICG20660L_WOM_XYZ_INT){
          Serial.println("Motion wake-up detected!");
      }else{
          Serial.println("Error!");
      }
  }
}


