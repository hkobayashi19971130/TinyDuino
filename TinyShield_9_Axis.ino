#include <STBLE.h>

//-------------------------------------------------------------------------------
//  TinyCircuits MPU-9150 9 Axis TinyShield Example Sketch
//  Last Updated 19 July 2016
//  
//  This demo is intended for the ASD2612 9 Axis TinyShield with a MPU-9150
//  9 axis sensor populated. It shows basic use of a modified RTIMULib with the
//  sensor.
//
//  Modified by Ben Rose for TinyCircuits, https://tinycircuits.com
//
//-------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
////////////////////////////////////////////////////////////////////////////


#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include <EEPROM.h>
//---- trial Addition for BLE COM --START--
#include <SPI.h>
#include <STBLE.h>

#ifndef BLE_DEBUG
#define BLE_DEBUG true
#endif
//---- trial Addition for BLE COM --END--

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  50                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

#ifdef SERIAL_PORT_MONITOR
  #define SerialMonitor SERIAL_PORT_MONITOR
#else
  #define SerialMonitor Serial
#endif

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

//---- trial Addition for BLE COM --START--
uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;
#define PIPE_UART_OVER_BTLE_UART_TX_TX 0
//---- trial Addition for BLE COM --END--


void setup()
{
    int errcode;

    SerialMonitor.begin(SERIAL_PORT_SPEED);
    //while (!SerialMonitor);//Optional- On TinyScreen+/SAMD21 boards this will block until the serial monitor is opened
    Wire.begin();
    SerialMonitor.println("HI");
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    SerialMonitor.print("ArduinoIMU starting using device "); SerialMonitor.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        SerialMonitor.print("Failed to init IMU: "); SerialMonitor.println(errcode);
    }

    if (imu->getCalibrationValid())
        SerialMonitor.println("Using compass calibration");
    else
        SerialMonitor.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    fusion.setGyroEnable(false);
    fusion.setAccelEnable(false);
    fusion.setCompassEnable(false);
    
    //---- trial Addition for BLE COM --START--
    BLEsetup();
    //---- trial Addition for BLE COM --END--
}

void loop()
{  
  unsigned long now = millis();
  unsigned long delta;
  
  //---- trial Addition for BLE COM --START--
  aci_loop();//Process any ACI commands or events from the NRF8001- main BLE handler, must run often. Keep main loop short.
  if (ble_rx_buffer_len) {//Check if data is available
    SerialMonitor.print(ble_rx_buffer_len);
    SerialMonitor.print(" : ");
    SerialMonitor.println((char*)ble_rx_buffer);
    ble_rx_buffer_len = 0;//clear afer reading
  }
  //---- trial Addition for BLE COM --END--

  if (imu->IMURead()) {                                // get the latest data if ready yet

    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      SerialMonitor.print("Sample rate: "); SerialMonitor.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        SerialMonitor.println(", gyro bias valid");
      else
        SerialMonitor.println(", calculating gyro bias - don't move IMU!!");        
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      RTVector3 accelData=imu->getAccel();

      //if (SerialMonitor.available()) {//Check if serial input is available to send
        //delay(50);//should catch input
      if(isConnected()){
        sendBLE(accelData.x(), accelData.y(), accelData.z());
      }else{
        displayAxis("Accel:", accelData.x(), accelData.y(), accelData.z());        // accel data
      }
    }
  }
}

//---- trial Addition for BLE COM --START--
void sendBLE(float x, float y, float z)
{
  SerialMonitor.println("sendBLE function");
  if (ble_rx_buffer_len) {//Check if data is available
    SerialMonitor.print(ble_rx_buffer_len);
    SerialMonitor.print(" : ");
    SerialMonitor.println((char*)ble_rx_buffer);
    ble_rx_buffer_len = 0;//clear afer reading
  }
  
    uint8_t sendBuffer[21];
    uint8_t sendLength = 0;

    sendBuffer[sendLength] = x*100;
    sendLength++;
    sendBuffer[sendLength] = ',';
    sendLength++;
    sendBuffer[sendLength] = y*100;
    sendLength++;
    sendBuffer[sendLength] = ',';
    sendLength++;
    sendBuffer[sendLength] = z*100;
    sendLength++;
  
    sendBuffer[sendLength] = '\0'; //Terminate string
    sendLength++;
    SerialMonitor.print("sending ");SerialMonitor.println((char*)sendBuffer);
    if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, sendLength))
    {
      SerialMonitor.println(F("TX dropped!"));
    }
   
}
//---- trial Addition for BLE COM --END--

void displayAxis(const char *label, float x, float y, float z)
{
//  SerialMonitor.print(label);
  SerialMonitor.print(" x:"); SerialMonitor.print(x);
  SerialMonitor.print(", y:"); SerialMonitor.print(y);
  SerialMonitor.print(", z:"); SerialMonitor.print(z);
}
