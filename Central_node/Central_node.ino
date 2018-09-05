// rf95_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

//////////////////////////////////////////////////////////////////////////////////////
//POWER SAVE
#define LOW_POWER true
int SetWU_Hour = 18;
int SetWU_Min = 35;

const int wakeUpPin = 2;  // Use pin 2 as wake up pin
bool hibernate=true;
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//
#ifdef LOW_POWER
#define LOW_POWER_PERIOD 8
// you need the LowPower library from RocketScream
// https://github.com/rocketscream/Low-Power
#include "LowPower.h"
#include <RTClibExtended.h>
RTC_DS3231 RTC;      //we are using the DS3231 RTC
#endif
//
//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
// Modem config
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define GW_ADDRESS 100

// Singleton instance of the radio driver
RH_RF95 driver;
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// Variables
uint8_t data[250] = "Central node";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

// CHANGE HERE THE TIME IN MINUTES BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriodInMin = 60; //20

unsigned int nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;

//Amount of data obtained
int setAmountoOfData = 10;
int numOfData = setAmountoOfData;

bool configNodes = false;
int numParam = 0;

//
///////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// arduino setup function
void setup() 
{
  
  Serial.println("CONFIGURING CENTRAL NODE");
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 915.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  Serial.println("Radio modem successfully configured..!");


//////////////////////////////////////////////////////////////////////////////////////
//Init RTC module
#ifdef LOW_POWER

  //Set pin D3 as INPUT for accepting the interrupt signal from DS3231
  //pinMode(wakePin, INPUT);

  //Initialize communication with the clock
 
  RTC.begin();
  RTC.adjust(DateTime(__DATE__, __TIME__));   //set RTC date and time to COMPILE time
  
  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);

  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  RTC.writeSqwPinMode(DS3231_OFF);

  //Set alarm1 every day at 18:30
  RTC.setAlarm(ALM1_MATCH_HOURS,0,SetWU_Min, SetWU_Hour,0);   //set your wake-up time here
  RTC.alarmInterrupt(1, true);
  Serial.println("RTC successfully configured");
#endif
//
//////////////////////////////////////////////////////////////////////////////////////

}
//
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//  here the interrupt is handled after wakeup
void wakeUp()       
{
}
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// loop arduino function
void loop()
{

//////////////////////////////////////////////////////////////////////////////////////
//  Receiving 
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got data from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);

      // Send a reply back to the originator client
      Serial.println("Sending ACK...");
      if (!manager.sendtoWait(data, sizeof(data), CLIENT_ADDRESS))
        Serial.println("sendtoWait failed");
      else
        Serial.println("Sent ACK OK...!");

      delay(500);
      Serial.println("Sending data to GW");
      while(!manager.sendtoWait(buf, sizeof(buf), GW_ADDRESS)){
        delay(100); 
      }
        Serial.println("Sent data GW OK..!");
    }
  }
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//  low power
#ifdef LOW_POWER
      
    DateTime now = RTC.now();

//    Serial.print(now.hour(), DEC);
//    Serial.print(':');
//    Serial.print(now.minute(), DEC);
//    Serial.print(':');
//    Serial.print(now.second(), DEC);
//    Serial.println();
    
    uint8_t set_hour =6;
    uint8_t set_minute = 45;

    if(((now.hour()-set_hour)==0) && ((now.minute()-set_minute) >= 0)){
       hibernate = true;
    }
  
      if(hibernate){
               
        Serial.println("Going to hiberbate");
        delay(10);  

        bool e;
      
        Serial.println("Switch to power saving mode");

        e = driver.sleep();

        if (e)
          Serial.println("Successfully switch LoRa module in sleep mode");
        else  
          Serial.println("Could not switch LoRa module in sleep mode");
 
        delay(50);

        // Allow wake up pin to trigger interrupt on low.
        attachInterrupt(0, wakeUp, LOW); //HIGH
        // Enter power down state with ADC and BOD module disabled.
        // Wake up when wake up pin is low.
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
            
        // Disable external pin interrupt on wake up pin.
             
//        LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, 
//                TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF, 
//                USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);

        detachInterrupt(0); 

//        //clear any pending alarms
//        RTC.armAlarm(1, false);
//        RTC.clearAlarm(1);
//        RTC.alarmInterrupt(1, false);
//
//        //Set alarm1 every day at 18:30
//        RTC.setAlarm(ALM1_MATCH_HOURS,0,30, 18,0);   //set your wake-up time here
//        RTC.alarmInterrupt(1, true);
          
          Serial.println("Wake up..!!!");
          hibernate = false;
      }
      delay(50);

#endif
//
//////////////////////////////////////////////////////////////////////////////////////  
}
//
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


