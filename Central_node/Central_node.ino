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
#include <EEPROM.h>
//////////////////////////////////////////////////////////////////////////////////////
//POWER SAVE
//s#define LOW_POWER true
int SetWU_Hour = 18;
int SetWU_Min = 35;

const int wakeUpPin = 2;  // Use pin 2 as wake up pin
bool hibernate=false;
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
//#define CONFIG_NODES 1
#define FORCE_DEFAULT_VALUE false
#define DEFAULT_CHANNEL 915.0
#define LORAMODE 0
#define MAX_DBM 5
#define DEBUG_MODE 1

// Singleton instance of the radio driver
RH_RF95 driver;
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

uint8_t node_address = SERVER_ADDRESS;
int loraMode = LORAMODE;
uint32_t setNewChannel;
uint8_t powerlevel;

struct Radioconfig {
  uint8_t flag1;
  uint8_t flag2;
  uint8_t seq;
  uint8_t addr;  
  unsigned int idle_period;  
  uint32_t channel;
  int loramode;
  uint8_t powerlevel;
  uint8_t SetWU_Hour;
  uint8_t SetWU_Min;
  uint8_t overwrite;
 // can add other fields such as LoRa mode,...
};Radioconfig my_Radioconfig;
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// Variables
uint8_t data[250] = "/@A4#H18#M45#I2#D12#C12#P5#O0##";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];


// CHANGE HERE THE TIME IN MINUTES BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriodInMin = 60; //20

#ifdef LOW_POWER
unsigned int nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
#endif

//Amount of data obtained
int setAmountoOfData = 10;
int numOfData = setAmountoOfData;

bool configNodes = false;
int numParam = 0;

int packetNumber = 0;

//
///////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//
void loadConfig(){
  //////////////////////////////////////////////////////////////////////////////////////
// get config from EEPROM
  EEPROM.get(0, my_Radioconfig);

  // found a valid config?
  if (my_Radioconfig.flag1==0xFF && my_Radioconfig.flag2==0x55) {
    Serial.println("Get back previous radio config\n");

    // set sequence number
    packetNumber=my_Radioconfig.seq;
    Serial.print("Using packet sequence number of ");
    Serial.println(packetNumber,DEC);
    
    #ifdef FORCE_DEFAULT_VALUE
        Serial.println("Forced to use default parameters");
        my_Radioconfig.flag1=0xFF;
        my_Radioconfig.flag2=0x55;
        my_Radioconfig.seq=packetNumber;
        my_Radioconfig.addr=SERVER_ADDRESS;
        my_Radioconfig.idle_period=idlePeriodInMin;
        my_Radioconfig.channel= DEFAULT_CHANNEL;
        my_Radioconfig.loramode = LORAMODE;
        my_Radioconfig.powerlevel = MAX_DBM;
        my_Radioconfig.SetWU_Hour = SetWU_Hour;
        my_Radioconfig.SetWU_Min = SetWU_Min;            
        my_Radioconfig.overwrite=0;
        EEPROM.put(0, my_Radioconfig);

        node_address=my_Radioconfig.addr;
        idlePeriodInMin=my_Radioconfig.idle_period;
        setNewChannel=my_Radioconfig.channel; 
        loraMode=my_Radioconfig.loramode;
        powerlevel=my_Radioconfig.powerlevel;
        SetWU_Hour=my_Radioconfig.SetWU_Hour; 
        SetWU_Min=my_Radioconfig.SetWU_Min;
    #else
        // get back the node_addr
        if (my_Radioconfig.addr!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored address");
            node_address=my_Radioconfig.addr;        
        }
        else
            Serial.println("Stored node addr is null"); 
    
        // get back the idle period
        if (my_Radioconfig.idle_period!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored idle period");
            idlePeriodInMin=my_Radioconfig.idle_period;        
        }
        else
            Serial.println("Stored idle period is null\n");   

        // get back the channel
        if (my_Radioconfig.channel!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored channel");
            setNewChannel=my_Radioconfig.channel;        
        }
        else
            Serial.println("Stored channel is null");     

        // get back the loramode
        if (my_Radioconfig.loramode!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored loramode\n");
            loraMode=my_Radioconfig.loramode;        
        }
        else
            Serial.println("Stored loramode is null"); 

        // get back the powerlevel
        if (my_Radioconfig.powerlevel!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored powerlevel\n");
            powerlevel=my_Radioconfig.powerlevel;        
        }
        else
            Serial.println("Stored powerlevel is null\n");

        // get back the Hour
        if (my_Radioconfig.SetWU_Hour!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored SetWU_Hour");
            SetWU_Hour=my_Radioconfig.SetWU_Hour;        
        }
        else
            Serial.println("Stored SetWU_Hour is null");      

       // get back the Min
        if (my_Radioconfig.SetWU_Hour!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored SetWU_Min");
            SetWU_Min=my_Radioconfig.SetWU_Min;        
        }
        else
            Serial.println("Stored SetWU_Min is null"); 
                   
    #endif  
        
        Serial.print("Using node addr of ");
        Serial.println(node_address,DEC);
        
        Serial.print("Using idle period of ");
        Serial.println(idlePeriodInMin,DEC);
        
      }
  else {
    // otherwise, write config and start over
    
    my_Radioconfig.flag1=0xFF;
    my_Radioconfig.flag2=0x55;
    my_Radioconfig.seq=packetNumber;
    my_Radioconfig.addr=SERVER_ADDRESS;
    my_Radioconfig.idle_period=idlePeriodInMin;
    my_Radioconfig.channel= DEFAULT_CHANNEL;
    my_Radioconfig.loramode = LORAMODE;
    my_Radioconfig.powerlevel = MAX_DBM;
    my_Radioconfig.SetWU_Hour = SetWU_Hour;
    my_Radioconfig.SetWU_Min = SetWU_Min;    
    my_Radioconfig.overwrite=1;
    EEPROM.put(0, my_Radioconfig);
    delay(2000);
    Serial.println("First time boot, please reset the node..!");
    }
//
//////////////////////////////////////////////////////////////////////////////////////
  
}


//
//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// arduino setup function
void setup() 
{
  
  Serial.println("CONFIGURING CENTRAL NODE");
 Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available

  loadConfig();

  if (!manager.init()){ // Defaults after init are 915.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    Serial.println("INIT FAIL..! reboot NODE..!!");
    while(true){
 
    }
  }
   
  driver.setModeIdle();

  switch (loraMode){
    case 0:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128))
              break;
            else
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");
    case 1:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128))
              break;
            else
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");

   case 2:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512))
              break;
            else
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");

   case 3:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096))
              break;
            else
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");

   default:
      
           Serial.println("Unrecognized cmd");       
           break;
    
  }

    
  driver.setThisAddress(node_address);

  #if (DEBUG_MODE > 0)
      Serial.print("Node address is: ");
      Serial.println(manager.thisAddress(),DEC);
  
  #endif
  
  driver.setPreambleLength(8); // Default is 8
 
  if(!driver.setFrequency(setNewChannel))
    Serial.println("Error in frequency setting");
 
  driver.setTxPower(powerlevel);
  
  Serial.println("Radio modem successfully configured..!");

//////////////////////////////////////////////////////////////////////////////////////
//Init RTC module
//#ifdef LOW_POWER
//
//  //Set pin D3 as INPUT for accepting the interrupt signal from DS3231
//  //pinMode(wakePin, INPUT);
//
//  //Initialize communication with the clock
// 
//  RTC.begin();
//  RTC.adjust(DateTime(__DATE__, __TIME__));   //set RTC date and time to COMPILE time
//  
//  //clear any pending alarms
//  RTC.armAlarm(1, false);
//  RTC.clearAlarm(1);
//  RTC.alarmInterrupt(1, false);
//  RTC.armAlarm(2, false);
//  RTC.clearAlarm(2);
//  RTC.alarmInterrupt(2, false);
//
//  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
//  //The output of the DS3231 INT pin is connected to this pin
//  //It must be connected to arduino D2 pin for wake-up
//  RTC.writeSqwPinMode(DS3231_OFF);
//
//  //Set alarm1 every day at 18:30
//  RTC.setAlarm(ALM1_MATCH_HOURS,0,SetWU_Min, SetWU_Hour,0);   //set your wake-up time here
//  RTC.alarmInterrupt(1, true);
//  Serial.println("RTC successfully configured");
//#endif
//
//////////////////////////////////////////////////////////////////////////////////////

}
//
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
 
    // Extract floating part
    float fpart = n - (float)ipart;
 
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
 
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot
 
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
 
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
//
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

  Serial.println("Wait for a broadcast..!");
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
      Serial.println(buf[16]);
      Serial.println(sizeof(buf),DEC);
      Serial.println();

      // Send a reply back to the originator client
      Serial.println("Sending ACK...");
      if (!manager.sendtoWait(data, sizeof(data), 3))
        Serial.println("sendtoWait failed");
      else
        Serial.println("Sent ACK OK...!");

      delay(500);
//      Serial.println("Sending data to GW");
//      while(!manager.sendtoWait(buf, sizeof(buf), GW_ADDRESS)){
//        delay(100); 
//      }
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


