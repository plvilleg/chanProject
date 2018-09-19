// rf95_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

//////////////////////////////////////////////////////////////////////////////////////
//POWER SAVE
#define LOW_POWER 0
int SetWU_Hour = 18;
int SetWU_Min = 35;

const byte wakeUpPin = 3;  // Use pin 3 as wake up pin
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
#define CLIENT_ADDRESS 3
#define SERVER_ADDRESS 2
#define CONFIG_NODES 1
#define FORCE_DEFAULT_VALUE
#define DEFAULT_CHANNEL 915.00
#define LORAMODE 0
#define MAX_DBM 5
#define DEBUG_MODE 1

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

uint8_t node_address = CLIENT_ADDRESS;
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
  uint8_t AoD;
  uint16_t dataFileLogcount;
  uint16_t dataFileLogver;
  uint8_t overwrite;
 // can add other fields such as LoRa mode,...
};Radioconfig my_Radioconfig;

const uint32_t CH_00_900 = 903.08; // channel 00, central freq = 903.08MHz
const uint32_t CH_01_900 = 905.24; // channel 01, central freq = 905.24MHz
const uint32_t CH_02_900 = 907.40; // channel 02, central freq = 907.40MHz
const uint32_t CH_03_900 = 909.56; // channel 03, central freq = 909.56MHz
const uint32_t CH_04_900 = 911.72; // channel 04, central freq = 911.72MHz
const uint32_t CH_05_900 = 913.88; // channel 05, central freq = 913.88MHz
const uint32_t CH_06_900 = 916.04; // channel 06, central freq = 916.04MHz
const uint32_t CH_07_900 = 918.20; // channel 07, central freq = 918.20MHz
const uint32_t CH_08_900 = 920.36; // channel 08, central freq = 920.36MHz
const uint32_t CH_09_900 = 922.52; // channel 09, central freq = 922.52MHz
const uint32_t CH_10_900 = 924.68; // channel 10, central freq = 924.68MHz
const uint32_t CH_11_900 = 926.84; // channel 11, central freq = 926.84MHz
const uint32_t CH_12_900 = 915.00; // default channel 915MHz, the module is configured with it

//
//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
// Variables
DateTime unixTime;
uint32_t timestamp;

uint8_t data[100] = "Node 1";
uint8_t message[RH_RF95_MAX_MESSAGE_LEN];
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

// CHANGE HERE THE TIME IN MINUTES BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriodInMin = 0.1; //20

#ifdef LOW_POWER
unsigned int nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
#endif

//Amount of data obtained
int setAmountoOfData = 10;
int numOfData;

bool configNodes = false;
int numParam = 0;

int packetNumber = 0;

const int SDcardSelect = 53;

File dataFileLog;
File eventFileLog;
String bufferLog;
//
//////////////////////////////////////////////////////////////////////////////////////

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
        my_Radioconfig.addr=CLIENT_ADDRESS;
        my_Radioconfig.idle_period=idlePeriodInMin;
        my_Radioconfig.channel= DEFAULT_CHANNEL;
        my_Radioconfig.loramode = LORAMODE;
        my_Radioconfig.powerlevel = MAX_DBM;
        my_Radioconfig.SetWU_Hour = SetWU_Hour;
        my_Radioconfig.SetWU_Min = SetWU_Min;
        my_Radioconfig.AoD = setAmountoOfData;
        my_Radioconfig.overwrite=1;
        EEPROM.put(0, my_Radioconfig);

        node_address=my_Radioconfig.addr;
        idlePeriodInMin=my_Radioconfig.idle_period;
        setNewChannel=my_Radioconfig.channel; 
        loraMode=my_Radioconfig.loramode;
        powerlevel=my_Radioconfig.powerlevel;
        SetWU_Hour=my_Radioconfig.SetWU_Hour; 
        SetWU_Min=my_Radioconfig.SetWU_Min;
        setAmountoOfData=my_Radioconfig.AoD;
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

            // Amount of data
        if (my_Radioconfig.AoD!=0 && my_Radioconfig.overwrite==1) {
          
            Serial.println("Used stored setAmountoOfData");
            setAmountoOfData=my_Radioconfig.AoD;       
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
    my_Radioconfig.addr=CLIENT_ADDRESS;
    my_Radioconfig.idle_period=idlePeriodInMin;
    my_Radioconfig.channel= DEFAULT_CHANNEL;
    my_Radioconfig.loramode = LORAMODE;
    my_Radioconfig.powerlevel = MAX_DBM;
    my_Radioconfig.SetWU_Hour = SetWU_Hour;
    my_Radioconfig.SetWU_Min = SetWU_Min;   
    my_Radioconfig.AoD = setAmountoOfData; 
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
//
#ifdef CONFIG_NODES
long getCmdValue(int &i, char* strBuff=NULL) {
  
    char seqStr[7]="******";
    
    int j=0;
    // character '#' will indicate end of cmd value
    while ((char)message[i]!='#' && (i < strlen((char*)message)) && j<strlen(seqStr)) {
            seqStr[j]=(char)message[i];
            i++;
            j++;
    }
    
    // put the null character at the end
    seqStr[j]='\0';
    
    if (strBuff) {
            strcpy(strBuff, seqStr);        
    }
    else
            return (atol(seqStr));
}   
#endif
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// open the file. note that only one file can be open at a time,
// so you have to close this one before opening another.
bool eventWriteLog(String tmpbuff){
  
  timestamp= millis();
  
  eventFileLog = SD.open("eventdatalog.txt", FILE_WRITE);

  bufferLog = "";
  bufferLog += String(timestamp);
  bufferLog += ",";
  bufferLog += tmpbuff;

  if (eventFileLog) {
    eventFileLog.println(bufferLog);
    eventFileLog.close();
    return true;
  }
   else {
    return false;
  }  
}

bool dataWriteLog(String bufferLog){

  timestamp= millis();


  if(dataFileLogcount > 8000)
     dataFileLogver++;
     
  dataFileLog = SD.open("Datalog"+String(dataFileLogver)+".txt", FILE_WRITE);

  bufferLog = "";
  bufferLog += String(timestamp);
  bufferLog += ",";
  bufferLog += tmpbuff;

  if (dataFileLog) {
    dataFileLog.println(bufferLog);
    dataFileLog.close();
    //dataFileLogcount++;
    return true;
  }
   else {
    return false;
  }  
}

//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// Reset function
void resetNode(){
  Serial.println("Restarting..!!");
  delay(1000);
}

//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// arduino setup function
void setup() 
{
  Serial.println("CONFIGURING NODE 1");

  pinMode(wakeUpPin, INPUT_PULLUP);

  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available

  //Wire.begin();

//////////////////////////////////////////////////////////////////////////////////////
////Init RTC module
#ifdef LOW_POWER
//Set pin D3 as INPUT for accepting the interrupt signal from DS3231
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
#endif
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//  Config system time
    //timestamp= unixTime.unixtime();
//  
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// SD card Config
Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SDcardSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //while (1);
    //resetNode();
  }
  Serial.println("card initialized.");

//
//////////////////////////////////////////////////////////////////////////////////////


  loadConfig();

  if (!manager.init()){ // Defaults after init are 915.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    Serial.println("INIT FAIL..! reboot NODE..!!");
    if(!eventWriteLog("INIT FAIL..! reboot NODE..!!"))
      resetNode();
  }
  
  driver.setModeIdle();

  switch (loraMode){
    case 0:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128))
              break;
            else{
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");
              if(!eventWriteLog("SET LoraMode FAIL..! reboot NODE..!!"))
                resetNode();
            }
    case 1:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128))
              break;
            else{
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");
              if(!eventWriteLog("SET LoraMode FAIL..! reboot NODE..!!"))
                resetNode();
            }
   case 2:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512))
              break;
            else{
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");
              if(!eventWriteLog("SET LoraMode FAIL..! reboot NODE..!!"))
                resetNode();
            }

   case 3:
            if(driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096))
              break;
            else{
              Serial.println("SET LoraMode FAIL..! reboot NODE..!!");
              if(!eventWriteLog("SET LoraMode FAIL..! reboot NODE..!!"))
                resetNode();
            }

   default:
      
           Serial.println("Unrecognized cmd"); 
           if(!eventWriteLog("Unrecognized cmd"))
                resetNode();      
           break;
    
  }

    
  manager.setThisAddress(node_address);

  #if (DEBUG_MODE > 0)
      Serial.print("Node address is: ");
      Serial.println(manager.thisAddress(),DEC);
  
  #endif
  
  driver.setPreambleLength(8); // Default is 8
 
  if(!driver.setFrequency(setNewChannel))
    Serial.println("Error in frequency setting");
 
  driver.setTxPower(powerlevel);
  
  Serial.println("Radio modem successfully configured..!");

   bufferLog += String(millis());
   bufferLog += ",";
   bufferLog+="Radio modem successfully configured..!";
  
   numOfData = setAmountoOfData;


   
  
//////////////////////////////////////////////////////////////////////////////////////
// Detph
//   Wire.begin();
//  while (!sensor.init()) {
//    Serial.println("Init failed!");
//    Serial.println("Are SDA/SCL connected correctly?");
//    Serial.println("Blue Robotics Bar30: White = SDA, Green = SCL");
//    Serial.println();
//    delay(5000);
//  }
//  sensor.setModel(MS5837::MS5837_30BA);
//  sensor.setFluidDensity(1003); // kg/m^3 (freshwater, 1029 for seawater)
//  Serial.println("MS5837 successfully configured");
//
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//Boot log

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


//////////////////////////////////////////////////////////////////////////////////////
// Sensing and transmition 
  float depth=0;
  char aux[6] = "";
  char final_str[80] = "\\$";
  Serial.println("Sensing..!");

  depth = 130.0; //sensor.depth();

  ftoa(depth,aux, 2);

  sprintf(final_str, "%s/%s/%s", final_str, "MS5837", aux);


  sprintf((char*)data, final_str);

   Serial.println("Sensed..! OK");

   #if (DEBUG_MODE > 0)

  
   #endif
  
  // Send a message to manager_server
  Serial.println("Sending data..!");
  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
  {
    Serial.println("Sent data OK..!");

    //////////////////////////////////////////////////////////
    // Window of reception
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (manager.recvfromAckTimeout(buf, &len, 3000, &from))
    {
      Serial.print("got reply from CENTRAL NODE: ");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("No reply, is CENTRAL NODE running?");
    }
    //
    //////////////////////////////////////////////////////////
  
  }
  else
    Serial.println("sendtoWait failed");
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//
#ifdef CONFIG_NODES
    
    if(configNodes){  
    memset(message,'\0',RH_RF95_MAX_MESSAGE_LEN);
    uint16_t  r_size; 
    EEPROM.get(0, my_Radioconfig);
    char sync[]="(@N";
    sprintf(sync,"%s%d",sync,my_Radioconfig.addr);
    r_size=sprintf((char*)message, sync);
    
    
    do{

         
      Serial.println((char*)message);
    
     
            
      if (manager.sendtoWait(message, r_size, SERVER_ADDRESS))
      {
      
      // rutine of update
        Serial.print("Sending: ");
        Serial.println((char*)(message)); 
        Serial.println("Packet sent OK! ");
        
        memset(message,'0',RH_RF95_MAX_MESSAGE_LEN);
        Serial.println("Wait for config message...");

        uint8_t from;
        
        if (manager.recvfromAckTimeout(message, RH_RF95_MAX_MESSAGE_LEN, 3000, &from))
        {
          Serial.print("got message config from GW: ");
          Serial.println((char*)message);
          
          driver.setModeIdle();

         int i=0;
         int cmdValue;
    

        i=0;

        // commands have following format /@A6#
        //
        if (message[i]=='/' && message[i+1]=='@') {
    
            Serial.println("Parsing command");      
            i=i+2;   

          while(message[i]!='#'){

            switch (message[i]) {

                  // set the node's address, /@A10# to set the address to 10 for instance
                  case 'A': 

                      i++;
                      cmdValue=getCmdValue(i);
                      
                      // cannot set addr greater than 255
                      if (cmdValue > 254)
                              cmdValue = 254;
                      // cannot set addr lower than 2 since 0 is broadcast and 1 is for gateway
                      if (cmdValue < 2)
                              cmdValue = 3;
                      // set node addr        
                                           
                      // Set the node address and print the result                                           
                      Serial.print("Setting LoRa node address to ");
                      Serial.println(cmdValue); 
                      Serial.println("..."); 
                      
                      manager.setThisAddress(cmdValue);
                      delay(100);
                      Serial.print("LoRa node address set to: ");
                      Serial.println(manager.thisAddress(),DEC);
           
                      // save new node_addr in case of reboot
                      my_Radioconfig.addr=cmdValue;
                      my_Radioconfig.overwrite=1;
                      EEPROM.put(0, my_Radioconfig);
            

                      break;        

                  // set the time between 2 transmissions, /@I10# to set to 10 minutes for instance
                  case 'I': 

                      i++;
                      cmdValue=getCmdValue(i);

                      // cannot set addr lower than 1 minute
                      if (cmdValue < 1)
                              cmdValue = idlePeriodInMin;
                      // idlePeriodInMin      
                      idlePeriodInMin=cmdValue; 

                 
                      
                      Serial.print("Set duty-cycle to ");
                      Serial.println(idlePeriodInMin,DEC);  
                     
                      // save new idle_period in case of reboot
                      my_Radioconfig.idle_period=idlePeriodInMin;
                      my_Radioconfig.overwrite=1;
                      EEPROM.put(0, my_Radioconfig);
                      break;  

                  // Hour of the day to wakeUp 0 - 23
                  case 'H': 

                      i++;
                      cmdValue=getCmdValue(i);

                      // cannot set hour lower than 0 hours
                      if (cmdValue < 0 )
                              cmdValue = 0;
                      // cannot set hour greater than 23 hours
                      if (cmdValue > 23  )
                              cmdValue = 23;
                              
                      // WakeUp hour      
                      SetWU_Hour=cmdValue; 
                      
                      
                      Serial.print("Set hour to: ");
                      Serial.println(SetWU_Hour,DEC);  
                          
                      // save new SetWU_Hour in case of reboot
                      my_Radioconfig.SetWU_Hour=SetWU_Hour;
                      my_Radioconfig.overwrite=1;
                      EEPROM.put(0, my_Radioconfig);
                      break;  


                  // Minute of hour to wakeUp 0 - 59
                  case 'M': 

                      i++;
                      cmdValue=getCmdValue(i);

                      // cannot set minute lower than 0 minutes
                      if (cmdValue < 0 )
                              cmdValue = 0;
                      // cannot set minute greater than 59 minutes
                      if (cmdValue > 59  )
                              cmdValue = 59;
                              
                      // WakeUp minute      
                      SetWU_Min=cmdValue; 
                      
                      Serial.print("Set minute to ");
                      Serial.println(SetWU_Min,DEC);  
                    
                      // save new SetWU_Min in case of reboot
                      my_Radioconfig.SetWU_Min=SetWU_Min;
                      my_Radioconfig.overwrite=1;
                      EEPROM.put(0, my_Radioconfig);
                      break;  


                  // Amount of data to acdquire, should be greater than 0 and don't excess the time of operation of the central node
                  case 'D': 

                      i++;
                      cmdValue=getCmdValue(i);

                      // cannot set data lower than 0
                      if (cmdValue < 0 )
                              cmdValue = 1;
                    
                      // Amount of data     
                      setAmountoOfData=cmdValue;
                      // save new SetWU_Min in case of reboot
                      my_Radioconfig.AoD=cmdValue;
                      my_Radioconfig.overwrite=1;
                      EEPROM.put(0, my_Radioconfig);
                      
                      Serial.print("Set amount of data to ");
                      Serial.println(cmdValue,DEC);  
                      break;              


                  // set the node's address, /@A10# to set the address to 10 for instance
                  case 'C': 

                      i++;
                      //uint32_t newChannel = DEFAULT_CHANNEL;
                      cmdValue=getCmdValue(i);
                      
                      
                      // cannot set channel greater than 12
                      if (cmdValue > 12)
                              setNewChannel = DEFAULT_CHANNEL;
                      // cannot set channel lower than 0
                      if (cmdValue < 0)
                              setNewChannel = DEFAULT_CHANNEL;

                      if(cmdValue == 0 )
                          setNewChannel = CH_00_900;

                      if(cmdValue == 1 )
                          setNewChannel = CH_01_900;

                      if(cmdValue == 2 )
                          setNewChannel = CH_02_900;

                      if(cmdValue == 3 )
                          setNewChannel = CH_03_900;

                      if(cmdValue == 4 )
                          setNewChannel = CH_04_900;

                      if(cmdValue == 5 )
                          setNewChannel = CH_05_900;

                      if(cmdValue == 6 )
                          setNewChannel = CH_06_900;

                      if(cmdValue == 7 )
                          setNewChannel = CH_07_900;

                      if(cmdValue == 8 )
                          setNewChannel = CH_08_900;

                      if(cmdValue == 9 )
                          setNewChannel = CH_09_900;

                      if(cmdValue == 10 )
                          setNewChannel = CH_10_900;
                        
                      if(cmdValue == 11 )
                          setNewChannel= CH_11_900;

                      if(cmdValue == 12 )
                          setNewChannel = CH_12_900;
                      
                    
                      Serial.print("Set LoRa node channel to ");
                      Serial.println(cmdValue,DEC);  
                      //driver.setModeIdle();
                                           
                      // Select frequency channel
                      if(!driver.setFrequency(setNewChannel))
                          Serial.println("Error in frequency setting");
                      Serial.println("Setting Channel OK");
          
                      // save new node_addr in case of reboot
                      my_Radioconfig.channel=setNewChannel;
                      my_Radioconfig.overwrite=1;
                      EEPROM.put(0, my_Radioconfig);
                      break; 


              // Set the power of trasmition
                case 'P': 

                    i++;
                    cmdValue=getCmdValue(i);

                    // cannot set channel greater than 12
                    if (cmdValue > 14)
                      cmdValue = MAX_DBM;
                      // cannot set channel lower than 0
                    if (cmdValue < 0)
                      cmdValue = 5;
                    
                    driver.setTxPower(cmdValue);
                    Serial.print("Set Power");
                                      
                    // save new power in case of reboot
                    my_Radioconfig.powerlevel=cmdValue;
                    my_Radioconfig.overwrite=1;
                    EEPROM.put(0, my_Radioconfig);
                   
                    break;                                                               

              // Set the power of trasmition
                case 'O': 
                    i++;
                    cmdValue=getCmdValue(i);
                    // cannot set mode greater than 11 (11 being the LoRaWAN test mode)
                   
                    //driver.setModeIdle();

                    switch (cmdValue){
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
                    
                    Serial.print("Set LoRa mode to ");
                    Serial.println(cmdValue,DEC);
                    
                   // save new operation mode in case of reboot
                   my_Radioconfig.loramode=cmdValue;
                   my_Radioconfig.overwrite=1;
                   EEPROM.put(0, my_Radioconfig);
                  
                    
                    break;                                                               

                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  // add here new commands
                  //  

                  //
                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

                  default:
      
                    Serial.println("Unrecognized cmd");       
                    break;
            }

            i++;
          }
          delay(500);
          configNodes = false;            
        }



          
        }
        else
        {
          Serial.println("No packet");
        }
      
      
      
      } 
      else{
        Serial.println("Communications problems");  
       
      }
       
         
        
    }while(configNodes);
 
   }
#endif
//
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//  low power
#ifdef LOW_POWER

      bool e;
      
      Serial.println("Switch to power saving mode");

      e = driver.sleep();

      if (e)
        Serial.println("Successfully switch LoRa module in sleep mode");
      else  
        Serial.println("Could not switch LoRa module in sleep mode");
 
      delay(500);
    
      nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
     

      if(numOfData == 1){
        configNodes = true;        
      }
      
      if(numOfData == 0){
        hibernate = true;        
      }
  
      if(hibernate){
               
        Serial.println("Going to hiberbate");
        delay(500);  

        // Allow wake up pin to trigger interrupt on low.
        attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW); //HIGH
        // Enter power down state with ADC and BOD module disabled.
        // Wake up when wake up pin is low.
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
            
        // Disable external pin interrupt on wake up pin.
             
//        LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, 
//                TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF, 
//                USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);

        detachInterrupt(digitalPinToInterrupt(wakeUpPin)); 

        //clear any pending alarms
//        RTC.armAlarm(1, false);
//        RTC.clearAlarm(1);
//        RTC.alarmInterrupt(1, false);
//
//        //Set alarm1 every day at 18:30
//        RTC.setAlarm(ALM1_MATCH_HOURS,0,SetWU_Min , SetWU_Hour,0);
//        // RTC.setAlarm(ALM1_MATCH_HOURS,0,20 , 16,0);   //set your wake-up time here
//        RTC.alarmInterrupt(1, true);

          delay(1000);
          Serial.println("Wake up");
          hibernate = false;
          numOfData = setAmountoOfData;
          
      } else {
         Serial.print("Next transmition at (min): ");
         Serial.println(idlePeriodInMin,DEC);

         delay(500);
         
         for (int i=0; i<nCycle; i++) {   
    
           LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
   
        }
      }
      Serial.print(numOfData,DEC);
      Serial.println();
      
      numOfData--;
      delay(50);

#endif
//
//////////////////////////////////////////////////////////////////////////////////////

}
//
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

