/****************************************************************************************************************************\
 * Arduino project "ESP Easy" ï¿½ Copyright www.esp8266.nu
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * You received a copy of the GNU General Public License along with this program in file 'License.txt'.
 *
 * IDE download    : https://www.arduino.cc/en/Main/Software
 * ESP8266 Package : https://github.com/esp8266/Arduino
 *
 * Source Code     : https://sourceforge.net/projects/espeasy/
 * Support         : http://www.esp8266.nu
 * Discussion      : http://www.esp8266.nu/forum/
 *
 * Additional information about licensing can be found at : http://www.gnu.org/licenses
\*************************************************************************************************************************/

// This file is to be loaded onto an Arduino Pro Mini so it will act as a simple IO extender to the ESP module.
// Communication between ESP and Arduino is using the I2C bus, so only two wires needed.
// It best to run the Pro Mini on 3V3, although the 16MHz versions do not officially support this voltage level on this frequency.
// That way, you can skip levelconverters on I2C.
// Arduino Mini Pro uses A4 and A5 for I2C bus. ESP I2C can be configured but they are on GPIO-4 and GPIO-5 by default.

/****************************************************************************************************************************\
 Modified POLIN 2019  : (we are using an arduino Nano, but that's irrelevant) 
 to receive and transmit remote probes value for our homer system, via Cheap RF433 transmissionm using RFTXNodes (ATTINY based) 
 (reminder, RFTXnodes send a fixead size char array to (any) receiver using manchester.h, in there we have a sender (int) information 
 and (among other things) a value (unsigned long). 
 1) Added a function (and associated variables) void receiveManchester() which will constantly listen for RF433 Radio 
 and fill the table rftxNodeLastValue[index] with the received value (we don't use the rest) "index" is the sender number (0-254)
 1a) Receiver's (this code) myExpectedPassword must match messages 2 passwords (manchesterBuffer[2] and manchesterBuffer[10]
 2) by convention, ESPEasy can interrogate this table by calling a port number >= 100 using CMD_ANALOG_READ:
 as receiveEvent has been modified to present the rftxNodeLastValue[portnumber-100] value when MD_ANALOG_READ is called  (see code)
 3) ESPEasy OOtB simply has to be configured as follows 
 Device : Extra IO - ProMini Extender
 Name   : Anything 
 Enabled : On  
 Sensor Port : 100+index of sender (to be clear, sender 3 will have it's value stored in rftxNodeLastValue[3] so port must be 103)
 Sensor Port Type : Analog 
 Send to controller : yes 
 IDX : Idx of the device for domoticz
 interval : anything sensible 
 Values :  1 value, formula : whatever is relevant if needed , decimals, whatever is relevant  
 4) Receive commands is enabled via the ESPEasy i2c bus as follows : 
 in the receiveEvent function, if the i2c input command is CMD_DIGITAL_WRITE AND the port is > 100 then the variable 
 int i2cPseudoPortValue[(i2c command port - 100)] is filled in the the value read in the i2c command (normally 0 or 1) 
 the simplest way to pass this command is to send a http://espeasyadre/control?cmd=EXTGPIO,102,0
 5) rf433 radio is processed in the receivemanchester() function initiated by the infinite loop 
    commands are received by the I2c receiveEvent pseudo interrupt function that simply fills the table  i2cPseudoPortValue
	these commands are then processed, where there is time, in the processI2CValues() function that is initiated every myDelay microseconds. 
	
\*************************************************************************************************************************/
#include <Wire.h>
#include <Manchester.h>
#include <CRC32.h>

#define I2C_MSG_IN_SIZE    4
#define I2C_MSG_OUT_SIZE   4

#define CMD_DIGITAL_WRITE  1
#define CMD_DIGITAL_READ   2
#define CMD_ANALOG_WRITE   3
#define CMD_ANALOG_READ    4

volatile uint8_t sendBuffer[I2C_MSG_OUT_SIZE];
int loopcount = 0 ; 
//unsigned long rftxNodeLastValue[10] ;
//uint32_t rftxNodeLastValue[10] ;
//volatile int rftxNodeLastValue[50] ;
//volatile int i2cPseudoPortValue[50];
//volatile int i2cPseudoPortPort[50];
//volatile int i2cPseudoPortValueLast[50];
int rftxNodeLastValue[50] ;
int i2cPseudoPortValue[50];
int i2cPseudoPortPort[50];
int i2cPseudoPortValueLast[50];
#define VERSION 9
#define PO_MAN_BUFFER_SIZE 16
// temporarily increase buffer size to test corruption uint8_t manchesterBuffer[PO_MAN_BUFFER_SIZE];
uint8_t manchesterBuffer[255];
int MANRX_PIN = 10; 
int TX_PIN = 2;
//uint8_t ledOn = 1;
//int my_blueLed = 11; // pin showing reception mode 
//int my_greenLed = 6;
//int myRedLed = 6;
uint8_t my_blueLed = 4; // receiving PB2 = pin 5 
uint8_t my_greenLed = 5; // transmitting ok PB0 = pin 7
uint8_t my_redLed = 6;  // error somewhere PB5 = pin 1 //not that simple on ATtiny 
uint8_t myExpectedPassword = 123; 
unsigned long waitUntil = 0;
unsigned long processI2CDelayMicros = 1000000; 
unsigned long processI2CLastMicros = 0; 
unsigned long heartBeatDelayMicros = 60000000; 
unsigned long heartBeatLastMicros = 0; 
uint8_t payloadSentToCount[50] = {} ; 
// RF433 transmit values
         uint8_t myPassword=123;
         uint8_t myNodeNumber=5;
         uint8_t myLength=16;
         boolean ledOn=true; 
         const uint8_t repeatRFCommand = 3; 
void setup()
{
  Serial.begin(9600); 
  pinMode(my_blueLed, OUTPUT);
  pinMode(my_greenLed, OUTPUT);
  pinMode(my_redLed, OUTPUT);
  Wire.begin(0x7f); // change this address 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  man.setupReceive(MANRX_PIN, MAN_1200);
  man.setupTransmit(TX_PIN, MAN_1200);
  man.beginReceiveArray(PO_MAN_BUFFER_SIZE, manchesterBuffer);
}

void loop() {
  //Serial.println(" Loop"); 
  digitalWrite(my_greenLed, LOW); // TXMode off   
  digitalWrite(my_blueLed, HIGH); // manchester RX mode 
    char serialInput; 
        if (Serial.available() > 0) { // if anything on serial 
                // 
                serialInput = Serial.read();
                if (serialInput == 'l') { 
                  Serial.print(" RFrxd : ");
                   for (int n=0;n<50;n++){
                    Serial.print(n);
                    Serial.print(": ");
                    Serial.print(rftxNodeLastValue[n]);
                    Serial.print(" | "); 
                    if (((n+1)%10) == 0) {
                       Serial.println(" ") ;
                       Serial.print(" RFrxd : ");
                    }
                   }
                   Serial.println(" ") ;  
                   Serial.print(" i2Port : ");
                   for (int n=0;n<50;n++){                   
                    Serial.print(n);
                    Serial.print(" : ");
                    Serial.print(i2cPseudoPortPort[n]);
                    Serial.print(" | ");    
                    if (((n+1)%10) == 0) {
                       Serial.println(" ") ;
                       Serial.print(" i2Port : ");
                    }                     }
                   Serial.println(" ") ;  
                   Serial.print(" i2CMD : ");
                   for (int n=0;n<50;n++){                   
                    Serial.print(n);
                    Serial.print(" : ");
                    Serial.print(i2cPseudoPortValue[n]);
                    Serial.print(" | "); 
                    if (((n+1)%10) == 0) {
                       Serial.println(" ") ;
                       Serial.print(" i2CMD : ");
                    }                       
                   }
                }
                if (serialInput == 'v') { 
                  Serial.print(" Version : "); 
                  Serial.println(VERSION) ;
                }
        }
  receiveManchester();
      if (micros() - processI2CLastMicros > processI2CDelayMicros) { // process Gateway received commands every xx micros
           processI2CValues();
           processI2CLastMicros = micros();  
           digitalWrite(my_greenLed, HIGH); // TXMode on   
           digitalWrite(my_blueLed, LOW); //  RX mode off 
           if (micros() - heartBeatLastMicros > heartBeatDelayMicros) { //// heartbeat every xx seconds  
              Serial.println(" Sending Heartbeat ") ;
              prepareAndSendPayLoad( myLength, "HB_", 69, myPassword,  myNodeNumber);
              heartBeatLastMicros = micros(); 
           }
           digitalWrite(my_greenLed, LOW); // TXMode off   
           digitalWrite(my_blueLed, HIGH); // RX mode on  
           man.beginReceiveArray(PO_MAN_BUFFER_SIZE, manchesterBuffer); // immediately start to listen for another message 
      }
  }
void receiveEvent(int count)
{
  if (count == I2C_MSG_IN_SIZE)
  {
    byte cmd = Wire.read();
    byte port = Wire.read();
    int value = Wire.read();
    value += Wire.read()*256;
    switch(cmd)
      {
        case CMD_DIGITAL_WRITE:
          pinMode(port,OUTPUT);
          // if port is greater than 100, it means we want to tell arduino to do something later (not here are we are in the I2C interrupt 
          if (port > 100)
              {
            digitalWrite(my_redLed, HIGH); // error on just to debug 
            i2cPseudoPortValue[port-100]=value;   // force cast in small int     
            i2cPseudoPortPort[port-100]=int(port);   // force cast in int  (overkill) 
            if (port==114){ // POLIN todelete, this needs fixing : solve heap problem first... 
                i2cPseudoPortValue[1+port-100]=99; 
                int aa = 0  ; 
              }      
            }
          else {
            digitalWrite(port,value);            
            }
          break;
        case CMD_DIGITAL_READ:
          pinMode(port,INPUT_PULLUP);
          clearSendBuffer();
          sendBuffer[0] = digitalRead(port);
          break;
        case CMD_ANALOG_WRITE:
          analogWrite(port,value);
          break;
        case CMD_ANALOG_READ:
          clearSendBuffer();
          int valueRead ;
          int myport = int(port); 
          if (myport > 200){
            valueRead = int(rftxNodeLastValue[myport-200]);  
          }
          else {
            valueRead = analogRead(port);
          }
          sendBuffer[0] = valueRead & 0xff;
          sendBuffer[1] = valueRead >> 8;
          break;
      }
  }
}

void clearSendBuffer()
{
  for(byte x=0; x < sizeof(sendBuffer); x++)
    sendBuffer[x]=0;
}

void requestEvent()
{
  Wire.write((const uint8_t*)sendBuffer,sizeof(sendBuffer));
}

// check all values stored via i2c communications, if anything is there and was changed since last check, send it via RF433 TX 
void processI2CValues(){ 
     digitalWrite(my_redLed, LOW); // error off   
/****************************************************************************************************************************\
 i2cPseudoPortValue[pseudoportnumber] contains and integer (2 bytes) received by i2c (i2c port - 100) 
 convention is : 
etc ...
*  
** we scan the current table vs the "Last" one, and if there is a change (made by the i2c interrupt code) in the value we send to everyone, tagging the packet's verb with the target node number : 
* 
* 
*   Verb = node(actuator) number  
*   Value =  whatever is contained in the i2cpseudoport for a certain reference in the table, NOTE that this is an unsigned long (4Bytes) in the frame but can be any int value in the table (1 to 4 Bytes). 
*
\*************************************************************************************************************************/
  for (int nn=0;nn<50;nn++){
    char myVerb[4]="CMD";
    // now all commands sent from this code have a numeric verb which is the unique actuator number. 
    // strcpy(myVerb,RFCommands[myCommand]); 
    if (i2cPseudoPortValue[nn] != i2cPseudoPortValueLast[nn]) // if the value for that pseudo port has changed from the one in history 
		{
      unsigned long myValue=(i2cPseudoPortValue[nn]); // transmit value is the one read from the i2c command recently 
      snprintf(myVerb,4,"%03d",i2cPseudoPortPort[nn]); // copy the pseudoport (with the added 100 added during i2c command receive interrupt) in the verb as a CHAR (I know it's not optimized but I like it like that for debug) 
      Serial.print("--Sending a command ");
      Serial.print(myVerb);
      Serial.print(" pseudoPort: "); 
      Serial.print(nn); 
      Serial.print(" value: "); 
      Serial.println(myValue);
      prepareAndSendPayLoad( myLength, myVerb, myValue, myPassword,  myNodeNumber);
      payloadSentToCount[nn]++ ;
      if (payloadSentToCount[nn] >= repeatRFCommand) {  // We send the same command repeatRFCommand times to make sure it's transmitted (cannot trust RF433), it's the end node problem to manage multiple times the same command for the same actuator.
        Serial.println("command sent 3 times, removing it"); 
		    i2cPseudoPortValueLast[nn] = i2cPseudoPortValue[nn]; // copy current value to last table so we don't re-issue the command next loop.
        payloadSentToCount[nn]=0;
        } // end if payload...
		  } // end if i2c...
	  } // end for 
} // end function
// Prepare payload and execute transmission on the fly  
void prepareAndSendPayLoad(uint8_t myLength, char * myVerb, unsigned long myValue, uint8_t myPassword, uint8_t myNodeNumber ) {
// Data structure to send will be 
// [0]length of packet 
// [1]Originator # 
// [2]password  
// [3][10] payload (length 7) 
  // payload verb : [3][4][5]
  // payload value (unsigned long) [6][7][8][9] 
// [10] repeated password // e.g. paranoia 
// [11][12][13][14]CRC32 
// [15] \0 
  uint8_t myData[16]; // this contains the whole frame to send, including payload and contrl chars
// start of preparation   
  // reset data to nothing 
  myData[0] = '\0';  
  myData[0] = myLength; 
  myData[1] = myNodeNumber;  // identify Emitting node // POLIN have to find a way to set this value on each sensor of the RFTXNODE
  myData[2] = myPassword; 
  // verb ; 
  myData[3] = myVerb[0];
  myData[4] = myVerb[1];
  myData[5] = myVerb[2];
  // value 
  myData[6] = (byte) ((myValue & 0xFF000000) >> 24 );
  myData[7] = (byte) ((myValue & 0x00FF0000) >> 16 );
  myData[8] = (byte) ((myValue & 0x0000FF00) >> 8  );
  myData[9] = (byte) ((myValue & 0X000000FF)       );
//  int len = strlen(payLoad); // this should be strictly equal to 7 (terminating 0 is not counted by strlen ! 
//  uint8_t len = 7 ;
  myData[10]=myPassword; 
  myData[15] = '\0'; // end of cstring // position=15
  // prepare for CRC32 calculation
  size_t numBytes = 11; 
  uint32_t checksum = CRC32::calculate(myData, numBytes);
  //slot CRC at the end of the packet 
  myData[11] = (byte) ((checksum & 0xFF000000) >> 24 );
  myData[12] = (byte) ((checksum & 0x00FF0000) >> 16 );
  myData[13] = (byte) ((checksum & 0x0000FF00) >> 8  );
  myData[14] = (byte) ((checksum & 0X000000FF)       );
  // end of preparation , data is ready to be sent ! 
  man.stopReceive(); // stop receiving 
  // digitalWrite(GTXLed,HIGH); // transmitting 3 times the same frame hoping it's sufficient. 
  man.transmitArray(myLength, myData);
  //man.beginReceiveArray(PO_MAN_BUFFER_SIZE, manchesterBuffer); // immediately start to listen for another message 
 }
void receiveManchester() {
 if (man.receiveComplete()) 
    {
//    char tmps[40];
//    strncpy(tmps,manchesterBuffer,17);  
//    String str = (char *)manchesterBuffer;
//    Serial.println(" buffer: "+String(tmps)) ; 
//    Serial.print(" hex : ") ; 
//    for (int nn=0;nn<16;nn++){
//      Serial.print(byte(manchesterBuffer[nn]),HEX) ;Serial.print(".");
//    }
//   Serial.println("==");
//   Serial.print(byte(manchesterBuffer[11]),HEX) ;Serial.print(".");  
//   Serial.print(byte(manchesterBuffer[12]),HEX) ;Serial.print(".");  
//   Serial.print(byte(manchesterBuffer[13]),HEX) ;Serial.print(".");  
//   Serial.print(byte(manchesterBuffer[14]),HEX) ;Serial.print(".");  
//   Serial.println("..."); 
    digitalWrite(my_blueLed, HIGH); // manchester mode 
//   digitalWrite(led_pin, LedOn); // just show something is happening 
    // Data structure to receive is 
    // [0]length of packet 
    // [1]Originator # 
    // [2]password  
    // [3][10] payload (length 7) 
      // payload verb : [3][4][5]
      // deleteme payload separator []
      // payload value (unsigned long) [6][7][8][9] 
    // [10] repeated password // e.g. paranoia 
    // [11][12][13][14]CRC32 
    // [15] \0 
    
    // compute expected CRC 
    size_t numBytes = sizeof(manchesterBuffer) - 5; // manchesterBuffer - CRC32 information - final '\0' 
    numBytes = 11; 
    uint32_t myExpectedCRC = CRC32::calculate(manchesterBuffer, numBytes);
    uint8_t receivedSize = 0;
    char  my_Verb[4] = "" ; 
    uint8_t myManReceivedSize = manchesterBuffer[0];  // size in first byte
    uint8_t mySender = manchesterBuffer[1]; // who sent it ? 
    uint8_t myfirstPassword = manchesterBuffer[2];
    int m=5 ;
    //int o=10 ; int v=3;   
    boolean foundslash=false; 
    my_Verb[0] = manchesterBuffer[3];
    my_Verb[1] = manchesterBuffer[4];
    my_Verb[2] = manchesterBuffer[5];
    my_Verb[3] = '\0';
    unsigned long my_value = 0;
//    uint32_t my_value = 0;

    uint8_t d3 = manchesterBuffer[6] ;
    uint8_t d4 = manchesterBuffer[7] ;  
    uint8_t d5 = manchesterBuffer[8] ;  
    uint8_t d6 = manchesterBuffer[9] ;
    my_value = 0 ;
    my_value += (uint32_t)manchesterBuffer[6] << 24;
    my_value += (uint32_t)manchesterBuffer[7] << 16;
    my_value += (uint32_t)manchesterBuffer[8] << 8;
    my_value += (uint32_t)manchesterBuffer[9];  
    loopcount++; 
    Serial.print("Proextender Sequence: ");
    Serial.print(loopcount);
    Serial.print(" Verb ");
    Serial.print(my_Verb );
    Serial.print(" received from sender: ");
    Serial.print(mySender);
    Serial.print(" Value "); 
    Serial.println(my_value); 
    uint8_t mySecondPassword = manchesterBuffer[10] ; // one byte after the delimiter 
    // restore CRC from message 
    if ((mySecondPassword == myfirstPassword) && (mySecondPassword == myExpectedPassword )){
    uint32_t myReadCRC = 0;
    myReadCRC += (uint32_t)manchesterBuffer[11] << 24; 
    myReadCRC += (uint32_t)manchesterBuffer[12] << 16; 
    myReadCRC += (uint32_t)manchesterBuffer[13] << 8; 
    myReadCRC += (uint32_t)manchesterBuffer[14];
  
  //        long myReadCRC = (unsigned long)(manchesterBuffer[14] << 24) 
  //        | (manchesterBuffer[13] << 16) 
  //        | (manchesterBuffer[12] << 8) 
  //        | manchesterBuffer[11];
          if (myReadCRC == myExpectedCRC) { 
            digitalWrite(my_redLed, LOW); // error off   
            // fill out node last value table (using mySender number as index) 
            // note that the verb is totally ignored
            rftxNodeLastValue[manchesterBuffer[1]] = int(my_value) ;
       /*   keep this for debug
          Serial.print("Originator # ");
          Serial.print(manchesterBuffer[1]); 
          Serial.print(" total length: "); 
          Serial.print(String(manchesterBuffer[0])); 
          Serial.print(" opening password : "); 
          Serial.print(String(manchesterBuffer[2]));
          Serial.print(" closing password : "); 
          Serial.print(String(mySecondPassword));  
          Serial.print(" Verb : "); 
          Serial.print(my_Verb); 
          Serial.print(" pw 2--->: " + String(password2)+" pd3 "+String(pd3)+" pd4 "+String(pd4)+" pd5 "+String(pd5)+" pd6 "+String(pd6));
          Serial.print(" Value : "); 
          Serial.println(my_value);
          */
          } 
        else {
          Serial.print(" CRC error : read in packet : ");
          Serial.print(myReadCRC);
          Serial.print(" calculated : ");
          Serial.println(myExpectedCRC); 
          digitalWrite(my_redLed, HIGH); // error ON   
        }
    }
    else {
          //Serial.println(" Passwords dont match first: "+String(myfirstPassword)+ " second : "+String(mySecondPassword) + " expected : "+String(myExpectedPassword) ); 
        digitalWrite(my_redLed, HIGH); // error ON
        }
    man.beginReceiveArray(PO_MAN_BUFFER_SIZE, manchesterBuffer); // immediately start to listen for another message 
    //LedOn = !LedOn;
    //digitalWrite(my_blueLed, LOW); // end of reception process 
  }
}
