#include <CRC32.h>
#include <Manchester.h>
#include <DiOremote.h>
//#include <avr/io.h>
#include <DHT.h>
/*
 * Use IRremote library from https://github.com/z3t0/Arduino-IRremote/archive/master.zip 
 * by default,the IR LEDs are connected to Arduino PWM pin 3. but I had to change it as it conflicts with the manchester library timer. 
 * the IRremote library has therefore been updated to use pin 9. 
 * Originally by Pascal Olin may 2019
 * modified summer 2020 to remove non node specific functions, all functions are now run by the GenericHomerRelay.v??.h library located in the ../../nano/libraries folder (use latest version) 
 * modified november 2021 to enhance self contain documentation, move led pins to Analog (unused) pins to freeup digital pins
 * modified november 2021 to allow relays to report their status. 
 * **** Attention this version is now incompatible with version < 20 ****
 */
#include <IRremote.h>
IRsend irsend;
#define DEBUG 
//#undef DEBUG
#define DHTPRESENT
//#undef DHTPRESENT
#ifdef DEBUG
#define SerialprintF(x) Serial.print(F(x));
#define SerialprintlnF(x) Serial.println(F(x));
#define Serialprint(x) Serial.print(x);
#define Serialprintln(x) Serial.println(x);

#else
#define Serialprint(x) ;
#define Serialprintln(x) ;
#define SerialprintF(x) ;
#define SerialprintlnF(x) ;
#endif

#if defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define Serialprint(x) x;
#define Serialprintln(x) x;
#define SerialprintF(x) ;
#define SerialprintlnF(x) ;

#else
#define Serialprint(x) Serial.print(x);
#define Serialprintln(x) Serial.println(x);
#endif
 
/****************************************************************************************************************************\
 * ATTiny85 Expected wiring :  
 *                                  _______
 *                )<===  PB5/ADC0 -|1     8|- VCC
 *  RF TX data pin <===  PB3/ADC3 -|2     7|- PB2/ADC1/SCK ===> TX LED (Green) relay 2
 *     Sensor(5v)  <===  PB4/ADC2 -|3     6|- PB1/MISO     ===> RF RX data pin
 *                            GND -|4_____5|- PB0/MOSI     ===> RX Led (Blue) relay 1 
 * 
 * 
 *
 * Nano Expected wiring :  
 *                                         USB 
 *                                       __|  |_
 *                      <===  SCK/D13  -|  |__| |- MISO/D12      ==> DHT if present
 *                      <===  3.3V Out -|       |- OC2/MOSI/D11  ==> IR receive if present.                             
 *                      <===  Aref     -|       |- OCIB/SS/D10   ==> RX from RF433 Receiver
 *                      <===  A0/ADC0  -|       |- OCIA/D9      
 *Green Transmit mode led     A1/ADC1  -|       |- CLK0/D8       ==> 
 *Red Status led        <===  A2/ADC2  -|       |- AIN1/D7       ==> Blue Receive mode led
 *                      <===  A3/ADC3  -|       |- AIN0/D6/OC0A  ==> 
 *                      <===  A4/ADC4  -|       |- T1/D5/OC0B    ==> Relay 2 command 
 *                      <===  A5/ADC5  -|       |- T0/D4/XCK     ==> Relay 1 command
 *                      <===  A6/ADC6  -|       |- INT1/D3/OC2B  ==> IR transmit if present
 * LDR in 10k div bridge      A7/ADC7  -|       |- INT0/D2       ==> TX to RF433 Emittor
 *                      <===  5V Out   -|       |- GND          
 *                      <===  RST/C6   -|       |- RST/C6       
 *                      <===  GND      -|       |- RXD/D0       
 *                            Vin      -|       |- RXD/D1                                  
 *                                     -|__ICSP_|                                
 * 
 * 
 * 
 * Logic for nodes 
 * given a node has a number n, it's read only sensors from ESP point of view will be 200+n+{1-5}(5 read only sensors per node) 
 * its command channels (digital for the moment) will be 100+n+{1-5}
 * so to read a value of sensor m on node n, ESP will check I2c channel 200+m+n 
 * to send a command to actuator m on node n, domoticz will send a http://<esp-IP>/control?cmd=EXTGPIO,100+m+n,0 or 1 
 * as ESP don't know what type of actuator to run, and is busy with it's i2c role, it is the responsibility of this relay code to determine what type of command to actually run (can be anyhing : DIOremote, flick a relay, etc) 
 * for command the value received from the gateway via RF433 are 
 * verb = my node number in  char (0 must be avoided) 
 * value = anything from 0 to 255 (uint8_t) 
 * so to know if we have to react to this command we will use the value received, and if it is in our action range (node number to node number + 5) we 
***********************************************************************************************************************/
const char Version[] = "NaNoRelayNode30V20";
uint8_t G_RFTXPin = 2; // tx= PB3 = pin 2 
uint8_t G_RFRXPin = 10; // rx = PB1 = pin 10
uint8_t my_blueLed = 7; // receiving = pin 4 
uint8_t my_greenLed = A1; // transmitting = pin 5
uint8_t my_redLed = A2;  // error somewhere = pin 7 //not that simple on ATtiny 
uint8_t mySensorPin = 12; // my (only) sensor is = pin 3  
uint8_t pinsToTransmit[5] = {4,6,12,13,15}; // these pins will be switched to input (pinMode(xxx, INPUT) so they cannot be the same as relaypins under
int LUXPinsNumber[5]={4,6,0,0,0}; // these are the (Digital) pin numbers where we expect to find a LUX (an electric relay or similar) to switch on or off!
const uint8_t datalength = 16; // size of the whole frame (fixed size) 
const uint8_t rxlength = 16; // size of the whole frame (fixed size) 
uint8_t data[16]; // this contains the whole frame to send, including payload and control chars
unsigned long clearCMDHistoryAfterMicros = 30000000; 
char histCommandSummary[6][5] = {"0000","1111","2222","3333","4444","5555"};
const char dio[4] = "DIO";
char G_thisNodeCommands[5][4] = {"LUX","LUX","DIO","any","any"}; // type of commands that this node will access (from actuator 0 to actuator 4) i2c port-100+(0=LUX,1=LUX,2=DIO,3=DIO,4=any) 
// DIO codes for DIO nodes 0-4 (on,off,on,off...)
unsigned long CCode[10] = {0,0,1,1,1637792897,1637792913,1637792896,1637792912,5,5}; // DIOCODEs to send if configured in G_thisNodeCommands above
unsigned long G_IRCode[10] = {3772793023,3772793023,3772833823,3772829743,0,1,0,1,0,1}; // IRCODEs to send if configured in G_thisNodeCommands above
//////////////////////////////////////////////////
boolean received = false; 
//////////////////////////////////////////////////
// configuration for this node when not relaying  
unsigned long lastTimeSensorsWereSentMicros = 0; 
#ifdef DHTPRESENT
#define DHTPIN 12     // what pin we're connecting to DHT22 sensor, pseudo pin allowing to send temperature from DHT22
#define DHTPIN2 13    // Pseudo pin number to allow sending humidity from DHT22
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
#endif
unsigned long microsBeforeSendingMySensor = 60000000; // in microseconds
boolean relayMode = false; // decide if this node is an end active node (rx/tx/actuator) or also a relay node 
uint8_t password = 123; // transmission password *used to validate sender*
uint8_t myExpectedPassword = 123; // expected password for relayed frames 
#define NODE_NUMBER 30 // 0 to 254, unique for each emmitor node.. this#define line is not used by generic code, can be removed from here 
uint8_t nodeNumber = 30; // 0 to 254, unique for each emmitor node
char G_myVerb[4] = "R30" ; // that's the verb used when we are transmitting our own sensors. 
DiOremote myRemote = DiOremote(G_RFTXPin);
// the setup function runs once when you press reset or power the board
// include the generic code defined in C:\Users\po\ownCloud\perso\Everycircuit\nano\libraries\HomerCommon
#include <GenericHomerRelay.v20.h>
