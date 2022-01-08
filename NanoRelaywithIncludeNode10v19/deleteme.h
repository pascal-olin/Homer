/****************************************************************************************************************************\
 * ATTiny85 Expected wiring :  
 *                                  _______
 *                )<===  PB5/ADC0 -|1     8|- VCC
 *  RF TX data pin <===  PB3/ADC3 -|2     7|- PB2/ADC1/SCK ===> TX LED (Green) relay 2
 *     Sensor(5v)  <===  PB4/ADC2 -|3     6|- PB1/MISO     ===> RF RX data pin
 *                            GND -|4_____5|- PB0/MOSI     ===> RX Led (Blue) relay 1 
 * 
 * 
 * Logic for nodes 
 * given a node has a number n, it's read only sensors from ESP point ov view will be 200+n+{1-5}(5 read only sensors per node) 
 * its command channels (digital for the moment) will be 100+n+{1-5}
 * so to read a value of sensor m on node n, ESP will check I2c channel 200+m+n 
 * to send a command to actuator m on node n, domoticz will send a http://<esp-IP>/control?cmd=EXTGPIO,100+m+n,0 or 1 
 * as ESP don't know what type of actuator to run, and is busy with it's i2c role, it is the responsibility of this relay code to determine what type of command to actually run (can be anyhing : DIOremote, flick a relay, etc) 
 * for command the value received from the gateway via RF433 are 
 * verb = my node number in  char (0 must be avoided) 
 * value = anything from 0 to 255 (uint8_t) 
 * so to know if we have to react to this command we will use the value received, and if it is in our action range (node number to node number + 5) we 
***********************************************************************************************************************/
// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(9600); 
    // setup transmit pin and speed)
    man.setupTransmit(G_RFTXPin, MAN_1200);
    // rx mode until anything has been received 
    man.setupReceive(G_RFRXPin, MAN_1200);
    man.beginReceiveArray(MY_RX_SIZE, rx_tx_buffer);
    // initialize digital pin my_blueLed as an output.
    pinMode(my_greenLed, OUTPUT);
    pinMode(my_blueLed, OUTPUT);
    pinMode(my_redLed, OUTPUT);
    digitalWrite(my_greenLed, HIGH); // not sending anymore 
    digitalWrite(my_blueLed, HIGH); // we are NOT in receive mode
    digitalWrite(my_redLed, HIGH); // we are NOT in receive mode
    // DDRB |= (1 << my_blueLed);  
    // DDRB |= (1 << my_redLed);
    // DDRB |= (1 << my_greenLed);
  }
// the loop function runs over and over again forever
void loop() {
  // empty history of commands passed more than x seconds ago 
  //Serialprintln("History table : ----------"); 
  static int histCommandNDX = 0; 
  if ((histCommandNDX < 0) || (histCommandNDX > 6) ) {exit(0);} 
  for (int m=0;m<6;m++){
    if (histCommandMicros[m] + clearCMDHistoryAfterMicros < micros()) { // clear value after clearCMDHistoryAfterMicros seconds 
      histCommandMicros[m]=0;
      strncpy(histCommandSummary[m],"---=",4); // command + 1 byte of value  
      histCommandSummary[m][4] = '\0'; // terminate cstring
    }
  }
  // by default, we receive until we get something to relay 
  received = false ; 
  digitalWrite(my_blueLed, HIGH); // switch to receive mode
  receiveManchester();
  // now something is received, we want to decode it, check it and then reassemble it using our new verb (unless it is a command, in which case we act on it) 
  if (received) {
    // am I asked to do anything ? 
      char tmp[4] = {'1','2','3','\0'}; 
      tmp[0] = tx_buffer[3];
      tmp[1] = tx_buffer[4];
      tmp[2] = tx_buffer[5];      
      tmp[3] = '\0';
      uint8_t DIOVal = uint8_t(tx_buffer[9]) ;   //force casting to uiint8_t
      Serialprintln("histcommandNDX is : "+ String(histCommandNDX) ); //debug to delete 
      // if (strcmp(tmp,dio) == 0) // if this is a DIO command. is this a command /   
      // is this for me ???
      int targetNode = atoi(tmp); 
      int _minNodeNumber = int(nodeNumber+100) ; 
      int _maxNodeNumber = int(nodeNumber+104); 
            Serialprintln("in received = true, Target: "+ String(targetNode) + "mM:" +String(_minNodeNumber)+String(_maxNodeNumber) ); 
      if ((targetNode >= _minNodeNumber) && (targetNode <= _maxNodeNumber )) // scope for commands on this node is NODE_NUMBER to NODE_NUMBER+5 
      // if (targetNode >= 100 && targetNode <= 105) // scope for commands on this node is NODE_NUMBER to NODE_NUMBER+5 
      { 
        Serialprintln("Command received : "+String((char*)(tmp)) + " is for me node #: " + nodeNumber) ; 
        char thisNodeCommands[5][4] = {"LUX","LUX","DIO","DIO","any"}; // i2c port-100+(0=DIO,1=DIO,2=DIO,3=DIO,4=any) 
        // if (strcmp(tmp,"CMD") == 0) // is this a command /  
        // start with display last commands array. 
        for (int m=0;m<6;m++){
            Serialprintln("History of command: "+ String((char*)(histCommandSummary[m]))+" at Micros " + histCommandMicros[m]); 
            }
        // then find if this command has already been processed in the past few seconds 
        boolean runthiscommand = true;    
        char thisCommandSummary[5] = {'e','m','p','t','\0'} ; 
        strncpy(thisCommandSummary,tmp,3);  
        thisCommandSummary[3]=char(tx_buffer[9]);
        //strcat(thisCommandSummary,tx_buffer[9]);
        thisCommandSummary[4] = '\0'; // terminate string
        unsigned long thismicro = micros();
        // to delete Serialprintln("this command: "+ String((char*)(thisCommandSummary)));
        for (int m=0;m<6;m++){
          // to delete Serialprintln("this command: "+ String((char*)(thisCommandSummary)) +String((char*)(histCommandSummary[m])));
          if (strcmp(histCommandSummary[m],thisCommandSummary) == 0) // if we already have processed this command (meaning we received it), don't run it, its probably a duplicate send (history table is cleared every clearCMDHistoryAfterMicros microseconds) 
            {
              Serialprintln("Bingo command found in history, not running it");  // debug 
              runthiscommand = false; 
            }
          }
//// just display last commands array. //  for (int m=0;m<6;m++){
//    Serialprintln("Hist of command: "+ String(histCommandSummary[m])); 
//  }
        // moved todelete unsigned long CCode[4] = {1637792897,1637792913,1637792896,1637792912};
        if (runthiscommand==true) {   // send the command if it was not in the table  
            digitalWrite(my_blueLed, LOW); // stopping receive   
            digitalWrite(my_redLed, HIGH);
            // to restore man.stopReceive(); // stop receiving (so we can transmit) 
            Serialprintln(" Sending command value "+String(DIOVal)+" targetnode "+ String(targetNode) +" histcommand index = "+ String(histCommandNDX) ); 
// Code for Command management : 
// if we are here it means that the command is for this current node. 
// look at thisNodeCommands and check what the target type of device is, then act on it. 
// the _minNodeNumber+1 below is because we use arrays and they have index start at 0 !             
            if (strcmp(thisNodeCommands[targetNode - (_minNodeNumber+1)],"DIO")==0)
            {
                  unsigned long thisCode = findDIOCode(targetNode - (_minNodeNumber),DIOVal) ; 
                  Serialprintln("Sending DIO code: "+String(thisCode)); 
                  myRemote.send(thisCode);
              }
            if (strcmp(thisNodeCommands[targetNode - (_minNodeNumber+1)],"LUX")==0)
            {
              int relayPinsNumber[5]={0,2,9,9,9};
                  unsigned long thisCode = findDIOCode(targetNode,DIOVal) ; 
                  int thisRelay = relayPinsNumber[targetNode-(_minNodeNumber)];
                  Serialprintln("About to switch pin : "+String(thisRelay) + " to value " + DIOVal);
//                  myRemote.send(thisCode);
              }

              histCommandNDX = (histCommandNDX >= 5) ? 0 : histCommandNDX+1 ;  
              //strncpy(histCommandSummary[histCommandIndex],thisCommandSummary,5); 
              Serialprintln(" storing this command: "+ String((char*)(thisCommandSummary)) + " at index "+String(histCommandNDX));
              strncpy(histCommandSummary[histCommandNDX],thisCommandSummary,5); 
              histCommandSummary[histCommandNDX][4] = '\0'; 
              histCommandMicros[histCommandNDX]=micros();
              digitalWrite(my_redLed, LOW);
              digitalWrite(my_blueLed, HIGH); // switch to receive mode
              man.beginReceiveArray(MY_RX_SIZE, rx_tx_buffer); // immediately start to listen for another message 
              } 
        else { 
              Serialprintln(" Not sending command to DIO as present in recent history table Value requested "+String(DIOVal) ); 
             }
        }
      else if (relayMode){  // this packet is not a command, just relay it if we are is a realy node
        Serialprintln(" Relaying "); 
        // we have received something, assuming the the receicemanchester function properly computes the CRC32, we have a good message
        // we will simply now replace the verb (to make sure receptor know this is a relay) and the CRC and transmit again to whoever wants to hear it 
        //  replacement is done on the rx_tx_buffer itself to save ram 
        tx_buffer[3] = 'R'; 
        tx_buffer[4] = 'E'; 
        tx_buffer[5] = 'L';
        uint8_t numBytes =  11; //buffer - CRC 
        uint32_t checksum = CRC32::calculate(tx_buffer, numBytes);
        //slot CRC at the end of the packet 
        tx_buffer[11] = (byte) ((checksum & 0xFF000000) >> 24 );
        tx_buffer[12] = (byte) ((checksum & 0x00FF0000) >> 16 );
        tx_buffer[13] = (byte) ((checksum & 0x0000FF00) >> 8  );
        tx_buffer[14] = (byte) ((checksum & 0X000000FF)       );
        tx_buffer[15] = '\0'; // end of cstring // position=15
        // end of preparation , buffer is ready to be sent ! 
        digitalWrite(my_blueLed, LOW); // we are not in receive mode 
        man.stopReceive(); // stop receiving 
        ///// my_delay(500); // deleteme
        man.transmitArray(datalength,tx_buffer); // transmit our buffer 
        //my_delay(15); // deleteme
        digitalWrite(my_blueLed, HIGH); // switch to receive mode
        man.beginReceiveArray(MY_RX_SIZE, rx_tx_buffer); // immediately start to listen for another message 
        }    
    }
    else { // this is processed only if we have not received anything (basically for a relay node, it's more important to relay or action than transmit our own sensors. // might need adjustement on this logic. 
    if (micros() - waitUntil > microsBeforeSendingMySensor) { 
      Serialprintln("nothing to relay, no command --> ready to send our own sensor value" ); 
      // delete me Serialprint(" NDX-B: "+ String(histCommandNDX) );
      // now transmit our own sensors 
      waitUntil = micros();
      // delete me Serialprint(" NDX-C: "+ String(histCommandNDX) );
      unsigned long myValue = analogRead(mySensorPin); 
      //digitalWrite(my_greenLed, HIGH); // about to send 
      //my_delay(500); // deleteme
      //preparePayLoad(myVerb, myValue);
      // stop transmitting so we dont catch our own frames to relay .
       digitalWrite(my_blueLed, LOW); // stopping receive   
       man.stopReceive(); // stop receiving (so we can transmit) 
       // delete me Serialprint(" NDX-D: "+ String(histCommandNDX) );
       digitalWrite(my_greenLed, HIGH); // Switch to sending mode 
       prepareAndSendPayLoad(MY_RX_SIZE,myVerb, myValue,password,NODE_NUMBER);
       // delete me Serialprint(" NDX-E: "+ String(histCommandNDX) );
       //my_delay(500); // deleteme
       digitalWrite(my_greenLed, LOW); // not sending anymore 
       //my_delay(15); // deleteme
       digitalWrite(my_blueLed, HIGH); // switch to receive mode
       // delete me Serialprint(" NDX-F: "+ String(histCommandNDX) );
       man.beginReceiveArray(MY_RX_SIZE, rx_tx_buffer); // immediately start to listen for another message 
       // delete me Serialprint(" NDX-G: "+ String(histCommandNDX) );
      } 
    }

         //digitalWrite(my_blueLed, LOW); // we are not in receive mode 
}
unsigned long findDIOCode(int _node, int _value){
  Serialprintln("finding code for node index: "+ String(_node) + " value: "+ String(_value)) ; 
    unsigned long CCode[10] = {0,0,1,1,2,2,1637792897,1637792913,1637792896,1637792912};
    unsigned long _retCode=0;
    
    //int _ndx =((2*(_node - 113)) + _value); 
    int _ndx =((2*(_node)) + _value); 
    Serialprintln("DIO index = "+ String(_ndx)); 
    _retCode = CCode[_ndx]; 
    return(_retCode); 
}
/// prepare and send the actual datagram 
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
  man.transmitArray(myLength, myData);
 }

void receiveManchester() {
 if (man.receiveComplete())  // according to manchester Lib, we have a consistent frame.  
 {
    // Serialprintln(" Receive Manchester inner if " ); 
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
    // compute expected CRC from the received frame actual data
    size_t numBytes = sizeof(rx_tx_buffer) - 5; // rx_tx_buffer - CRC32 information - final '\0' 
    numBytes = 11 ; 
    long myExpectedCRC = CRC32::calculate(rx_tx_buffer, numBytes);
        //Serialprintln("1 histcommandindex is : "+ String(histCommandIndex) );
// not used in relay    uint8_t receivedSize = 0;
    char  my_Verb[4] = "" ; 
// unused    uint8_t myManReceivedSize = rx_tx_buffer[0];  // size in first byte
    uint8_t mySender = rx_tx_buffer[1]; // who sent it ? 
    //uint8_t myfirstPassword = rx_tx_buffer[2];
    //int m=5 ;
    //int o=10 ; int v=3;   
    // not used in relay       boolean foundslash=false; 
    my_Verb[0] = rx_tx_buffer[3];
    my_Verb[1] = rx_tx_buffer[4];
    my_Verb[2] = rx_tx_buffer[5];
    my_Verb[3] = '\0';
    unsigned long my_value = 0;
    my_value = ((unsigned long)(rx_tx_buffer[6]) << 24)
                + ((unsigned long)(rx_tx_buffer[7]) << 16)
                + ((unsigned long)(rx_tx_buffer[8]) << 8)
                + ((unsigned long)(rx_tx_buffer[9]));  
    // not testing passwords, we simply don't have the space in ram 
    //uint8_t mySecondPassword = rx_tx_buffer[10] ; // one byte after the delimiter 
    // restore CRC from message 
    //if ((mySecondPassword == myfirstPassword) && (mySecondPassword == myExpectedPassword )){
           // Serialprintln("3 histcommandindex is : "+ String(histCommandIndex) );
    Serialprintln("received:  Verb " + String(my_Verb) + " received from sender: " + String(int(mySender)) + " value : " + String(my_value) ); 
    uint32_t myReadCRC = 0;
    myReadCRC += (uint32_t)rx_tx_buffer[11] << 24; 
    myReadCRC += (uint32_t)rx_tx_buffer[12] << 16; 
    myReadCRC += (uint32_t)rx_tx_buffer[13] << 8; 
    myReadCRC += (uint32_t)rx_tx_buffer[14];
    /// delete wroint long myReadCRC = (unsigned long)(rx_tx_buffer[14] << 24) | (rx_tx_buffer[13] << 16) | (rx_tx_buffer[12] << 8) | rx_tx_buffer[11];
    if (myReadCRC == myExpectedCRC) { 
      // fill out node last value table (using mySender number as index) 
      //rftxNodeLastValue[rx_tx_buffer[1]] = my_value ;
      for (int vv=0; vv<MY_RX_SIZE-1;vv++){ // copy input to output buffer (as input buffer will be destroyed as soon as we start reading again) ? 
            tx_buffer[vv] = rx_tx_buffer[vv]; 
            }
      received = true; 
              //Serialprintln("5 histcommandindex is : "+ String(histCommandIndex) );
        } 
        else {
          received = false;
          Serialprintln(" CRC error : read : "+String(myReadCRC)+ " Expected : "+String(myExpectedCRC) ); 
          //digitalWrite(my_redLed, HIGH); // just show something is wrong
          // PORTB |= (1 << my_redLed);
                 // Serialprintln("6 histcommandindex is : "+ String(histCommandIndex) );
          }
    //}
    //else {
          // not on tiny Serialprint(" Passwords dont match : "+String(myfirstPassword)+ " Expected : "+String(mySecondPassword) ); 
          //digitalWrite(my_redLed, HIGH); // just show something is wrong  
    //    }
    digitalWrite(my_blueLed, HIGH); // switch to receive mode
           // Serialprintln("7 histcommandindex is : "+ String(histCommandIndex) );
    man.beginReceiveArray(MY_RX_SIZE, rx_tx_buffer);
           // Serialprintln("8 histcommandindex is : "+ String(histCommandIndex) );
  }
}
void condSerialPrint(String _myline){
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define Serialprintln nop 
#else  
#define SERIALABLE 1
#endif  
}
//void my_delay(int n ){ // in milliseconds 
//  for (int nn=0;nn<n;nn++){ 
//    delayMicroseconds(1000);// maximum delay is 65535 
// }
//}
//void heartbeat() {
//  static unsigned long last_time = 0;
//  unsigned long now = millis();
//  if ((now - last_time) < 80)
//    return;
//  last_time = now;
//  if (hbval > 242) hbdelta = -hbdelta;
//  if (hbval < 8) hbdelta = -hbdelta;
//  hbval += hbdelta;
//  analogWrite(my_blueLed, hbval);
//}
