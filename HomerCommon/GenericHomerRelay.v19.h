/****************************************************************************************************************************\
 * ATTiny85 Expected wiring :  
 *                                  _______
 *                )<===  PB5/ADC0 -|1     8|- VCC
 *  RF TX data pin <===  PB3/ADC3 -|2     7|- PB2/ADC1/SCK ===> TX LED (Green) relay 2
 *     Sensor(5v)  <===  PB4/ADC2 -|3     6|- PB1/MISO     ===> RF RX data pin
 *                            GND -|4_____5|- PB0/MOSI     ===> RX Led (Blue) relay 1 
 * 
 * Nano Expected wiring :  
 *                                    USB 
 *                                  __|  |_
 *                 <===  SCK/D13  -|  |__| |- MISO/D12
 *                 <===  3.3V Out -|       |- OC2/MOSI/D11                               
 *     	           <===  Aref     -|       |- OCIB/SS/D10                       
 *                 <===  A0/ADC0  -|       |- OCIA/D9      
 *                 <===  A1/ADC1  -|       |- CLK0/D8      
 *                 <===  A2/ADC2  -|       |- AIN1/D7      
 *                 <===  A3/ADC3  -|       |- AIN0/D6/OC0A 
 *                 <===  A4/ADC4  -|       |- T1/D5/OC0B   
 *                 <===  A5/ADC5  -|       |- T0/D4/XCK    
 *                 <===  A6/ADC6  -|       |- INT1/D3/OC2B 
 *                 <===  A7/ADC7  -|       |- INT0/D2      
 *                 <===  5V Out   -|       |- GND          
 *                 <===  RST/C6   -|       |- RST/C6       
 *                 <===  GND      -|       |- RXD/D0       
 *                       Vin      -|  ICSP |- RXD/D1                                  
 *                                -|__ICSP_|                                
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
 * so to know if we have to react to this command we will use the value received, and if it is in our action range (node number to node number + 5) we will execute the relevant subfunction
 ** 
 * implemented functions : (where NDX is the relative node number calculated from #define NODE_NUMBER xx and uint8_t nodeNumber = xx) 
 * IRD --> void ProcessRelayIRD sends the received payload value to the "relayPinsNumber"[ndx] using IRsend (see IRemote MODIFIED library (PWM on pin 9 instead of 3) 
 * LUX --> void ProcessRelayCMD switches ( 0 or 1 ) pin "relayPinsNumber"[ndx]
 * DIO --> void ProcessDIOCMD sends DIO code CCCode "[ndx] using the DiOremote Library via the RF433 Mhz emittor to the 
***********************************************************************************************************************/
// Common globals (non config) 
unsigned long histCommandMicros[6]= {0,0,0,0,0,0} ;
#define BUFFER_SIZE 16 // fix buffer size
#define MY_RX_SIZE 16 // fix buffer size
//uint8_t rx_tx_buffer[rxlength];
//uint8_t dummy1[20];
//uint8_t tx_buffer[rxlength];
uint8_t tx_buffer[255];
uint8_t rx_tx_buffer[255];
//uint8_t dummy2[20];
uint8_t histCommandNDX = 0; 
boolean receiving = false; 
const char fakeBuff[] = {0x10,0x05,0x7b,0x31,0x33,0x31,0x00,0x00,0x00,0x01,0x7b,0x90,0xa8,0x42,0xba,0x00};
// memory debug globals 
/* extern char _end;
extern "C" char *sbrk(int i);
char *ramstart=(char *)0x20070000;
char *ramend=(char *)0x20088000;
 */// Functions prototypes 
unsigned long findDIOCode(int _node, int _value) ;
void prepareAndSendPayLoad(uint8_t myLength, char * G_myVerb, unsigned long myValue, uint8_t myPassword, uint8_t myNodeNumber ); 
void receiveManchester();
void SendMySensors();
void RelayPacket();
void ProcessDIOCMD(int _l_targetNode, uint8_t _l_commandShortValue);
void ProcessRelayCMD(int _l_targetNode, uint8_t _l_commandShortValue);
void ProcessRelayIRD(int _l_targetNode, uint32_t _l_commandFullValue);
unsigned long findIRDCode(int _node, int _value);

void ProcessSerialInput();
//void condSerialPrint(String _myline);
//void my_delay(int n ); 
int G_minNodeNumber = 0;
int G_maxNodeNumber = 0;
// Setup everything
void setup() {
    Serial.begin(9600); 
	#ifdef DHTPRESENT
	dht.begin();
	#endif
    // setup transmit pin and speed)
    man.setupTransmit(G_RFTXPin, MAN_1200);
    // rx mode until anything has been received 
    man.setupReceive(G_RFRXPin, MAN_1200);
	man.beginReceiveArray(rxlength, rx_tx_buffer);
    // initialize digital pin my_blueLed as an output.
    pinMode(my_greenLed, OUTPUT);
    pinMode(my_blueLed, OUTPUT);
    pinMode(my_redLed, OUTPUT);
    digitalWrite(my_greenLed, HIGH); // not sending anymore 
    digitalWrite(my_blueLed, HIGH); // we are NOT in receive mode
    digitalWrite(my_redLed, HIGH); // we are NOT in sendcommand mode
    G_minNodeNumber = int(nodeNumber+100) ; // start id of first target
    G_maxNodeNumber = int(nodeNumber+104);  // end id of last target
    // DDRB |= (1 << my_blueLed);  
    // DDRB |= (1 << my_redLed);
    // DDRB |= (1 << my_greenLed);
  }
  void displayBuffers(){ 
	uint8_t *prxtx = rx_tx_buffer;
	char _tmp1[16]; //[16];
	for (int nn=0;nn<16;nn++) {
		sprintf(_tmp1,"%02x",(unsigned char)prxtx[nn]);
		Serial.print(_tmp1); 
		Serial.print("."); 
	}
	Serial.println(" ");
	Serial.println("0 |1 |2 |3 |4 |5 |6 |7 |8 |9 |A |B |C |D |E |F |");
	char *prxtx2 = tx_buffer;
	// _tmp1[5];
	for (int nn=0;nn<16;nn++) {
		//sprintf(_tmp1,"%02x",*prxtx ? *prxtx : ' ');
		sprintf(_tmp1,"%02x",(unsigned char)prxtx2[nn]);
		//prxtx2++; 
		Serial.print(_tmp1);
		Serial.print("."); 
	}
	Serial.println(" ");
	Serial.println("0 |1 |2 |3 |4 |5 |6 |7 |8 |9 |A |B |C |D |E |F |");
	#ifdef DEBUG
	Serial.println("");
	SerialprintF("Receiving flag : "); 
	Serialprintln(receiving); 
	#endif
} 

// the loop function runs over and over again forever
void trapCorruption(char* _txt,int _len){
	char _mytxt[80] = "";
	memcpy(_mytxt,_txt,_len);
	if ((G_minNodeNumber < 0 || G_minNodeNumber > 250)) {
			digitalWrite(my_redLed, HIGH); 
			Serial.print(" ****** Corruption at: " );
			Serial.print(_mytxt);
			Serial.print(" HistComandNDX: " );
			Serial.println(histCommandNDX); 
			Serial.print(" My range is : " );
			Serial.print(G_minNodeNumber); 
			Serial.print(" to "); 
			Serial.print(G_maxNodeNumber);
			Serial.print(" my node number "); 
			Serial.println(nodeNumber);
				displayBuffers();
			while (1) { // stop doing anything 
				} 
	}
}
/* 
This function displays debug information if operator required it 
for the moment "l" will List suspected variables status in mode debug and will then display the current and last input buffers.
*/ 
void ProcessSerialInput() {
	char serialInput = '0'; 
	if (Serial.available() > 0) { // if anything on serial 
		serialInput = Serial.read();
		if (serialInput == 'l') { 
			Serial.println(Version);
			Serial.print(F(" HistComandNDX: " ));
			Serial.println(histCommandNDX); 
			Serial.print(F(" My range is : " ));
			Serial.print(G_minNodeNumber); 
			Serial.print(F(" to ")); 
			Serial.println(G_maxNodeNumber); 
			Serial.println(F(" "));
			for (int m=0;m<6;m++)
			  {
				Serial.print(F("History of command: "));
				Serial.print(histCommandSummary[m]); 
				Serial.print(F(" at Micros "));
				Serial.println(histCommandMicros[m]);
				}
				displayBuffers();
			// now force transmit our own sensors 
			//lastTimeSensorsWereSentMicros = micros();				
			//SendMySensors(); 
			}
		}
} 
void loop() {
	#ifdef DEBUG
	char msg [] = "loop   " ;
 	trapCorruption(msg,strlen(msg));
	#endif
	ProcessSerialInput();
  // empty history of commands passed more than x seconds ago 
  	unsigned long _ltime = micros(); //moved above loop to try to spot memory corruption 
	for (int _m=0;_m<6;_m++){
		if (((_ltime - histCommandMicros[_m]) >= clearCMDHistoryAfterMicros) && (histCommandSummary[_m][0] != '-')){ 
		    #ifdef DEBUG
			SerialprintF(" emptying history of command for position : ");
			Serialprintln(_m); 
			#endif
			// if (histCommandMicros[m] + clearCMDHistoryAfterMicros < micros()) { // clear value after clearCMDHistoryAfterMicros seconds 
			//hunting memory corruption histCommandMicros[_m]=0; 
			memcpy(histCommandSummary[_m],"----",5); // command + 1 byte of value  
    }
  }
	// by default, we receive until we get something to relay or a command to process
	received = false ; 
	digitalWrite(my_blueLed, HIGH); // switch to receive mode
	#ifdef DEBUG
	char msg1 [] = "just before manchester " ;
	trapCorruption(msg1,strlen(msg1));
	#endif
	//if (man.receiveComplete()) { // only if buffer is populated (tracking corruption ) 
	if (1) { // only if buffer is populated (tracking corruption ) 
		receiveManchester();
	} 
	#ifdef DEBUG
  	char msg2 [] = "post manchester " ;
	trapCorruption(msg2,strlen(msg2));
	#endif
	digitalWrite(my_redLed, LOW); // we are NOT in sendcommand mode
	// now something is received, we want to decode it, check it and then reassemble it using our new verb (unless it is a command, in which case we act on it) 
	if (received) { // receicemanchester has set up this boolean to true if the packet received looks ok. 
		#ifdef DEBUG
		SerialprintlnF(" Received true");
		char msg3 [] = " at if received is true" ;
		trapCorruption(msg,strlen(msg3));
		#endif
		// am I asked to do anything ? 
		char tmp[4] = {'1','2','3','\0'}; 
		tmp[0] = tx_buffer[3];
		tmp[1] = tx_buffer[4];
		tmp[2] = tx_buffer[5];      
		tmp[3] = '\0';
		uint8_t commandShortValue = uint8_t(tx_buffer[9]) ;   //force casting to uint8_t

	    uint32_t commandFullValue = 0;
/* 		commandFullValue = ((uint32_t)(tx_buffer[6]) << 24)
                + ((uint32_t)(tx_buffer[7]) << 16)
                + ((uint32_t)(tx_buffer[8]) << 8)
                + ((uint32_t)(tx_buffer[9]));  
 */		commandFullValue += (uint32_t)tx_buffer[6] << 24;
		commandFullValue += (uint32_t)tx_buffer[7] << 16;
		commandFullValue += (uint32_t)tx_buffer[8] << 8;
		commandFullValue += (uint32_t)tx_buffer[9];  




		// is this for me ???
		int targetNode = atoi(tmp); 
		if (targetNode != 0){ // target node (actuator) is not null and is numeric. proceed. 
			#ifdef DEBUG
			// SerialprintlnF("Invalid target node in packet, abort this pass ");
			char msg4 [] = " after atoi " ;
			trapCorruption(msg4,strlen(msg4));
			#endif
			  if ((targetNode >= G_minNodeNumber) && (targetNode <= G_maxNodeNumber )) // scope for commands on this node is NODE_NUMBER to NODE_NUMBER+5 
				{ 
				#ifdef DEBUG
				SerialprintF("in received = true, Target: ");
				Serialprint(targetNode); 
				SerialprintF(" min/MAX: "); 
				Serialprint(G_minNodeNumber);
				SerialprintF("/"); 
				Serialprintln(G_maxNodeNumber);
				#endif
				#ifdef DEBUG
				SerialprintF("Command received : "); 
				Serialprint(tmp);
				SerialprintF(" is for me node # ") ; 
				Serialprint(nodeNumber) ; 
				SerialprintF(" value ") ; 
				Serialprintln(commandShortValue); 
				SerialprintF(" Fullvalue ") ; 
				Serialprintln(commandFullValue); 

				#endif
				#ifdef DEBUG
				// start with display last commands array. 
				for (int m=0;m<6;m++){
					SerialprintF("History of command: ") ; 
					Serialprint(histCommandSummary[m]); 
					SerialprintF(" at Micros "); 
					Serialprintln(histCommandMicros[m]); 

				}
				#endif					
				// then find if this command has already been processed in the past few seconds 
				#ifdef DEBUG 
				SerialprintF("histcommandNDX is : "); 
				Serialprintln(histCommandNDX);
				#endif	
				boolean runthiscommand = true; // a priori, we will run this command.    
				// build a command summary C-string to compare with the history table. 
				// if then found in history, do not run the command (the gateway will issue the same command several times
				// to compensate the risk of lost/malformed 433mhz packets
				char thisCommandSummary[5] = {'e','m','p','t','\0'} ; // just to initialize it
				memcpy(thisCommandSummary,tmp,4);  // used to compare it with history table later in code
				thisCommandSummary[3]=tx_buffer[9]; // add the value
				thisCommandSummary[4] = '\0'; // terminate string
				// now look for command in history table
				for (int m=0;m<6;m++){
				// if we already have processed this command (meaning we received it), don't run it, its probably a duplicate send (history table is cleared every clearCMDHistoryAfterMicros microseconds) 
				if (strcmp(histCommandSummary[m],thisCommandSummary) == 0)
					{
					#ifdef DEBUG	
					#endif  
					// command found in history, will not be run 
					runthiscommand = false; 
					}
				  }
				if (runthiscommand==true) {   // run the command as it was not in the table  
					digitalWrite(my_redLed, HIGH); // Command sending mode. 
					#ifdef DEBUG
					SerialprintF(" getting commmand type from G_thisNodeCommands at index ");
					Serialprint(targetNode - (G_minNodeNumber));
					SerialprintF(" set to value : "); 
					Serialprint(commandShortValue); 
					SerialprintF(" targetnode "); 
					Serialprint(targetNode);
					SerialprintF(" histcommand index = "); 
					Serialprintln(histCommandNDX); 
					#endif
					// Code for Command management : 
					// if we are here it means that the command is for this current node. 
					// look at G_thisNodeCommands and check what the target type of device is, then act on it. 
					// the G_minNodeNumber below is because we use arrays and they have index start at 0 !             
					if (strcmp(G_thisNodeCommands[targetNode - (G_minNodeNumber)],"DIO")==0)
					{
						ProcessDIOCMD(targetNode,commandShortValue);
					  }
					if (strcmp(G_thisNodeCommands[targetNode - (G_minNodeNumber)],"LUX")==0)
					{
						ProcessRelayCMD(targetNode,commandShortValue);
					  }
					if (strcmp(G_thisNodeCommands[targetNode - (G_minNodeNumber)],"IRD")==0)
					{
						ProcessRelayIRD(targetNode,commandShortValue);
					  }

					  histCommandNDX = (histCommandNDX >= 5) ? 0 : histCommandNDX+1 ;  
					  #ifdef DEBUG
					  SerialprintF(" storing this command: "); 
					  Serialprint(thisCommandSummary); 
					  SerialprintF(" at index "); 
					  Serialprintln(histCommandNDX); 
					  #endif 
					  memcpy(histCommandSummary[histCommandNDX],thisCommandSummary,5); // remember what command was issued 
					  // histCommandSummary[histCommandNDX][4] = '\0'; 
					  histCommandMicros[histCommandNDX]=micros(); // remember when this command was issued
					  } 
				else { // command has not been run. just saying it . 
					#ifdef DEBUG	
					  SerialprintF(" Not sending command to target actuator ") 
					  Serialprintln(targetNode);
					  SerialprintF(" as it is present in recent history table ... for info Value requested was "); 
					  Serialprintln(commandShortValue); 
					  #endif
					 }
				}
			  else if (relayMode){  // this packet is not a command, just relay it if we are is a relay node
				RelayPacket(); 
				}    
			}
		} 
    else { // this is processed only if we have not received anything (basically for a relay node, it's more important to relay or action than transmit our own sensors. // might need adjustement on this logic. 
		SendMySensors(); 
	}
}
/* 
This function will be called if an IRD command has been received 
*/ 
void ProcessRelayIRD(int _l_targetNode, uint32_t _l_commandShortValue){
				  digitalWrite(my_redLed, HIGH);	
			      // which pin corresponds to this target node ? relayPinsNumber names the actuator pins
				  unsigned long thisCode = findIRDCode(_l_targetNode - (G_minNodeNumber),_l_commandShortValue) ; 
				  #ifdef DEBUG
				  SerialprintF("Sending IRD code: "); 
				  Serialprint(thisCode); 
				  SerialprintF(" To node/actuator: "); 
				  Serialprintln(_l_targetNode); 
				  SerialprintF(" to value ");
				  Serialprintln(_l_commandShortValue);
				  #endif
				  for (int i = 0; i < 3; i++) {
					Serial.print("S-");
					irsend.sendSAMSUNG(thisCode, 32);
					delay(40);
					}

				  //pinMode(thisRelay, OUTPUT);
				  //digitalWrite(thisRelay,_l_commandShortValue);
				  digitalWrite(my_redLed, LOW); // Command sending mode off. 

//                  myRemote.send(thisCode);
}
/* 
This function will be called if a LUX command has been received 
*/ 
void ProcessRelayCMD(int _l_targetNode, uint8_t _l_commandShortValue){
				  digitalWrite(my_redLed, HIGH);
				  // which pin corresponds to this target node ? relayPinsNumber names the actuator pins
                  int thisRelay = relayPinsNumber[_l_targetNode-(G_minNodeNumber)];
				  #ifdef DEBUG
				  SerialprintF("About to switch pin : "); 
				  Serialprint(thisRelay); 
                  SerialprintF(" To node/actuator: "); 
				  Serialprint(_l_targetNode); 
				  SerialprintF(" to value ");
				  Serialprintln(_l_commandShortValue);
				  #endif
				  pinMode(thisRelay, OUTPUT);
				  digitalWrite(thisRelay,_l_commandShortValue);
				  digitalWrite(my_redLed, LOW); // Command sending mode off. 

//                  myRemote.send(thisCode);
}
void ProcessDIOCMD(int _l_targetNode, uint8_t _l_commandShortValue) { 
				  digitalWrite(my_blueLed, LOW);  //stop receiving
				  digitalWrite(my_redLed, HIGH);
				  man.stopReceive(); // stop receiving 
				  receiving = false; 	
				  unsigned long thisCode = findDIOCode(_l_targetNode - (G_minNodeNumber),_l_commandShortValue) ; 
				  #ifdef DEBUG
				  SerialprintF("Sending DIO code: "); 
				  Serialprint(thisCode); 
				  SerialprintF(" To node/actuator: "); 
				  Serialprintln(_l_targetNode); 
				  #endif
				  digitalWrite(my_greenLed, HIGH); // Switch to sending mode 
				  myRemote.send(thisCode);
				  digitalWrite(my_greenLed, LOW); // stop sending mode 
				  // not here digitalWrite(my_blueLed, HIGH); // switch to receive mode
				  digitalWrite(my_redLed, LOW); // Command sending mode off. 
				  //restore man.beginReceiveArray(rxlength, rx_tx_buffer); // immediately start to listen for another message 
					if (!receiving) { 
						//man.beginReceiveArray(rxlength, rx_tx_buffer);
						//receiving = true; 
					}
				  
}
/* 

*/ 
void RelayPacket(){ 
	  #ifdef DEBUG
        Serialprintln(" Relaying "); 
		#endif
        // we have received something, assuming the the receicemanchester function properly computes the CRC32, we have a good message
        // we will simply now replace the verb (to make sure receptor know this is a relay) and the CRC and transmit again to whoever wants to hear it 
        //  replacement is done on the rx_tx_buffer itself to save ram 
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
		digitalWrite(my_greenLed, HIGH); // Switch to sending mode 
        man.stopReceive(); // stop receiving 
		receiving = false; 
        man.transmitArray(datalength,tx_buffer); // transmit our buffer 
		digitalWrite(my_greenLed, LOW); // Switch to sending mode 
        digitalWrite(my_blueLed, HIGH); // switch to receive mode
        //restore man.beginReceiveArray(rxlength, rx_tx_buffer); // immediately start to listen for another message 
		if (!receiving) { 
			//man.beginReceiveArray(rxlength, rx_tx_buffer);
			//receiving = true; 
	}
} 
void SendMySensors() { 
	//SerialprintlnF(" Nothing received, maybe sending sensors ");
	#ifdef DEBUG	
 	char msg [] = "before sending sensors " ;
 	trapCorruption(msg,strlen(msg));
	#endif
	// return; // trying to find where the bloody memory corruption is 
	unsigned long ltime = micros();
	if (ltime - lastTimeSensorsWereSentMicros > microsBeforeSendingMySensor) 
		{ 
		//SerialprintlnF(" time to really send sensors  ");
			char msg [] = "Really sending sensors " ;
			trapCorruption(msg,strlen(msg));

		// now transmit our own sensors 
		lastTimeSensorsWereSentMicros = micros();
		//unsigned long myValue = 0;
		int myValue = 0; 
		// stop transmitting so we dont catch our own frames to relay .
		digitalWrite(my_blueLed, LOW); // stopping receive   
		man.stopReceive(); // stop receiving (so we can transmit) 
		receiving = false; 
		digitalWrite(my_greenLed, HIGH); // Switch to sending mode 
		// pinsToTransmit names the reported pins
		for (uint8_t _nn=0;_nn<5;_nn++)
		{
			// int _mm = nodeNumber+_nn;
			uint8_t _mm = nodeNumber+_nn;
			if (false) { 
			//  this is just here to allow the else ifs in the conditional compilation below
			} 
			#ifdef DHTPRESENT
			// DHT pin, only one allowed for the moment (otherwise I will need to make a loop)  
			else if (pinsToTransmit[_nn] == DHTPIN)  { 
				myValue = int(dht.readTemperature());
							SerialprintF(" temp with value ");
			Serialprintln(myValue);
			}
			else if (pinsToTransmit[_nn] == DHTPIN2) {
				myValue = int(dht.readHumidity());
											SerialprintF(" humi with value ");
			Serialprintln(myValue);
			}
			#endif
			// Analog pins (A0 is 14 --> A7 is 21) 
			else if ((pinsToTransmit[_nn] >= 14) && (pinsToTransmit[_nn] <=21)) 
			{
				pinMode(pinsToTransmit[_nn], INPUT);
				myValue = analogRead(pinsToTransmit[_nn]);
			}
			// digital pins
			else { 
				pinMode(pinsToTransmit[_nn], INPUT);
				myValue = digitalRead(pinsToTransmit[_nn]); 
			} 
			#ifdef DEBUG
			SerialprintF("Preparing report on pin : "); 
			Serialprint(pinsToTransmit[_nn]); 
			SerialprintF(" node # "); 
			Serialprint(_mm);
			SerialprintF(" with value ");
			Serialprintln(myValue);
			#endif
			prepareAndSendPayLoad(rxlength,G_myVerb,myValue,password,_mm);
			digitalWrite(my_greenLed, LOW);
			SerialprintlnF(" Report sent  ");
		}   
		
		// digitalWrite(my_greenLed, LOW); // not sending anymore 
		// digitalWrite(my_blueLed, HIGH); // switch to receive mode not yet, we are not listening 
		if (!receiving) { 
			//man.beginReceiveArray(rxlength, rx_tx_buffer);
			//receiving = true; 
		}
	  } 
} 
unsigned long findDIOCode(int _node, int _value){
	#ifdef DEBUG
	#endif
	// table is 10 (5 DIOs, each either on or off) .. arg "node" starts at 0 so formula for _ndx under is simplified. 
    //unsigned long CCode[10] = {0,0,1,1,2,2,1637792897,1637792913,1637792896,1637792912};
    unsigned long _retCode=0;
    int _ndx =((2*(_node)) + _value); 
	#ifdef DEBUG
	#endif
    _retCode = CCode[_ndx]; 
    return(_retCode); 
}
unsigned long findIRDCode(int _node, int _value){
	#ifdef DEBUG
	#endif
	// table is 10 (5 DIOs, each either on or off) .. arg "node" starts at 0 so formula for _ndx under is simplified. 
    //unsigned long CCode[10] = {0,0,1,1,2,2,1637792897,1637792913,1637792896,1637792912};
    unsigned long _retCode=0;
    int _ndx =((2*(_node)) + _value); 
	#ifdef DEBUG
	#endif
    _retCode = G_IRCode[_ndx]; 
    return(_retCode); 
}
/// prepare and send the actual datagram 
void prepareAndSendPayLoad(uint8_t myLength, char * G_myVerb, unsigned long myValue, uint8_t myPassword, uint8_t myNodeNumber ) {
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
  digitalWrite(my_greenLed, HIGH); // Switch to sending mode 
  uint8_t myData[rxlength]; // this contains the whole frame to send, including payload and contrl chars
// start of preparation   
  // reset data to nothing 
  // chasing corruption myData[0] = '\0';  
  myData[0] = myLength; 
  myData[1] = myNodeNumber;  // identify Emitting node // POLIN have to find a way to set this value on each sensor of the RFTXNODE
  myData[2] = myPassword; 
  // verb ; 
  myData[3] = G_myVerb[0];
  myData[4] = G_myVerb[1];
  myData[5] = G_myVerb[2];
  // value 
  myData[6] = (byte) ((myValue & 0xFF000000) >> 24 );
  myData[7] = (byte) ((myValue & 0x00FF0000) >> 16 );
  myData[8] = (byte) ((myValue & 0x0000FF00) >> 8  );
  myData[9] = (byte) ((myValue & 0X000000FF)       );
//  int len = strlen(payLoad); // this should be strictly equal to 7 (terminating 0 is not counted by strlen ! 
//  uint8_t len = 7 ;
  myData[10]=myPassword; 
  //chasing corruption myData[15] = '\0'; // end of cstring // position=15
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
  delay(100); //debatable 
  
 }

void receiveManchester() {
	digitalWrite(my_blueLed, HIGH);
	//SerialprintlnF(" ReceiveManchester ");
	char msg [] = "In receivedManchester " ;
	trapCorruption(msg,strlen(msg));
	//memcpy(rx_tx_buffer,fakeBuff,16); // debug fake packet
 //if (1) // (man.receiveComplete())  // according to manchester Lib, we have a consistent frame.  

	if (man.receiveComplete())  // according to manchester Lib, we have a consistent frame.  
//		if (1)  // according to manchester Lib, we have a consistent frame.  

 {
	char msg [] = " man.receive complete " ;
	trapCorruption(msg,strlen(msg));
	char _l_myVerb[4] = "xxx" ;
    // Data structure to receive is 
    // [0]length of packet 
    // [1]Originator # 
    // [2]password  
    // [3][10] payload (length 7) 
      // payload verb : [3][4][5]
      // payload value (unsigned long) [6][7][8][9] 
    // [10] repeated password // e.g. paranoia 
    // [11][12][13][14]CRC32 
    // [15] \0 
    // compute expected CRC from the received frame actual data
    size_t numBytes = rxlength - 5; // rx_tx_buffer - CRC32 information - final '\0' 
    // corruption hunting numBytes = 11 ; 
    // corruption hunting long myExpectedCRC = CRC32::calculate(rx_tx_buffer, numBytes);
	uint32_t myExpectedCRC = CRC32::calculate(rx_tx_buffer, numBytes);
    //Serialprintln("1 histcommandindex is : "+ String(histCommandIndex) );
	// not used in relay    uint8_t receivedSize = 0;
    // corruption hunting commented so we use the global verb char  my_Verb[4] = "" ; 
	// unused    uint8_t myManReceivedSize = rx_tx_buffer[0];  // size in first byte
    uint8_t mySender = rx_tx_buffer[1]; // who sent it ? 
    //uint8_t myfirstPassword = rx_tx_buffer[2];
    //int m=5 ;
    //int o=10 ; int v=3;   
    // not used in relay       boolean foundslash=false; 
    _l_myVerb[0] = rx_tx_buffer[3];
    _l_myVerb[1] = rx_tx_buffer[4];
    _l_myVerb[2] = rx_tx_buffer[5];
    _l_myVerb[3] = '\0';
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
	#ifdef DEBUG
	SerialprintF("received:  Verb "); 
	Serialprint(_l_myVerb) ;
	SerialprintF(" received from sender: "); 
	Serialprint(mySender);
	SerialprintF(" value : "); 
	Serialprintln(my_value); 
	#endif
    uint32_t myReadCRC = 0;
    myReadCRC += (uint32_t)rx_tx_buffer[11] << 24; 
    myReadCRC += (uint32_t)rx_tx_buffer[12] << 16; 
    myReadCRC += (uint32_t)rx_tx_buffer[13] << 8; 
    myReadCRC += (uint32_t)rx_tx_buffer[14];
	#ifdef DEBUG
	SerialprintF("CRCs are (read vs expected: "); 
	Serialprint(myReadCRC); 
	SerialprintF(" vs ") ; 
	Serialprintln(myExpectedCRC); 
	#endif
    /// delete wroint long myReadCRC = (unsigned long)(rx_tx_buffer[14] << 24) | (rx_tx_buffer[13] << 16) | (rx_tx_buffer[12] << 8) | rx_tx_buffer[11];
    if (myReadCRC == myExpectedCRC) { 
		#ifdef DEBUG
		displayBuffers(); // check memory corruption 
		SerialprintlnF(" CRC checked ");
		#endif
		// copy input to output buffer (as input buffer will be destroyed as soon as we start reading again) ? 
		memcpy(tx_buffer,rx_tx_buffer,rxlength);
		#ifdef DEBUG
		displayBuffers(); // check memory corruption 
		#endif
		//Flag packet as valid for future processing 
		received = true; 
        } 
        else {
          received = false; // Packet is malformed, flag it as such 
		  #ifdef DEBUG
          SerialprintlnF(" CRC error ");
		  #endif
          }
		  // man.beginReceiveArray(rxlength, rx_tx_buffer);
		  //receiving = true; 
		  receiving = false; 
	  }
    // restore man.beginReceiveArray(rxlength, rx_tx_buffer);
	if (!receiving) { 
	    digitalWrite(my_blueLed, HIGH); // switch to receive mode
		SerialprintlnF(" about to beginreceiveArray ");
		man.beginReceiveArray(rxlength, rx_tx_buffer);
		receiving = true; 
	}
}
// Debug memory status function 
/* void memtest() {
char sbuf[64]; // output string

char *heapend=sbrk(0);
register char * stack_ptr asm ("sp");
struct mallinfo mi=mallinfo();
sprintf(sbuf, "Dyn.RAM used: %-10ld ", mi.uordblks);
Serial.println(sbuf);

sprintf(sbuf, "Prg.stat.RAM used %-10ld ", & _end - ramstart);
Serial.println(sbuf);

sprintf(sbuf, "Stack RAM used %-10ld ", ramend - stack_ptr);
Serial.println(sbuf);

sprintf(sbuf, "Free mem: %-10ld ", stack_ptr - heapend + mi.fordblks);
Serial.println(sbuf);

}
 */

//void condSerialPrint(String _myline){
//#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
//#define Serialprintln nop 
//#else  
//#define SERIALABLE 1
//#endif  
//}
//void my_delay(int n ){ // in milliseconds 
//  for (int nn=0;nn<n;nn++)
//    { 
//    delayMicroseconds(1000);// maximum delay is 65535 
//	}
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
