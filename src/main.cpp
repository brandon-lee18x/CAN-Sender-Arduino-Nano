#include "Arduino.h"
#include "mcp_can.h"
#include <SPI.h>


#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);
unsigned long sendTime = 0;
unsigned long LED_time;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

typedef struct CAN_params {
  unsigned long id;
  byte ext;
  byte len;
} CAN_params;

//named based on "notes" column on CAN ID spreadsheet
CAN_params ACU_gen = {0x96, 0, 8};

void LED_on();
void LED_timer();
void send_msg(unsigned long id, byte ext, byte len, byte data[8]);

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 8MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  pinMode(CAN0_INT, INPUT);   // Configuring pin for /INT input
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }

    if((rxId & 0x1FFFFFFF)==0x10FFF)      //Turns on pin 13 LED for 10ms when "Data to VDM" or "Ping" frames are recieved
      LED_on();
    else if((rxId & 0x1FFFFFFF)==0x11002)
      LED_on();
        
    Serial.println();
  }
  

  if(millis()-sendTime>250){
    byte ACU_gen_data[8] = {0, 0, 0, 0, 0, 0, 0, 90};

    send_msg(ACU_gen.id, ACU_gen.ext, ACU_gen.len, ACU_gen_data);
    
  }

  LED_timer();
}

void LED_on(){  //Turn on LED
  digitalWrite(LED_BUILTIN, HIGH);
  LED_time=millis();
}

void LED_timer(){ //Turn off LED after 10ms
  if(millis()-LED_time>10){
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void send_msg(unsigned long id, byte ext, byte len, byte data[8]) {
  byte sndStat = CAN0.sendMsgBuf(id, ext, len, data);  //Send data (mimic Ping Request)
  delay(10);
  if(sndStat == CAN_OK){
    Serial.println("Ping Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }

  sendTime=millis();
}