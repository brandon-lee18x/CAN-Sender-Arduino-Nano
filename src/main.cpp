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
CAN_params ACU_gen_2 = {0x97, 0, 8};
CAN_params pedal_inputs = {0xC8, 0, 8};
CAN_params vdm_speed = {0xF1, 0, 8};
CAN_params vdm_stuff = {0xF0, 0, 8}; //maxpower, CAN ok, System ok, TCM ok, VDM state, VDM mode
CAN_params ac_curr = {0x2116, 1, 8};
CAN_params e_rpm = {0x2016, 1, 8};
CAN_params batt_temp = {0x96, 0, 8};
CAN_params motor_temp = {0x2216, 1, 8}; //also inv temp (controller temp)
CAN_params VDM_popups = {0xF4, 0, 8};

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
    if (rxId == 0x11002) {
      Serial.println("knob frame received");
      for (int a : rxBuf) {
        Serial.print(a);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  

  if(millis()-sendTime>250){
    byte ACU_gen_data[8] = {(byte)random(0, 256), (byte)random(0, 256), 0, 0, 0, 0, 0, (byte)random(0, 101)};
    byte pedal_inputs_data[8] = {0, 0, 0, 0, (byte)random(0, 256), (byte)random(0, 256), (byte)random(0, 256),
                                (byte)random(0, 256)};
    byte vdm_speed_data[8] = {0, 0, 0, 0, 0, 0, (byte)random(0, 100), 0};
    byte vdm_stuff_data[8] = {(byte)random(1, 4), (byte)random(1, 5), 0, (byte)random(1, 3), (byte)random(1, 3), (byte)random(1, 5), (byte)random(0, 81)};
    byte ac_curr_data[8] = {(byte)random(0, 2), (byte)random(0, 2), 0, 0, 0, 0, 0, 0,};
    byte e_rpm_data[8] = {0, 0, (byte)random(0, 10), (byte)random(0, 256), 0, 0, 0, 0};
    int16_t rand = random(-32768, 32768);
    byte batt_temp_data[8] = {0, 0, 0, 0, (rand >> 8) & 0xFF, rand & 0xFF, 0, 0};
    byte motor_temp_data[8] = {0, (byte)random(0, 90), 0, (byte)random(0, 90), 0, 0, 0, 0};
    byte error_code = (byte)random(1, 12);
    byte vdm_popup_data[8] = {error_code, 1, 0, 0, 0, 0, 0, 0};
    if (error_code == 9) { //vehicle settings (trq map, etc)
      vdm_popup_data[2] = (byte)random(1, 5);
      vdm_popup_data[3] = (byte)random(0, 121);
      vdm_popup_data[4] = (byte)random(1, 5);
    }

    send_msg(ACU_gen_2.id, ACU_gen_2.ext, ACU_gen_2.len, ACU_gen_data);
    send_msg(pedal_inputs.id, pedal_inputs.ext, pedal_inputs.len, pedal_inputs_data);
    send_msg(vdm_speed.id, vdm_speed.ext, vdm_speed.len, vdm_speed_data);
    send_msg(vdm_stuff.id, vdm_stuff.ext, vdm_stuff.len, vdm_stuff_data);
    send_msg(ac_curr.id, ac_curr.ext, ac_curr.len, ac_curr_data);
    send_msg(e_rpm.id, e_rpm.ext, e_rpm.len, e_rpm_data);
    send_msg(batt_temp.id, batt_temp.ext, batt_temp.len, batt_temp_data);
    send_msg(motor_temp.id, motor_temp.ext, motor_temp.len, motor_temp_data);
    send_msg(VDM_popups.id, VDM_popups.ext, VDM_popups.len, vdm_popup_data);
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