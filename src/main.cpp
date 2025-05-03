#include <Arduino.h>
#include <stdio.h>
#include <IWatchdog.h>

// Load FastLED
// #include <FastLED.h>

// canbus stuff
#include <STM32_CAN.h>
static CAN_message_t CAN_RX_msg;

#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_MY_IFACE_TYPE BOX_SW_6GANG_HIGH
#define CAN_SELF_MSG 1


STM32_CAN can1( CAN1, ALT );

template <typename T>
void cpArray(T from[], T to[], int len) // copies one array to another
{
  for (int i = 0; i < len; i++)
    to[i] = from[i];
}

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 500

static void setDisplayMode(uint8_t *data, uint8_t displayMode) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  static uint16_t displayID = (data[4] << 8) | data[5]; // switch ID
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  
  switch (displayMode) {
    case 0: // display off
      Serial.printf("Unit %d Display %d OFF\n", unitID, displayID);
      break;
    case 1: // display on
      Serial.printf("Unit %d Display %d ON\n", unitID, displayID);
      break;
    case 2: // clear display
      Serial.printf("Unit %d Display %d CLEAR\n", unitID, displayID);
      break;
    case 3: // flash display
      Serial.printf("Unit %d Display %d FLASH\n", unitID, displayID);
      break;
    default:
      Serial.println("Invalid display mode");
      break;
  }
}

static void setSwMomDur(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t swDuration = (data[6] << 8) | data[7]; // duration in msD 
}


static void setSwBlinkDelay(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t swBlinkDelay = (data[6] << 8) | data[7]; // delay in ms 
}

static void setSwStrobePat(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint8_t swStrobePat = data[6]; // strobe pattern
}


static void setPWMDuty(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t PWMDuty = (data[6] << 8) | data[7]; // switch ID 
}

static void setPWMFreq(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t PWMFreq = (data[6] << 8) | data[7]; // switch ID 
}
static void setSwitchMode(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint8_t switchMode = data[6]; // switch mode

  switch (switchMode) {
    case 0: // solid state (on/off)
      break;
    case 1: // one-shot momentary
      break;
    case 2: // blinking
      break;
    case 3: // strobing
      break;
    case 4: // pwm
      break;
    default:
      Serial.println("Invalid switch mode");
      break;
  }

}

static void setSwitchState(uint8_t *data, uint8_t swState) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  
  switch (swState) {
    case 0: // switch off
      Serial.printf("Unit %d Switch %d OFF\n", unitID, switchID);
      break;
    case 1: // switch on
      Serial.printf("Unit %d Switch %d ON\n", unitID, switchID);
      break;
    case 2: // momentary press
      Serial.printf("Unit %d Switch %d MOMENTARY PRESS\n", unitID, switchID);
      break;
    default:
      Serial.println("Invalid switch state");
      break;
  }
}



static void sendIntroduction() {
  uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  // send_message(CAN_MY_IFACE_TYPE, dataBytes, sizeof(dataBytes));

}

static void sendIntroack() {
  uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  // send_message(ACK_INTRODUCTION, dataBytes, sizeof(dataBytes));
}







void setup() {
  pinMode(PC13, OUTPUT); // blue pill LED
  Serial.begin(115200);
  delay(5000);

  Serial.begin(115200);
  can1.begin(true); // begin CAN bus with auto retransmission
  can1.setBaudRate(250000);  //250KBPS

  Serial.begin(115200);

  Serial.println("CAN Bus Test");


}

int lastPending = 0;
uint32_t last = 0;
int lastMillis = 0;
void loop()

{
  if (millis() - lastMillis > POLLING_RATE_MS) {
    lastMillis = millis();
    Serial.print(".");
    digitalWrite(PC13, digitalRead(PC13) ^ 1); // toggle LED
  }
  if (can1.read(CAN_RX_msg) ) {
    Serial.print("Channel: ");
    Serial.print(CAN_RX_msg.bus);
    if (CAN_RX_msg.flags.extended == false) {
      Serial.print(" Standard ID:");
    }
    else {
      Serial.print(" Extended ID:");
    }
    Serial.print(CAN_RX_msg.id, HEX);

    Serial.print(" DLC: ");
    Serial.print(CAN_RX_msg.len);
    if (CAN_RX_msg.flags.remote == false) {
       Serial.print(" buf: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        Serial.print("0x"); 
        Serial.print(CAN_RX_msg.buf[i], HEX); 
        if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
      }
      Serial.println();
    } else {
       Serial.println(" Data: REMOTE REQUEST FRAME");
    }
  }
} // end of loop