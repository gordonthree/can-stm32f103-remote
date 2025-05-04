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

// Interval:
uint16_t TRANSMIT_RATE_MS = 1000;
#define POLLING_RATE_MS 500


struct outputSwitch {
  uint8_t  swState = 0;          // switch state on, off, momentary
  uint8_t  swMode  = 0;          // switch mode 0 toggle, 1 momentary, 2 blinking, 3 strobe, 4 pwm, 5 disabled
  uint8_t  swType  = 0;          // mosfet, relay, sink
  uint8_t  featuresMask[2];    // feature mask
  uint16_t pwmDuty = 20;       // pwm duty cycle
  uint16_t pwmFreq = 1000;     // pwm frequency
  uint16_t blinkDelay = 5000;  // blink delay in ms
  uint16_t momPressDur = 500;  // momentary press duration in ms
  uint8_t  strobePat = 1;      // strobe pattern
  uint8_t  stateMemory = 1;    // state memory
  time_t   lastSeen = 0;       // last time seen
};

struct outputSwitch nodeSwitch[8]; // list of switches

unsigned long previousMillis = 0;  // will store last time a message was send

static const char *TAG = "can_control";
static const uint8_t mySwitchCount = 6; // number of switches
static const uint8_t* myNodeFeatureMask = FEATURE_BOX_SW_6GANG_HIGH;

volatile uint8_t nodeSwitchState[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // switch state
volatile uint8_t nodeSwitchMode[8]  = {0, 0, 0, 0, 0, 0, 0, 0};  // switch mode

volatile uint16_t introMsg[8]       = {0, 0, 0, 0, 0, 0, 0, 0};  // intro messages  
volatile uint8_t  introMsgPtr       = 0;                         // intro message pointer
volatile uint8_t  introMsgData[8]   = {0, 0, 0, 0, 0, 0, 0, 0};  // intro message data
volatile uint8_t  introMsgCnt       = 0;                         // intro message count

volatile static uint32_t UID[3];

int period = 1000;
int8_t ipCnt = 0;

static volatile uint8_t myNodeID[] = {0, 0, 0, 0}; // node ID

void getmyNodeID(){

  UID[0] = HAL_GetUIDw0();
  UID[1] = HAL_GetUIDw1();
  UID[2] = HAL_GetUIDw2();

  Serial.printf("UID: %08x:%08x:%08x\n", UID[0], UID[1], UID[2]);

  //   Serial.printf("Node ID: %02x:%02x:%02x:%02x\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
  // } else {
  //   Serial.println("Failed to set NODE ID");
  // }
}


// print list of switches and their attributes to the web console
static void dumpSwitches() {
  Serial.println("\n\n--------------------------------------------------------------------------\n");

  Serial.println("Switches:\n\n");
  for (int i = 0; i < mySwitchCount; i++) {
    Serial.printf("Switch %d: Last Update: %d\n", i, nodeSwitch[i].lastSeen);
      
    Serial.printf("State %d, Mode %d, Type %d, Feature Mask %02x:%02x\n",  
      nodeSwitch[i].swState,
      nodeSwitch[i].swMode,
      nodeSwitch[i].swType,
      nodeSwitch[i].featuresMask[0],
      nodeSwitch[i].featuresMask[1]);
      
    Serial.printf("pwmDuty %d, pwmFreq %d, blinkDelay %d, momPressDur %d, strobePat %d\n\n",
      nodeSwitch[i].pwmDuty,
      nodeSwitch[i].pwmFreq,
      nodeSwitch[i].blinkDelay,
      nodeSwitch[i].momPressDur,
      nodeSwitch[i].strobePat);
      
      delay(5);
  }
  Serial.println("\n\nEnd of Switches\n");
  Serial.println("--------------------------------------------------------------------------\n\n");

}

static void send_message(const uint16_t msgID, const uint8_t *msgData, const uint8_t dlc) {
  CAN_message_t message;
  // static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  /*
  // Format message
  message.identifier = msgID;       // set message ID
  message.extd = 0;                 // 0 = standard frame, 1 = extended frame
  message.rtr = 0;                  // 0 = data frame, 1 = remote frame
  message.self = CAN_SELF_MSG;      // 0 = normal transmission, 1 = self reception request 
  message.dlc_non_comp = 0;         // non-compliant DLC (0-8 bytes)  
  message.data_length_code = dlc;   // data length code (0-8 bytes)
  memcpy(message.data, (const uint8_t*) msgData, dlc);  // copy data to message data field 
  
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    // Serial.printf("TX: MSG: %03x WITH %u DATA", msgID, dlc);

    // printf("Message queued for transmission\n");
    // Serial.printf("TX: MSG: %03x Data: ", msgID);
    // for (int i = 0; i < dlc; i++) {
    //   Serial.printf("%02x ", message.data[i]);
    // }
    // Serial.printf("\n");
  } else {
    leds[0] = CRGB::Red;
    FastLED.show();
    // ESP_LOGE(TAG, "Failed to queue message for transmission, initiating recovery");
    Serial.printf("ERR: Failed to queue message for transmission, resetting controller\n");
    twai_initiate_recovery();
    twai_stop();
    Serial.printf("WARN: twai Stopped\n");
    vTaskDelay(500);
    twai_start();
    Serial.printf("WARN: twai Started\n");
    // ESP_LOGI(TAG, "twai restarted\n");
    // wifiOnConnect();
    vTaskDelay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  // vTaskDelay(100);
  */
}

static void rxSwMomDur(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t swDuration = (data[5] << 8) | data[6]; // duration in ms
  
  nodeSwitch[switchID].momPressDur = swDuration; // update momentary press duration
}

static void rxSwBlinkDelay(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t swBlinkDelay = (data[5] << 8) | data[6]; // delay in ms 

  nodeSwitch[switchID].blinkDelay = swBlinkDelay; // update blink delay
}

static void rxSwStrobePat(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint8_t swStrobePat = data[5]; // strobe pattern

  nodeSwitch[switchID].strobePat = swStrobePat; // update strobe pattern
}

static void rxPWMDuty(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t PWMDuty = (data[5] << 8) | data[6]; // pwm duty cycle

  nodeSwitch[switchID].pwmDuty = PWMDuty; // update pwm duty cycle
}

static void rxPWMFreq(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t PWMFreq = (data[5] << 8) | data[6]; // pwm frequency 

  nodeSwitch[switchID].pwmFreq = PWMFreq; // update pwm frequency
}

static void rxSwitchState(const uint8_t *data, const uint8_t swState) {
  uint8_t switchID = data[4]; // switch ID 
  // static uint8_t unitID[] = {data[0], data[1], data[2], data[3]}; // unit ID
  uint8_t dataBytes[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID}; // send my own node ID, along with the switch number

  Serial.printf("RX: Set Switch %d State %d\n", switchID, swState);
  // nodeSwitchState[switchID] = swState; // update switch buffer
  nodeSwitch[switchID].swState = swState; // update switch buffer
  // nodeSwitch[switchID].lastSeen = getEpoch(); // update last seen time
  

  switch (swState) {
    case 0: // switch off
      // send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
      break;
    case 1: // switch on
      // send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
      break;
    case 2: // momentary press
      // send_message(DATA_OUTPUT_SWITCH_MOM_PUSH , dataBytes, sizeof(dataBytes));
      // send_message(DATA_OUTPUT_SWITCH_ON, dataBytes, sizeof(dataBytes));
      // send_message(DATA_OUTPUT_SWITCH_OFF, dataBytes, sizeof(dataBytes));
      break;
    default:
      Serial.println("Invalid switch state");
      break;
  }
}

static void rxSwitchMode(const uint8_t *data) {
  uint8_t switchID = data[4]; // switch ID 
  uint8_t switchMode = data[5]; // switch mode

  uint8_t dataBytes[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID, switchMode}; // send my own node ID, along with the switch number

  Serial.printf("RX: Set Switch %d Mode %d\n", switchID, switchMode);
  // send_message(DATA_OUTPUT_SWITCH_MODE, dataBytes, sizeof(dataBytes));    
  // nodeSwitchMode[switchID] = switchMode; // update switch mode
  nodeSwitch[switchID].swMode = switchMode; // update switch mode
  // nodeSwitch[switchID].lastSeen = getEpoch(); // update last seen time


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
    case 5: // disabled
      break;
    default:
      Serial.println("Invalid switch mode");
      break;
  }
}

// send an introduction message to the bus
// static void txIntroduction(const uint8_t* txNodeID, const uint8_t* txNodeFeatureMask, const uint_8t txMsgData, const uint16_t txmsgID, const uint8_t ptr) {
static void txIntroduction(uint8_t* txNodeID, uint8_t* txNodeFeatureMask, uint8_t txMsgData, uint16_t txmsgID, uint8_t ptr) {
     Serial.printf("TX: INTRO PTR %d\n", ptr);    

    if (txmsgID < 1) {
      // Serial.printf("RX: INTRO ACK NULL\n");
      return; // exit function if message ID is null
    }

    if (ptr == 0) {
      uint8_t dataBytes[6] = { txNodeID[0], txNodeID[1], txNodeID[2], txNodeID[3], 
                               txNodeFeatureMask[0], txNodeFeatureMask[1] }; 

      send_message(txmsgID, dataBytes, 6);
    } else {
      uint8_t dataBytes[5] = { txNodeID[0], txNodeID[1], txNodeID[2], txNodeID[3], txMsgData }; 

      send_message(txmsgID, dataBytes, 5);
    }
}

static void nodeCheckStatus() {
  if (FLAG_SEND_INTRODUCTION) {
    // send introduction message to all nodes
    txIntroduction((uint8_t*)myNodeID, (uint8_t*)myNodeFeatureMask, introMsgData[introMsgPtr], introMsg[introMsgPtr], introMsgPtr);
    Serial.printf("TX: INTRO MSG %d OF %d\n", introMsgPtr, introMsgCnt);

    if (introMsgPtr >= introMsgCnt) {
      FLAG_SEND_INTRODUCTION = false; // clear flag to send introduction message
      FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation 
    }
  }

  if (!FLAG_BEGIN_NORMAL_OPER) {
    return; // normal operation not started, exit function
  }

  for (uint8_t switchID = 0; switchID < mySwitchCount; switchID++) {
    uint8_t swState = nodeSwitch[switchID].swState; // get switch state
    uint8_t swMode = nodeSwitch[switchID].swMode; // get switch mode
    uint8_t stateData[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID}; // send my own node ID, along with the switch number
    uint8_t modeData[] = {myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], switchID, swMode}; // send my own node ID, along with the switch number
      
    // send_message(DATA_OUTPUT_SWITCH_MODE, modeData, sizeof(modeData));  
    // Serial.printf("TX: DATA: Switch %d State %d Mode %d\n", switchID, swState, swMode);

    switch (swState) {
      case 0: // switch off
        send_message(DATA_OUTPUT_SWITCH_OFF, stateData, sizeof(stateData));
        break;
      case 1: // switch on
        send_message(DATA_OUTPUT_SWITCH_ON, stateData, sizeof(stateData));
        break;
      case 2: // momentary press
        send_message(DATA_OUTPUT_SWITCH_MOM_PUSH, stateData, sizeof(stateData));
        break;
      default:
        break;
    }
  }
 
}

static void handle_rx_message(CAN_message_t &message) {
  bool msgFlag = false;
  bool haveRXID = false; 
  int msgIDComp;
  uint8_t rxNodeID[4] = {0, 0, 0, 0}; // node ID

  /*

  // check if message contains enough data to have node id
  if (message.data_length_code >= 3) { 
    memcpy((void *)rxNodeID, (const void *)message.data, 4); // copy node id from message
    msgIDComp = memcmp((const void *)rxNodeID, (const void *)myNodeID, 4);
    haveRXID = true; // set flag to true if message contains node id

    if (msgIDComp == 0) { // message is for us
      msgFlag = true; // message is for us, set flag to true
    }
  }


  // if ((!msgFlag) && (message.identifier <= 0x17F)) { // switch control message but not for us
  //   return; // exit function
  // } 

  if (message.data_length_code > 0) { // message contains data, check if it is for us
    if (msgFlag) {
      // Serial.printf("RX: MATCH MSG: %03x DATA: %u\n", message.identifier, message.data_length_code);
    } else {
      Serial.printf("RX: NO MATCH MSG: %03x DATA: u%\n", message.identifier, message.data_length_code);
    }
  } else {
    if (msgFlag) {
      // Serial.printf("RX: MATCH MSG: %03x NO DATA\n", message.identifier);
    } else {
      Serial.printf("RX: NO MATCH MSG: %03x NO DATA\n", message.identifier);
    } 
  }


  switch (message.identifier) {
    case MSG_NORM_OPER: // normal operation message
      Serial.printf("RX: Normal Operation Message\n");
      FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation
      // introMsgPtr = introMsgPtr + 1; // increment intro message pointer 4th step
      break;
    case MSG_HALT_OPER: // halt operation message
      Serial.printf("RX: Halt Operation Message\n");
      // introMsgPtr = 0; // reset intro message pointer
      FLAG_BEGIN_NORMAL_OPER = false; // clear flag to halt normal operation
      break;
    case SW_SET_OFF:            // set output switch off
      rxSwitchState(message.data, 0);
      break;
    case SW_SET_ON:             // set output switch on
      rxSwitchState(message.data, 1);
      break;
    case SW_MOM_PRESS:          // set output momentary
      rxSwitchState(message.data, 2);
      break;
    case SW_SET_MODE:           // setup output switch modes
      rxSwitchMode(message.data);
      break;
    case SW_SET_PWM_DUTY:          // set output switch pwm duty
      rxPWMDuty(message.data);  
      break;
    case SW_SET_PWM_FREQ:          // set output switch pwm frequency
      rxPWMFreq(message.data);
      break;
    case SW_SET_MOM_DUR:          // set output switch momentary duration
      rxSwMomDur(message.data);
      break;
    case SW_SET_BLINK_DELAY:          // set output switch blink delay
      rxSwBlinkDelay(message.data);
      break;
    case SW_SET_STROBE_PAT:          // set output switch strobe pattern
      rxSwStrobePat(message.data);
      break;
    case REQ_SWITCHBOX: // request for box introduction, kicks off the introduction sequence
      if (haveRXID) { // check if REQ message contains node id
        Serial.printf("RX: REQ BOX Responding to %02x:%02x:%02x:%02x\n", rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
        introMsgPtr = 0; // reset intro message pointer
        FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      }
      break;
    case ACK_SWITCHBOX:
      if (msgFlag) { // message was sent to our ID
        if (introMsgPtr < introMsgCnt) {
          Serial.printf("RX: INTRO ACK %d\n", introMsgPtr);  
          FLAG_SEND_INTRODUCTION = true; // keep sending introductions until all messages have been acknowledged
          introMsgPtr = introMsgPtr + 1; // increment intro message pointer 1st step
        }
      }
      break;
    
    default:
 
      break;
  }
*/

} // end of handle_rx_message

static void TaskTWAI(void *pvParameters) {
  // give some time at boot the cpu setup other parameters


  // hardware acceptance filter
  // filter 0x100:0x17f and 0x410:0x47f
  // filter also contains DB1 of the node ID  
  const uint16_t filterF1 = (0x100 << 5) | (myNodeID[0] >> 4);            
  const uint16_t filterF2 = (0x400 << 5) | (myNodeID[0] & 0x0F);  
  // const uint32_t maskF1F2 = (uint32_t) 0xF00FF00F;
  const uint32_t maskF1F2 = (uint32_t) 0xFF00FF0;


  // twai_filter_config_t f_config = {
  //   .acceptance_code = (uint32_t)0x20000000, // (0x100 << 21)
  //   .acceptance_mask = (uint32_t)0xF0000000,
  //   .single_filter = true          // Confirm single filter mode
  // };

  
}

static void loadJSONConfig() {/* 
  Serial.println("Starting JSON Parsing...");

  // Allocate the JsonDocument (use https://arduinojson.org/v6/assistant/ for sizing)
  // StaticJsonDocument doc; // Increased buffer slightly from recommendation

  // Deserialize the JSON input
  DeserializationError error = deserializeJson(doc, jsonInput);

  // Check for parsing errors
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return; // Don't continue if parsing failed
  }

  Serial.print("Loading JSON into memory... ");

  for (int i = 0; i <= 7; i++) {
    String currentKey = String(i); // Keys are strings: "0", "1", ...

    // Use doc[currentKey].is<JsonArray>() instead of containsKey()
    if (doc[currentKey].is<JsonArray>()) {
      JsonArray currentArray = doc[currentKey].as<JsonArray>();

      nodeSwitch[i].swState = currentArray[0]; // Set switch state
      nodeSwitch[i].swMode = currentArray[1]; // Set switch mode
      nodeSwitch[i].swType = currentArray[2]; // Set switch type
      nodeSwitch[i].featuresMask[0] = currentArray[3]; // Set feature mask byte 1
      nodeSwitch[i].featuresMask[1] = currentArray[4]; // Set feature mask byte 2
      nodeSwitch[i].pwmDuty = currentArray[5]; // Set PWM duty cycle
      nodeSwitch[i].pwmFreq = currentArray[6]; // Set PWM frequency
      nodeSwitch[i].blinkDelay = currentArray[7]; // Set blink delay
      nodeSwitch[i].momPressDur = currentArray[8]; // Set momentary press duration
      nodeSwitch[i].strobePat = currentArray[9]; // Set strobe pattern
      nodeSwitch[i].stateMemory = currentArray[10]; // Set state memory
      nodeSwitch[i].lastSeen = currentArray[11]; // Set last seen time

    }
  }
  Serial.println("Finished!\n.\n"); */
}



void recvMsg(uint8_t *data, size_t len){
  Serial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  Serial.println(d);

  if (d == "C0"){
    // vTaskSuspend(canbus_task_handle); // suspend canbus task
    // digitalWrite(LED, HIGH);
  }
  if (d == "C1"){
    // vTaskResume(canbus_task_handle); // resume canbus task
    // digitalWrite(LED, HIGH);
  }
  if (d == "W"){
    // printWifi();
    // digitalWrite(LED, HIGH);
  }

  if (d == "LOAD"){
    // vTaskSuspend(canbus_task_handle); // suspend canbus task
    loadJSONConfig();
    // vTaskSuspend(canbus_task_handle); // suspend canbus task
    // digitalWrite(LED, HIGH);
  }

  if (d == "TIME"){
    // printEpoch();
    // digitalWrite(LED, HIGH);
  }

  if (d == "FAST"){
   TRANSMIT_RATE_MS = 1000;
    Serial.printf("\nTX Rate: %d ms\n", TRANSMIT_RATE_MS);
    // digitalWrite(LED, HIGH); 
  }

  if (d == "FSTR"){
    TRANSMIT_RATE_MS = TRANSMIT_RATE_MS - 250;
     Serial.printf("\nTX Rate: %d ms\n", TRANSMIT_RATE_MS);
     // digitalWrite(LED, HIGH); 
   }

  if (d == "SLOW"){
    TRANSMIT_RATE_MS = 4000;
    Serial.printf("\nTX Rate: %d ms\n", TRANSMIT_RATE_MS);
    // digitalWrite(LED, HIGH); 
  }


  if (d == "RESTART"){
    Serial.println("Restarting...");
    // ESP.restart();
    // digitalWrite(LED, HIGH);
  }

  if (d == "NODEID"){
    Serial.printf("Node ID: %02x:%02x:%02x:%02x\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
    // digitalWrite(LED, HIGH);
  }
    
  if (d=="LIST"){
    dumpSwitches();
    // dumpNodeList();
    // digitalWrite(LED, LOW);
  }
  if (d == "ON"){
    FLAG_BEGIN_NORMAL_OPER = true; // flag to start normal operation
  }
  if (d=="OFF"){
    FLAG_BEGIN_NORMAL_OPER = false; // clear flag for normal operation
  }
}




void setup() {
  pinMode(PC13, OUTPUT); // blue pill LED
  Serial.begin(115200);
  delay(5000);

  Serial.begin(115200);
  can1.begin(true); // begin CAN bus with auto retransmission
  can1.setBaudRate(250000);  //250KBPS
  can1.setMBFilterProcessing( MB0, 0x17F, 0x780, STD ); // watch the three MSB of the ID (shifted << 5)
  can1.setMBFilterProcessing( MB1, 0x47F, 0x780, STD );

  Serial.println("CAN Bus Test");
  getmyNodeID(); // get node ID from UID

}

int lastPending = 0;
uint32_t last = 0;
int lastMillis = 0;


void loop()

{
  if (millis() - lastMillis > POLLING_RATE_MS) {
    lastMillis = millis();
    // Serial.print(".");
    digitalWrite(PC13, digitalRead(PC13) ^ 1); // toggle LED
  }
  if (can1.read(CAN_RX_msg) ) {
    Serial.print("Channel: ");
    Serial.print(CAN_RX_msg.bus);
    if (CAN_RX_msg.flags.extended == false) {
      Serial.print(" Standard ID: 0x");
    }
    else {
      Serial.print(" Extended ID: 0x");
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