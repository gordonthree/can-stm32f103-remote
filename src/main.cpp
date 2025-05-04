#include <Arduino.h>
#include <stdio.h>
#include <IWatchdog.h>

// Load FastLED
// #include <FastLED.h>
#include <CRC32.h>

// canbus stuff
#include <STM32_CAN.h>
static CAN_message_t CAN_RX_msg;

#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_MY_IFACE_TYPE BOX_SW_6GANG_HIGH
#define CAN_SELF_MSG 1

CRC32 crc;

STM32_CAN can1( CAN1, ALT ); // RX_SIZE_64, TX_SIZE_16

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

  // get unique hardware id from HAL
  UID[0] = HAL_GetUIDw0();
  UID[1] = HAL_GetUIDw1();
  UID[2] = HAL_GetUIDw2();

  // show the user
  // Serial.printf("UID: %08x:%08x:%08x\n", UID[0], UID[1], UID[2]);

  // add UID values to CRC calulation library
  crc.add(UID[0]);
  crc.add(UID[1]);
  crc.add(UID[2]);

  // show the user
  // Serial.printf("CRC %08x\n", crc.calc());

  // calculate 32-bit crc value from the uid
  int myCRC = crc.calc();

  // transfer those four bytes to myNodeID array
  myNodeID[0] = (myCRC >> 24) & 0xFF; // get first byte of CRC
  myNodeID[1] = (myCRC >> 16) & 0xFF; // get second byte of CRC
  myNodeID[2] = (myCRC >> 8) & 0xFF;  // get third byte of CRC
  myNodeID[3] = myCRC & 0xFF;         // get fourth byte of CRC

  // show the user
  Serial.printf("\nSTM32 CAN Remote\nNode ID: %02x:%02x:%02x:%02x\n\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
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
  static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  // Format message
  message.id             = msgID;                                      // set message ID
  message.flags.extended = STD;                                        // 0 = standard frame, 1 = extended frame
  message.flags.remote   = 0;                                          // 0 = data frame, 1 = remote frame
  message.len            = dlc;                                        // data length code (0-8 bytes)
  memcpy(message.buf, (const uint8_t*) msgData, dlc);                  // copy data to message data field 
  
  // Queue message for transmission
  if (can1.write(message)) {  // send message to bus, true = wait for empty mailbox
    // successful write?
  } else {
    Serial.printf("ERR: Failed to queue message\n");
  }
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
  
  if (txmsgID < 1) {
    // Serial.printf("RX: INTRO ACK NULL\n");
    return; // exit function if message ID is null
  }

  Serial.printf("TX: INTRO PTR %d ", ptr);  // message pointer
  Serial.printf("ID %02x:%02x:%02x:%02x ", txNodeID[0], txNodeID[1], txNodeID[2], txNodeID[3]); // node 
  Serial.printf("MSG %03x\n", txmsgID); // message ID


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
  uint16_t msgID = message.id; // get message ID
  uint8_t dlc = message.len; // get message data length code

  uint8_t rxNodeID[4] = {0, 0, 0, 0}; // node ID

  
  // check if message contains enough data to have node id
  if (dlc >= 3) { 
    memcpy((void *)rxNodeID, (const void *)message.buf, 4); // copy node id from message
    Serial.printf("RX: MSG %03x From ID %02x:%02x:%02x:%02x\n", msgID, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
    msgIDComp = memcmp((const void *)rxNodeID, (const void *)myNodeID, 4);
    haveRXID = true; // set flag to true if message contains node id

    if (msgIDComp == 0) { // message is for us
      msgFlag = true; // message is for us, set flag to true
      Serial.println("RX: Msg is for us!\n");
    }
  }


  if ((!msgFlag) && (msgID > 0x111)) { // switch control message but not for us
    Serial.println("RX: Not for us, ignoring message\n");
    return; // exit function
  } 

  if (dlc > 0) { // message contains data, check if it is for us
    if (msgFlag) {
      // Serial.printf("RX: MATCH MSG: %03x DATA: %u\n", msgID, message.data_length_code);
    } else {
      Serial.printf("RX: NO MATCH MSG: %03x DATA: %u\n", msgID, dlc);
    }
  } else {
    if (msgFlag) {
      // Serial.printf("RX: MATCH MSG: %03x NO DATA\n", msgID);
    } else {
      Serial.printf("RX: NO MATCH MSG: %03x NO DATA\n", msgID);
    } 
  }


  switch (msgID) {
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
      rxSwitchState(message.buf, 0);
      break;
    case SW_SET_ON:             // set output switch on
      rxSwitchState(message.buf, 1);
      break;
    case SW_MOM_PRESS:          // set output momentary
      rxSwitchState(message.buf, 2);
      break;
    case SW_SET_MODE:           // setup output switch modes
      rxSwitchMode(message.buf);
      break;
    case SW_SET_PWM_DUTY:          // set output switch pwm duty
      rxPWMDuty(message.buf);  
      break;
    case SW_SET_PWM_FREQ:          // set output switch pwm frequency
      rxPWMFreq(message.buf);
      break;
    case SW_SET_MOM_DUR:          // set output switch momentary duration
      rxSwMomDur(message.buf);
      break;
    case SW_SET_BLINK_DELAY:          // set output switch blink delay
      rxSwBlinkDelay(message.buf);
      break;
    case SW_SET_STROBE_PAT:          // set output switch strobe pattern
      rxSwStrobePat(message.buf);
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

} // end of handle_rx_message

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
  #ifdef STMNODE01
  introMsgCnt = 4; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[0] = (uint16_t) BOX_SW_6GANG_HIGH; // intro message for 4 relay switch box
  introMsg[1] = (uint16_t) OUT_HIGH_CURRENT_SW; // intro message for high current switch
  introMsg[2] = (uint16_t) OUT_LOW_CURRENT_SW; // intro message for low current switch
  introMsg[3] = (uint16_t) NODE_CPU_TEMP; // intro message for CPU temperature

  
  introMsgData[0] = 0; // send feature mask
  introMsgData[1] = 4; // four high current switches
  introMsgData[2] = 2; // two low current switches
  introMsgData[3] = 1; // sensor number for CPU temperature sensor

  #elif STMNODE02
  introMsgCnt = 3; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[0] = (uint16_t) BOX_SW_4GANG; // intro message for 4 relay switch box
  introMsg[1] = (uint16_t) OUT_HIGH_CURRENT_SW; // intro message for high current switch
  introMsg[2] = (uint16_t) OUT_LOW_CURRENT_SW; // intro message for low current switch

  introMsgData[0] = 0; // send feature mask
  introMsgData[1] = 2; // two high current switches
  introMsgData[2] = 2; // two low current switches
  #endif

  // setup hardware timer to send data in 50Hz pace
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *SendTimer = new HardwareTimer(Instance);
  SendTimer->setOverflow(1, HERTZ_FORMAT); // 50 Hz
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  SendTimer->attachInterrupt(1, SendData);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  SendTimer->attachInterrupt(nodeCheckStatus);
#endif
  SendTimer->resume();

  pinMode(PC13, OUTPUT); // blue pill LED
  Serial.begin(512000);
  delay(5000);

  can1.begin(); // begin CAN bus with no auto retransmission
  can1.setBaudRate(250000);  //250KBPS
  // can1.setMBFilter(ACCEPT_ALL); // accept all messages
  // can1.enableLoopBack(false); // disable loopback mode
  // can1.enableFIFO(true); // enable FIFO mode
  
  can1.setMBFilterProcessing( MB0, 0x17F, 0x780, STD ); // watch the three MSB of the ID (shifted << 5)
  can1.setMBFilterProcessing( MB1, 0x47F, 0x780, STD );

  getmyNodeID(); // get node ID from UID
  FLAG_SEND_INTRODUCTION = true;

}

int lastPending = 0;
uint32_t last = 0;
int lastMillis = 0;


void loop()

{
  if ((millis() - lastMillis) > TRANSMIT_RATE_MS) {
    lastMillis = millis();
    // Serial.print(".");
    digitalWrite(PC13, digitalRead(PC13) ^ 1); // toggle LED
    // nodeCheckStatus(); // handle node status
  }
  while (can1.read(CAN_RX_msg) ) {
    // Serial.printf("RX: MSG: %03x DATA: %u\n", CAN_RX_msg.id, CAN_RX_msg.len);
    handle_rx_message(CAN_RX_msg); // handle received message}
  }
} // end of loop