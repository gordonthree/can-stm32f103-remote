#include <Arduino.h>
#include <stdio.h>

#ifdef STMNODE01
#include <IWatchdog.h>
#include <stm32yyxx_ll_adc.h>
// canbus stuff
#include <STM32_CAN.h>
static CAN_message_t CAN_RX_msg;

/** Setup CAN1 interface using alternate pins. */
STM32_CAN can1( CAN1, ALT ); // RX_SIZE_64, TX_SIZE_16

#define CONSOLE Serial       /** Create an alias for the Serial peripheral.  */
#elif TEENSY01
#include <Metro.h>
#include <FlexCAN.h>

Metro sysTimer = Metro(1);// milliseconds

FlexCAN teensycan1(250000, 0);
static CAN_message_t CAN_RX_msg;

#define CONSOLE Serial       /** Create an alias for the Serial peripheral, */

#elif
#define CONSOLE Serial       /** Create an alias for the Serial peripheral. */

#endif

#include <CRC32.h>



// my canbus stuff
#include <canbus_project.h>

volatile uint8_t nodeIdArrSize      = 0;
volatile uint8_t featureMaskArrSize = 0;
volatile uint8_t introMsgCnt        = 0;
volatile uint8_t introMsgPtr        = 0;

/** STM32 internal temp sensor stuff. */ 
/** Values available in datasheet. */
#if defined(STM32C0xx)
#define CALX_TEMP 30
#else
#define CALX_TEMP 25
#endif

#if defined(STM32C0xx)
#define VTEMP      760
#define AVG_SLOPE 2530
#define VREFINT   1212
#elif defined(STM32F1xx)
#define VTEMP     1430
#define AVG_SLOPE 4300
#define VREFINT   1200
#elif defined(STM32F2xx) || defined(STM32F4xx)
#define VTEMP      760
#define AVG_SLOPE 2500
#define VREFINT   1210
#endif

#ifdef STMNODE01 // TODO how do we automate this?
#define LED_ON      0
#define LED_OFF     1

#define CAN_MY_IFACE_TYPE BOX_MULTI_IO
static const uint8_t mySwitchCount = 0; // no switches
static const uint8_t mySensorCount = 3; // got sensors

static const uint8_t* myNodeFeatureMask = FEATURE_BOX_SW_6GANG_HIGH;
#elif TEENSY01
#define LED_BUILTIN 13
#define LED_ON      0
#define LED_OFF     1

#define CAN_MY_IFACE_TYPE BOX_MULTI_IO
static const uint8_t mySwitchCount = 0; // no switches
static const uint8_t mySensorCount = 3; // got sensors

static const uint8_t* myNodeFeatureMask = FEATURE_BOX_SW_6GANG_HIGH;


#endif

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

float Temperature, V_Sense, V_Ref;
char TxBuffer[30];
uint16_t AD_RES[2];
#ifdef STMNODE01
ADC_HandleTypeDef hadc1;
#endif
// 32-bit CRC calculation library
CRC32 crc;


// Interval:
uint16_t TRANSMIT_RATE_MS = 1000;
#define POLLING_RATE_MS 500


struct canNodeInfo nodeInfo;       // information on this node

unsigned long previousMillis = 0;  // will store last time a message was send

/**
 * @brief Read the unique hardware ID from the STM processor. Run it through CRC32 to create a 32-bit hash to use as the node id.
 * 
 * @param none
 * 
 * @return uint8_t 4-byte array containing a unique 32-bit value.
 * 
 */
uint8_t* getNodeID(){
  uint32_t UID[3];
  uint8_t *buf = (uint8_t*)malloc(sizeof(uint8_t) * NODE_ID_SIZE);  /** Create a buffer to store the array until it is returned. */

  #ifdef STMNODE01
  // get unique hardware id from HAL
  UID[0] = HAL_GetUIDw0();
  UID[1] = HAL_GetUIDw1();
  UID[2] = HAL_GetUIDw2();
  #endif
  // show the user
  // Serial.printf("UID: %08x:%08x:%08x\n", UID[0], UID[1], UID[2]);

  // add UID values to CRC calculation library
  crc.add(UID[0]);
  crc.add(UID[1]);
  crc.add(UID[2]);

  // show the user
  // Serial.printf("CRC %08x\n", crc.calc());

  // calculate 32-bit crc value from the uid
  int myCRC = crc.calc();

  // transfer those four bytes to myNodeID array
  buf[0] = (myCRC >> 24) & 0xFF; // get first byte of CRC
  buf[1] = (myCRC >> 16) & 0xFF; // get second byte of CRC
  buf[2] = (myCRC >> 8) & 0xFF;  // get third byte of CRC
  buf[3] = myCRC & 0xFF;         // get fourth byte of CRC

  // show the user
  Serial.printf("\nSTM32 CAN Remote\nNode ID: %02x:%02x:%02x:%02x\n\n", nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3]);
}

// convert byte array into 32-bit integer
static uint32_t unchunk32(const uint8_t* dataBytes){
  static uint32_t result = ((dataBytes[0]<<24) | (dataBytes[1]<<16) | (dataBytes[2]<<8) | (dataBytes[3]));

  return result;
}
// convert a 32-bit number into a 4 byte array
char* chunk32(const uint32_t inVal = 0) {
  static char tempStr[4] = {(inVal >> 24) && 0xFF, (inVal >> 16) && 0xFF, (inVal >>  8) && 0xFF, inVal && 0xFF};
  return tempStr;
}

#ifndef TEENSY01
static int32_t readVref() {
#ifdef STM32U0xx
  /* On some devices Internal voltage reference calibration value not programmed
     during production and return 0xFFFF. See errata sheet. */
  if ((uint32_t)(*VREFINT_CAL_ADDR) == 0xFFFF) {
    return 3300U;
  }
#endif
#ifdef __LL_ADC_CALC_VREFANALOG_VOLTAGE
#ifdef STM32U5xx
  return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC1, analogRead(AVREF), LL_ADC_RESOLUTION));
#else
  return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
#endif
#else
  return (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
#endif
}
#endif

#ifdef ATEMP
static int32_t readTempSensor(int32_t VRef) {
#ifdef __LL_ADC_CALC_TEMPERATURE
#ifdef STM32U5xx
  return (__LL_ADC_CALC_TEMPERATURE(ADC1, VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#else
  return (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#endif
#elif defined(__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS)
  return (__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(AVG_SLOPE, VTEMP, CALX_TEMP, VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#else
  return 0;
#endif
}
#endif

#ifndef TEENSY01
static int32_t readVoltage(int32_t VRef, uint32_t pin)
{
#ifdef STM32U5xx
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(ADC1, VRef, analogRead(pin), LL_ADC_RESOLUTION));
#else
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(pin), LL_ADC_RESOLUTION));
#endif
}
#endif

/**
 * @brief Print a list of outputs (switches) and their attributes a console
 * 
 * @param nodeID (optional) On a control node, print outputs for a specific node 4-bytes uint8_t
**/
static void dumpSwitches(uint8_t* nodeID[NODE_ID_SIZE]=NULL) {
  Serial.println("\n\n--------------------------------------------------------------------------\n");

  if (nodeID != NULL) {
    Serial.println("Node output modules:\n\n");
    // TODO needs more code
  } else {
    Serial.printf("Node %02x:%02x:%02x:%02x output modules:\n\n\n", nodeID[0], nodeID[1], nodeID[2], nodeID[3]);

    for (int i = 0; i < NODE_MOD_MAX_CNT; i++) {
      if ((nodeInfo.subModules->modType >= MODULE_OUTPUTS) &&           /** Print info for outputs that have been defined and. */
          (nodeInfo.subModules->modType <= (MODULE_OUTPUTS | 0x0F))) {  /** are in the proper message id range */

        Serial.printf("Switch %d: Last Update: %d\n", i, nodeInfo.subModules[i].timestamp);
          
        Serial.printf("State %d, Mode %d, Type %d, Feature Mask %02x:%02x\n",  
          nodeInfo.subModules[i].u8Value,
          nodeInfo.subModules[i].outMode,
          nodeInfo.subModules[i].modType,
          nodeInfo.subModules[i].featureMask[0],
          nodeInfo.subModules[i].featureMask[1]);
          
        Serial.printf("pwmDuty %d, pwmFreq %d, blinkDelay %d, momPressDur %d, strobePat %d\n\n",
          nodeInfo.subModules[i].pwmDuty,
          nodeInfo.subModules[i].pwmFreq,
          nodeInfo.subModules[i].blinkDelay,
          nodeInfo.subModules[i].momPressDur,
          nodeInfo.subModules[i].strobePat);
      }
    }
  }
  Serial.println("\n\nEnd of Outputs Report\n");
  Serial.println("--------------------------------------------------------------------------\n\n");

}

static void send_message(const uint16_t msgID, const uint8_t *msgData, const uint8_t dlc) {
  CAN_message_t message;
  // static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  digitalWrite(LED_BUILTIN, LED_ON);

  #ifdef STMNODE01
  // Format message
  message.id             = msgID;                                      // set message ID
  message.flags.extended = STD;                                        // 0 = standard frame, 1 = extended frame
  message.flags.remote   = 0;                                          // 0 = data frame, 1 = remote frame
  message.len            = dlc;                                        // data length code (0-8 bytes)
  #elif TEENSY01
  // Format message
  message.id             = msgID;                                      // set message ID
  message.len            = dlc;                                        // data length code (0-8 bytes)
  #endif
  
  memcpy(message.buf, (const uint8_t*) msgData, dlc);                  // copy data to message data field 

  #ifdef STMNODE01
  // Queue message for transmission
  if (can1.write(message)) {  // send message to bus, true = wait for empty mailbox
    // successful write?
  #elif TEENSY01
  // Queue message for transmission
  if (teensycan1.write(message)) {  // send message to bus, true = wait for empty mailbox
    // successful write?

  #endif
  } else {
    Serial.printf("ERR: Failed to queue message\n");
  }

  digitalWrite(LED_BUILTIN, LED_OFF);

}

static void rxSwMomDur(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t swDuration = (data[5] << 8) | data[6]; // duration in ms
  
  nodeInfo.subModules[switchID].momPressDur = swDuration; // update momentary press duration
}

static void rxSwBlinkDelay(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t swBlinkDelay = (data[5] << 8) | data[6]; // delay in ms 

  nodeInfo.subModules[switchID].blinkDelay = swBlinkDelay; // update blink delay
}

static void rxSwStrobePat(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint8_t swStrobePat = data[5]; // strobe pattern

  nodeInfo.subModules[switchID].strobePat = swStrobePat; // update strobe pattern
}

static void rxPWMDuty(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t PWMDuty = (data[5] << 8) | data[6]; // pwm duty cycle

  nodeInfo.subModules[switchID].pwmDuty = PWMDuty; // update pwm duty cycle
}

static void rxPWMFreq(uint8_t *data) {
  static uint8_t switchID = data[4]; // switch ID 
  static uint16_t PWMFreq = (data[5] << 8) | data[6]; // pwm frequency 

  nodeInfo.subModules[switchID].pwmFreq = PWMFreq; // update pwm frequency
}

static void rxSwitchState(const uint8_t *data, const uint8_t swState) {
  uint8_t switchID = data[4]; // switch ID 
  // static uint8_t unitID[] = {data[0], data[1], data[2], data[3]}; // unit ID
  uint8_t dataBytes[] = {nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3], switchID}; // send my own node ID, along with the switch number

  // Serial.printf("RX: Set Switch %d State %d\n", switchID, swState);
  // nodeSwitchState[switchID] = swState; // update switch buffer
  nodeInfo.subModules[switchID].u8Value = swState; // update switch buffer
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

  uint8_t dataBytes[] = {nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3], switchID, switchMode}; // send my own node ID, along with the switch number

  Serial.printf("RX: Set Switch %d Mode %d\n", switchID, switchMode);
  // send_message(DATA_OUTPUT_SWITCH_MODE, dataBytes, sizeof(dataBytes));    
  // nodeSwitchMode[switchID] = switchMode; // update switch mode
  nodeInfo.subModules[switchID].u8Value = switchMode; // update switch mode
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

/**
 * @brief Concatenate arrays to build a data packet for sending with a CAN message.
 *        All parameters are required.
 * 
 * @param arr1 uint8_t* Typically containing the 4-byte node id we are sending from or to.
 * @param n1 uint8_t The size of the elements contained in arr1. 
 * @usage example: uint8_t n1 = sizeof(arr1) / sizeof(arr1[0]);
 * @param arr2 uint8_t* Typically containing four more bytes to send along with the node id.
 * @param n2 uint8_t The size of the elements contained in arr2.
 * 
 * @returns uint8_t* Array containing the merged contents of arr1 and arr2.
 * 
 */
uint8_t* messageBuilder(uint8_t arr1[], uint8_t n1, uint8_t arr2[], uint8_t n2) {
  uint8_t *buf = (u_int8_t*)malloc(sizeof(uint8_t) * n1 * n2);

  memcpy(buf, arr1, n1 * sizeof(uint8_t)); // copy the first array to the buffer

  memcpy(buf + n1, arr2, n2 * sizeof(uint8_t)); // copy the second array to the buffer

  return buf; // that's all folks return complete array
}

// send an introduction message to the bus
// static void txIntroduction(const uint8_t* txNodeID, const uint8_t* txNodeFeatureMask, const uint_8t txMsgData, const uint16_t txmsgID, const uint8_t ptr) {
/**
 * @brief Introduce this node and modules to the entire net
 * 
 * @param ptr optional pointer for where we are in the introductions. leave blank to only send the node intro message
 */
static void txIntroduction(int ptr = -1) {

  if ( ptr <= 0) {  /**  Step one introduce the node itself, then move onto modules below. */
    uint16_t txMsgID = nodeInfo.nodeType;
    Serial.printf("TX: NODE TYPE %03x INTRO PTR %i\n", txMsgID, ptr );  /** Tell the user some things. */
    uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, nodeInfo.featureMask, featureMaskArrSize);

    send_message(nodeInfo.nodeType, dataBytes, sizeof(dataBytes));  // send this data to the tx queue
  }
  
  if (ptr > 0) {                                                                /** Remaining introduction steps. */
    uint8_t modPtr = (ptr - 1);                                                 /** Reduce pointer by 1 so we start at beginning of the submodules array. */
    uint16_t txMsgID = nodeInfo.subModules[modPtr].modType;                     /** Retrieve module type. */

    if (txMsgID > 0) {                                                          /** Only proceed if the module is defined. */
      Serial.printf("TX: MODULE TYPE %03x INTRO PTR %i\n", txMsgID, modPtr);    /** Tell the user some things. */
      if (nodeInfo.subModules[modPtr].sendFeatureMask) {                        /**  This module requires the feature mask to be sent with the introduction. */
        uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, nodeInfo.subModules[ptr].featureMask, featureMaskArrSize);
        send_message(txMsgID, dataBytes, sizeof(dataBytes));                    /** Send this data to the tx queue. */
      } else {                                                                  /** No feature mask is available, send a module count. */
        uint8_t txFeatureMask[] = {nodeInfo.subModules[ptr].modCount, 0};       /** Create a basic feature mask with the count for this module type. */
        uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, txFeatureMask, featureMaskArrSize);
      }
    }
  }
}

static void nodeCheckStatus() {
  if (FLAG_SEND_INTRODUCTION) {
    // send introduction message to all nodes
    txIntroduction(introMsgPtr);
    Serial.printf("TX: INTRO MSG %d OF %d\n", introMsgPtr, introMsgCnt);

    if (introMsgPtr >= introMsgCnt) {
      FLAG_SEND_INTRODUCTION = false; // clear flag to send introduction message
      // FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation 
    }
  }

  if (!FLAG_BEGIN_NORMAL_OPER) {
    return; // normal operation not started, exit function
  }

  for (int i = 0; i < NODE_MOD_MAX_CNT; i++) {
    if ((nodeInfo.subModules->modType >= MODULE_OUTPUTS) &&           /** TX info on modules that have been defined. */
        (nodeInfo.subModules->modType <= (MODULE_OUTPUTS | 0x0F))) {  /** are in the proper message id range */
    uint8_t swState = nodeInfo.subModules[switchID].u8Value; // get switch state
    uint8_t swMode = nodeInfo.subModules[switchID].outMode; // get switch mode
    uint8_t stateData[] = {nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3], switchID}; // send my own node ID, along with the switch number
    uint8_t modeData[] = {nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3], switchID, swMode}; // send my own node ID, along with the switch number
      
    // send_message(DATA_OUTPUT_SWITCH_MODE, modeData, sizeof(modeData));  
    // Serial.printf("TX: DATA: Switch %d State %d Mode %d\n", switchID, swState, swMode);

    switch (swState) {
      case OUT_MODE_ALWAYS_OFF: /** output control disabled and always off */
        // send_message(DATA_OUTPUT_SWITCH_OFF, stateData, sizeof(stateData));
        break;
      case OUT_MODE_ALWAYS_ON: /** output control disabled and always on */
        send_message(DATA_OUTPUT_SWITCH_ON, stateData, sizeof(stateData));
        break;
      case OUT_MODE_TOGGLE: /** output acts like a toggle switch off/on */
        send_message(DATA_OUTPUT_SWITCH_MOM_PUSH, stateData, sizeof(stateData));
        break;
      default:
        break;
    }
  }

  for (uint8_t sensorID = 0; sensorID < MAX_SENSOR_CNT; sensorID++) { // loop through sensors 
    if (nodeSensor[sensorID].present) {                               // only send data for sensors that are present
      uint16_t privMsg = nodeSensor[sensorID].sensorMsg;          
      if (privMsg != 0) {                                             // use private channel assigned to send the data
        char buf[CAN_MAX_DLC] = {0};          
        sprintf(buf, "%d", nodeSensor[sensorID].i32Value);            // convert signed integer to string
        send_message(privMsg, (uint8_t*) buf, CAN_MAX_DLC);           // send message with ID assigned by controller
      } else {
        char buf[CAN_MAX_DLC] = {0};
        strcat(buf, (char*)myNodeID);                                  // copy my node id to the buffer first
        strcat(buf, (char*)chunk32(nodeSensor[sensorID].i32Value));    // now copy the sensor data


      }
    }
  }
 
}

static void handle_rx_message(CAN_message_t &message) {
  bool msgFlag = false;
  bool haveRXID = false; 
  int msgIDComp;
  uint16_t msgID = message.id;
  uint8_t rxNodeID[NODE_ID_SIZE] = {0, 0, 0, 0}; // node ID

  digitalWrite(LED_BUILTIN, LED_ON);

 
  // WebSerial.printf("RX: MSG: %03x DATA: %u\n", message.id, message.len);

  // check if message contains enough data to have node id
  if (message.len >= NODE_ID_SIZE) { 
    memcpy((void *)rxNodeID, (const void *)message.buf, 4); // copy node id from message
    msgIDComp = memcmp((const void *)rxNodeID, (const void *)myNodeID, 4);
    haveRXID = true; // set flag to true if message contains node id

    if (msgIDComp == 0) { // message is for us
      msgFlag = true; // message is for us, set flag to true
    }
  }

  // if ((!msgFlag) && (message.id <= 0x17F)) { // switch control message but not for us
  //   return; // exit function
  // } 

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


    case REQ_NODE_INTRO: // request for box introduction, kicks off the introduction sequence
      if (haveRXID) { // check if REQ message contains node id
        // Serial.printf("RX: REQ NODE responding to %02x:%02x:%02x:%02x\n", rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
        introMsgPtr = 0; // reset intro message pointer
        FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      }
      break;

    case ACK_INTRO:
      if (msgFlag) { // message was sent to our ID
        if (introMsgPtr < introMsgCnt) {
          Serial.printf("RX: INTRO ACK %d\n", introMsgPtr);  
          FLAG_SEND_INTRODUCTION = true; // keep sending introductions until all messages have been acknowledged
          introMsgPtr = introMsgPtr + 1; // increment intro message pointer 1st step
        }
      }
      break;
    

      default:
        if (msgID == DATA_EPOCH) {
          uint8_t epochBytes[4] = {message.buf[0], message.buf[1], message.buf[2], message.buf[3]};
          uint32_t rxTime = 0;
          rxTime = unchunk32(epochBytes);
    
          Serial.printf("RX: EPOCH TIME %u\n", rxTime);
        }

        if (message.len > 0) { // message contains data, check if it is for us
          if (msgFlag) {
            // Serial.printf("RX: MATCH MSG: %03x DATA: %u\n", message.id, message.len);
          } else {
            Serial.printf("RX: NO MATCH MSG: %03x DATA: u%\n", message.id, message.len);
          }
        } else {
          if (msgFlag) {
            // Serial.printf("RX: MATCH MSG: %03x NO DATA\n", message.id);
          } else {
            Serial.printf("RX: NO MATCH MSG: %03x NO DATA\n", message.id);
          } 
        }
      
      
      break;
  }
  
  digitalWrite(LED_BUILTIN, LED_OFF);
  
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
  #ifdef STMNODE01 // TODO how do we automate this?

  nodeSensor[0].present    = true;
  nodeSensor[0].sensorType = NODE_INT_VOLTAGE_SENSOR;
  nodeSensor[0].sensorMsg  = DATA_INTERNAL_PCB_VOLTS;
  nodeSensor[0].dataSize   = DATA_SIZE_16BITS;
  nodeSensor[1].present    = true;
  nodeSensor[1].sensorType = NODE_CPU_TEMP;
  nodeSensor[1].sensorMsg  = DATA_NODE_CPU_TEMP;
  nodeSensor[1].dataSize   = DATA_SIZE_16BITS;
  nodeSensor[2].present    = true;
  nodeSensor[2].sensorType = BUTTON_ANALOG_KNOB;
  nodeSensor[2].sensorMsg  = SENSOR_RESERVED_72C;
  nodeSensor[2].dataSize   = DATA_SIZE_16BITS;

  introMsgCnt = 4; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[0] = (uint16_t) BOX_MULTI_IO; // generic multi io box
  introMsg[1] = nodeSensor[0].sensorType; // intro message for high current switch
  introMsg[2] = nodeSensor[1].sensorType; // intro message for low current switch
  introMsg[3] = nodeSensor[2].sensorType; // intro message for CPU temperature
  
  introMsgData[0] = 0; // send feature mask
  introMsgData[1] = 1; // one of these
  introMsgData[2] = 1; // one of these
  introMsgData[3] = 1; // and one of these

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

  // these should be constants, but volatile works instead
  nodeIdArrSize      = sizeof(nodeInfo.nodeID) / sizeof(nodeInfo.nodeID[0]);
  featureMaskArrSize = sizeof(nodeInfo.featureMask) / sizeof(nodeInfo.featureMask[0]);


#ifndef TEENSY01
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
#endif

  analogReadResolution(12);

  pinMode(LED_BUILTIN, OUTPUT); // blue pill LED
  Serial.begin(256000);
  delay(5000);
  #ifdef STMNODE01
  can1.begin(); // begin CAN bus with no auto retransmission
  can1.setBaudRate(250000);  //250KBPS
  // can1.setMBFilter(ACCEPT_ALL); // accept all messages
  // can1.enableLoopBack(false); // disable loopback mode
  // can1.enableFIFO(false); // enable FIFO mode

  can1.setMBFilterProcessing( MB0, MSG_CTRL_SWITCHES, 0x780, STD ); // 0x780 watch the four MSB of the ID 
  can1.setMBFilterProcessing( MB1, MSG_REQ_INTRO, 0x780, STD );

  getmyNodeID(); // get node ID from UID
  FLAG_SEND_INTRODUCTION = true;
  SendTimer->resume();
  #elif TEENSY01
  teensycan1.begin();
  CAN_filter_t myFilter = {.id = (MSG_CTRL_IFACE<<21) | (MASK_CTRL_IFACE <<5)};
  

  //teensycan1.setFilter()
  #endif


} // end setup

int lastPending = 0;
uint32_t last = 0;
int lastMillis = 0;


void loop() {


  if ((millis() - lastMillis) > TRANSMIT_RATE_MS) {
    lastMillis = millis();
    // Serial.print(".");
    // digitalToggle(LED_BUILTIN); // toggle LED
    
    nodeCheckStatus(); // handle node status
    
    #ifdef STMNODE01
    // Print out the value read
    int32_t VRef = readVref();                       // get the voltage reference value
    nodeSensor[0].i32Value = VRef;                   // store vref reading
    nodeSensor[1].i32Value = readTempSensor(VRef);   // store cpu temp reading
    nodeSensor[2].i32Value = readVoltage(VRef, A0);  // store value of analog 0
    #endif
  }

  #ifdef STMNODE01
  while (can1.read(CAN_RX_msg) ) {
    // Serial.printf("RX: MSG: %03x DATA: %u\n", CAN_RX_msg.id, CAN_RX_msg.len);
    handle_rx_message(CAN_RX_msg); // handle received message}
  }
  #endif

} // end of loop