#include <Arduino.h>
#include <stdio.h>

#ifdef STM32
#include <stm32yyxx_ll_adc.h>
// canbus stuff
#include <STM32_CAN.h>
static CAN_message_t CAN_RX_msg;
#define CONSOLE Serial       /** Create an alias for the Serial peripheral.  */
#endif

#ifdef STNODE103
/** Setup CAN1 interface using alternate pins. */
STM32_CAN can1( CAN1, ALT ); // RX_SIZE_64, TX_SIZE_16
#elif STNODE303
/** Setup CAN1 interface using alternate pins. */
STM32_CAN can1( CAN1, DEF ); // RX_SIZE_64, TX_SIZE_16

#include <STM32RTC.h>
/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();
#elif STNODE432
/** Setup CAN1 interface using alternate pins. */
STM32_CAN can1( CAN1, DEF); // RX_SIZE_64, TX_SIZE_16

#include <STM32RTC.h>
/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();
#endif

#ifdef TEENSY01
#include <Metro.h>
#include <FlexCAN.h>

Metro sysTimer = Metro(1);// milliseconds

FlexCAN teensycan1(250000, 0);
static CAN_message_t CAN_RX_msg;

#define CONSOLE Serial       /** Create an alias for the Serial peripheral, */
#endif

#include <CRC32.h>


// my canbus stuff
#include <canbus_project.h>      /** Include canbus project header file. */
using namespace ByteConversion;  /** Include byte conversion functions. */


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

#ifdef STM32 
#define LED_ON      0
#define LED_OFF     1

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
#ifdef STM32
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
  if (buf == NULL) {
    return NULL;
  }

  #ifdef STM32
  // get unique hardware id from HAL
  UID[0] = HAL_GetUIDw0();
  UID[1] = HAL_GetUIDw1();
  UID[2] = HAL_GetUIDw2();
  #else
  return NULL; /** Not implemented on other processors */
  #endif

  // show the user
  CONSOLE.println("STM32 CAN BUS NODE");
  CONSOLE.printf("UID: %08x:%08x:%08x\n", UID[0], UID[1], UID[2]);

  // add UID values to CRC calculation library
  crc.add(UID[0]);
  crc.add(UID[1]);
  crc.add(UID[2]);

  // show the user
  // CONSOLE.printf("CRC %08x\n", crc.calc());

  // calculate 32-bit crc value from the uid
  int myCRC = crc.calc();

  // transfer those four bytes to a buffer to return 
  buf[0] = (myCRC >> 24) & 0xFF; // get first byte of CRC
  buf[1] = (myCRC >> 16) & 0xFF; // get second byte of CRC
  buf[2] = (myCRC >> 8) & 0xFF;  // get third byte of CRC
  buf[3] = myCRC & 0xFF;         // get fourth byte of CRC

  return buf;
}



#ifdef STM32
// STM32 specific ADC stuff
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
/**
 * @brief Reads the voltage from a specified analog pin.
 * 
 * @param VRef The reference voltage in millivolts.
 * @param pin The pin number from which to read the analog voltage.
 * 
 * @return int32_t The calculated voltage in millivolts.
 */
static int32_t readVoltage(int32_t VRef, uint32_t pin)
{
#ifdef STM32U5xx
  // Calculate voltage using STM32U5xx specific function
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(ADC1, VRef, analogRead(pin), LL_ADC_RESOLUTION));
#else
  // Calculate voltage using general function
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(pin), LL_ADC_RESOLUTION));
#endif
}
#endif

/**
 * @brief Return the current epoch in seconds.
 * 
 * @details This function returns the current epoch in seconds since 1970-01-01 00:00:00 UTC.
 * 
 * @return uint32_t The current epoch in seconds.
 */
static uint32_t getEpoch() {
  #ifdef STM32
    #ifndef STNODE103
    // Get the current epoch from the internal RTC
    uint32_t epoch = rtc.getEpoch();
    return epoch;
    #else
    // If this is not an STM32, return 0
    uint32_t epoch = 0;
    return epoch;
    #endif

  #else
  // If this is not an STM32, return 0
  uint32_t epoch = 0;
  return epoch;
  #endif

}

/**
 * @brief Print a list of outputs (switches) and their attributes a console
 * 
 * @param nodeID (optional) On a control node, print outputs for a specific node 4-bytes uint8_t
**/
static void dumpSwitches(uint8_t* nodeID[NODE_ID_SIZE]=NULL) {
  CONSOLE.println("\n\n--------------------------------------------------------------------------\n");

  if (nodeID != NULL) {
    CONSOLE.println("Node output modules:\n\n");
    // TODO needs more code
  } else {
    CONSOLE.printf("Node %02x:%02x:%02x:%02x output modules:\n\n\n", nodeID[0], nodeID[1], nodeID[2], nodeID[3]);

    for (int i = 0; i < NODE_MOD_MAX_CNT; i++) {
      if ((nodeInfo.subModules->modType >= MODULE_OUTPUTS) &&           /** Print info for outputs that have been defined and. */
          (nodeInfo.subModules->modType <= (MODULE_OUTPUTS | 0x0F))) {  /** are in the proper message id range */

        CONSOLE.printf("Switch %d: Last Update: %d\n", i, nodeInfo.subModules[i].timestamp);
          
        CONSOLE.printf("State %d, Mode %d, Type %d, Feature Mask %02x:%02x\n",  
          nodeInfo.subModules[i].u8Value,
          nodeInfo.subModules[i].outMode,
          nodeInfo.subModules[i].modType,
          nodeInfo.subModules[i].featureMask[0],
          nodeInfo.subModules[i].featureMask[1]);
          
        CONSOLE.printf("pwmDuty %d, pwmFreq %d, blinkDelay %d, momPressDur %d, strobePat %d\n\n",
          nodeInfo.subModules[i].pwmDuty,
          nodeInfo.subModules[i].pwmFreq,
          nodeInfo.subModules[i].blinkDelay,
          nodeInfo.subModules[i].momPressDur,
          nodeInfo.subModules[i].strobePat);
      }
    }
  }
  CONSOLE.println("\n\nEnd of Outputs Report\n");
  CONSOLE.println("--------------------------------------------------------------------------\n\n");

}

static void send_message(const uint16_t msgID, const uint8_t *msgData, const uint8_t dlc) {
  CAN_message_t message;
  // static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  digitalWrite(LED_BUILTIN, LED_ON);

  #ifdef STM32
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

  #ifdef STM32
  // Queue message for transmission
  if (can1.write(message)) {  // send message to bus, true = wait for empty mailbox
    // successful write?
  } else {
    CONSOLE.printf("ERR: Failed to queue message\n");
  }
  #elif TEENSY01
  // Queue message for transmission
  if (teensycan1.write(message)) {  // send message to bus, true = wait for empty mailbox
    // successful write?
  } else {
    CONSOLE.printf("ERR: Failed to queue message\n");
  }

  #endif

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

/**
 * @brief Handles the reception of an output state command and updates the attributes in the nodeInfo.
 *
 * @param data The received message data
 * @param outputState The output state value from the message data, default is 0xff
 */
static void rxOutputState(const uint8_t outputID, const uint8_t outputState = OUT_STATE_INVALID) {
  nodeInfo.subModules[outputID].u8Value = outputState; /** update switch state in nodeInfo */
  nodeInfo.subModules[outputID].timestamp = getEpoch(); /** update timestamp in nodeInfo */
  
  /** TODO: implement output state handling */
  switch (outputState) {
    case OUT_STATE_DISABLED:      /** disabled output, ignore control commands */
      break;
    case OUT_STATE_OFF:           /** output is off */
      break;
    case OUT_STATE_ON:            /** output is on */
      break;
    case OUT_STATE_MOMENTARY:     /** output momentary press command */
      break;
    case OUT_STATE_BLINKING:      /** output is blinking */
      break;
    case OUT_STATE_STROBE:        /** output is strobing */
      break; 
    case OUT_STATE_PWM:           /** output is pwm */
      break;
    default:                      /** invalid output state */
      // CONSOLE.println("Invalid output state");
      break;
  }
}

/**
 * @brief Handles the reception of an output mode command and updates the attributes in the nodeInfo.
 * 
 * This function processes the incoming data to set the mode of a specified output 
 * on the node. It updates the output mode in the subModules structure and records 
 * the current time as a timestamp. The function also includes a switch-case statement 
 * for implementing specific mode handling based on the mode received.
 * 
 * @param data A pointer to an array containing the message data, where:
 *             - data[4] represents the output ID.
 *             - data[5] represents the output mode.
 */

static void rxOutputMode(const uint8_t outputID, const uint8_t outputMode = OUT_MODE_INVALID) {
  // uint8_t outputID   = data[4]; /** output ID */
  // uint8_t outputMode = data[5]; /** output mode */

  nodeInfo.subModules[outputID].u8Value = outputMode; /** update output mode in nodeInfo */
  nodeInfo.subModules[outputID].timestamp = getEpoch(); /** update timestamp */


  /** TODO: implement mode handling */
  switch (outputMode) {
    case OUT_MODE_ALWAYS_OFF: /** Output control disabled and output is always off */
      break;  
    case OUT_MODE_ALWAYS_ON:  /** Output control disabled and output is always on */
      break;
    case OUT_MODE_TOGGLE:     /** Output acts like an on/off toggle switch */
      break;
    case OUT_MODE_MOMENTARY:  /** Output acts like a momentary push switch */
      break;
    case OUT_MODE_BLINKING:   /** Output blinks on and off like a turn signal */
      break;
    case OUT_MODE_STROBE:     /** Output strobes various timing patterns */
      break;
    case OUT_MODE_PWM:        /** Output uses a PWM signal */
      break;
    default:                  /** Invalid output mode */
      // CONSOLE.println("Invalid output mode");
      break;
  }
}


// send an introduction message to the bus
// static void txIntroduction(const uint8_t* txNodeID, const uint8_t* txNodeFeatureMask, const uint_8t txMsgData, const uint16_t txmsgID, const uint8_t ptr) {
/**
 * @brief Introduce this node and modules to the entire net
 * 
 * @param ptr optional pointer for where we are in the introductions. leave blank to only send the node intro message
 */
static void txIntroduction(int ptr = 0) {

  if ( ptr <= 0) {  /**  Step one introduce the node itself, then move onto modules below. */
    uint16_t txMsgID = nodeInfo.nodeType;
    if (txMsgID == 0) return; /** bail out if we don't have a node type defined. */
    CONSOLE.printf("TX: NODE INTRO TYPE %03x PTR %i\n", txMsgID, ptr );  /** Tell the user some things. */
    uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, nodeInfo.featureMask, featureMaskArrSize);

    send_message(nodeInfo.nodeType, dataBytes, sizeof(dataBytes));  // send this data to the tx queue
  }
  
  if (ptr > 0) {                                                                /** Remaining introduction steps. */
    uint8_t modPtr = (ptr - 1);                                                 /** Reduce pointer by 1 so we start at beginning of the submodules array. */
    uint16_t txMsgID = nodeInfo.subModules[modPtr].modType;                     /** Retrieve module type. */
    if (txMsgID == 0) {
      uint8_t tmp = introMsgPtr;
      introMsgPtr = tmp + 1; /** for some reason the pointer is calling for a module that does not exist */  
    }
    if (txMsgID > 0) {                                                          /** Only proceed if the module is defined. */
      CONSOLE.printf("TX: MODULE INTRO TYPE %03x PTR %i\n", txMsgID, modPtr);    /** Tell the user some things. */
      if (nodeInfo.subModules[modPtr].sendFeatureMask) {                        /**  This module requires the feature mask to be sent with the introduction. */
        uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, nodeInfo.subModules[ptr].featureMask, featureMaskArrSize);
        send_message(txMsgID, dataBytes, sizeof(dataBytes));                    /** Send this data to the tx queue. */
      } else {                                                                  /** No feature mask is available, send a module count. */
        uint8_t txFeatureMask[] = {nodeInfo.subModules[ptr].modCount, 0};       /** Create a basic feature mask with the count for this module type. */
        uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, txFeatureMask, featureMaskArrSize);
        send_message(txMsgID, dataBytes, sizeof(dataBytes));                    /** Send this data to the tx queue. */
      }
    }
  }
}

static void nodeCheckStatus() {
  if (FLAG_SEND_INTRODUCTION) {
    // send introduction message to all nodes
    txIntroduction(introMsgPtr);
    CONSOLE.printf("TX: INTRO MSG %d OF %d\n", introMsgPtr, introMsgCnt);

    if (introMsgPtr >= introMsgCnt) {
      FLAG_SEND_INTRODUCTION = false; // clear flag to send introduction message
      // FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation 
    }
  }

  if (!FLAG_BEGIN_NORMAL_OPER) {
    return; // normal operation not started, exit function
  }

  for (uint8_t i = 0; i < NODE_MOD_MAX_CNT; i++) {                        /** Step through the submodules. */
    /** lets do outputs first */
    if ((nodeInfo.subModules[i].modType >= MODULE_OUTPUTS) &&              /** Print info for outputs that have been defined and */
        (nodeInfo.subModules[i].modType <= (MODULE_OUTPUTS | 0x0F))) {     /** are in the proper message id range. */
      uint8_t swState     = nodeInfo.subModules[i].u8Value;  /**  get output state */
      uint8_t swMode      = nodeInfo.subModules[i].outMode;  /**  get output mode */
      uint8_t modeData[]  = {nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3], i, swMode};  // send my node ID, along with the switch number
      uint8_t stateData[] = {nodeInfo.nodeID[0], nodeInfo.nodeID[1], nodeInfo.nodeID[2], nodeInfo.nodeID[3], i, swState}; // send my node ID, along with the switch number
        
      send_message(DATA_OUTPUT_SWITCH_MODE,  modeData,  sizeof(modeData));  /** send output mode data */
      send_message(DATA_OUTPUT_SWITCH_STATE, stateData, sizeof(stateData)); /** send output state data */
    } else
    
    /** now lets do sensors1 and sensors2 */
    if ((nodeInfo.subModules[i].modType >= MODULE_SENSORS1) &&                          /** Print info for sensors that have been defined and */
        (nodeInfo.subModules[i].modType <= (MODULE_SENSORS1 | 0x0F)) ||                 /** are in the proper message id range. */
        (nodeInfo.subModules[i].modType >= MODULE_SENSORS2) &&                          
        (nodeInfo.subModules[i].modType <= (MODULE_SENSORS2 | 0x0F))) {                 
          bool privMsg     = nodeInfo.subModules[i].privMsg;                            /** check if sensor uses private channel to send data */
          uint16_t txMsgID = nodeInfo.subModules[i].txMsgID;                            /** get message ID for this sensor */
          if (privMsg) {                                                                /** sensor has a private channel, sending more than four bytes of data */
            char *buf = (char *)malloc(sizeof(char) * CAN_MAX_DLC);                     /** create a buffer to hold the data. */
            sprintf(buf, "%d", nodeInfo.subModules[i].i32Value);                        /** convert signed integer to c-string */
            send_message(txMsgID, (uint8_t*) buf, CAN_MAX_DLC);                         /** send message with ID assigned by controller */
          } else {                                                                      /** assemble data to send based on a smaller sensor value */
            uint8_t sensorValue[2] = {0};
            chunk16((uint16_t) nodeInfo.subModules[i].i32Value, sensorValue);           /** convert integer to byte array */
            uint8_t* dataBytes = messageBuilder(nodeInfo.nodeID, nodeIdArrSize, sensorValue, sizeof(sensorValue) / sizeof(sensorValue[0])); /** Aassemble byte array to send with message. */
            send_message(nodeInfo.subModules[i].txMsgID, dataBytes, nodeInfo.subModules[i].dataSize); /** send message to standard message ID with the node ID and sensor data */
          }
    } 
  } /** end for */
 
} /** end nodeCheckStatus */

static void handle_rx_message(CAN_message_t &message) {
  bool msgFlag = false;
  bool haveRXID = false; 
  // int msgIDComp;
  
  uint8_t* myNodeID = nodeInfo.nodeID;
  uint16_t msgID = message.id;
  uint8_t rxNodeID[NODE_ID_SIZE] = {0, 0, 0, 0}; // node ID

  digitalWrite(LED_BUILTIN, LED_ON);

 
  // WebCONSOLE.printf("RX: MSG: %03x DATA: %u\n", message.id, message.len);

  // check if message contains enough data to have node id
  if (message.len >= NODE_ID_SIZE) { 
    memcpy((void *)rxNodeID, (const void *)message.buf, 4); // copy node id from message
    haveRXID = true; // set flag to true if message contains node id

    if (memcmp((const void *)rxNodeID, (const void *)myNodeID, 4) == 0) msgFlag = true; // message is for us, set flag to true
  }

  // if ((!msgFlag) && (message.id <= 0x17F)) { // switch control message but not for us
  //   return; // exit function
  // } 

  switch (msgID) {
    case MSG_NORM_OPER: // normal operation message
      CONSOLE.printf("RX: Normal Operation Message\n");
      FLAG_BEGIN_NORMAL_OPER = true; // set flag to begin normal operation
      // introMsgPtr = introMsgPtr + 1; // increment intro message pointer 4th step
      break;
    case MSG_HALT_OPER: // halt operation message
      CONSOLE.printf("RX: Halt Operation Message\n");
      // introMsgPtr = 0; // reset intro message pointer
      FLAG_BEGIN_NORMAL_OPER = false; // clear flag to halt normal operation
      break;
    case SW_SET_OFF:            // set output switch off
      rxOutputState(message.buf[4], OUT_STATE_OFF);
      break;
    case SW_SET_ON:             // set output switch on
      rxOutputState(message.buf[4], OUT_STATE_ON);
      break;
    case SW_MOM_PRESS:          // set output momentary
      rxOutputState(message.buf[4], OUT_STATE_MOMENTARY);
      break;
    case SW_SET_MODE:           // setup output switch modes
      rxOutputMode(message.buf[4], message.buf[5]);
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
        // CONSOLE.printf("RX: REQ NODE responding to %02x:%02x:%02x:%02x\n", rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
        introMsgPtr = 0; // reset intro message pointer
        FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      }
      break;

    case ACK_INTRO:
      if (msgFlag) { // message was sent to our ID
        if (introMsgPtr < introMsgCnt) {
          CONSOLE.printf("RX: INTRO ACK %d fo %d\n", introMsgPtr, introMsgCnt);  
          FLAG_SEND_INTRODUCTION = true; // keep sending introductions until all messages have been acknowledged
          introMsgPtr = introMsgPtr + 1; // increment intro message pointer 1st step
        }
      }
      break;

    default: /** handle other messages here */
      if (msgID == DATA_EPOCH) { /** received time of day info from controller, set our onboard RTC */

        uint8_t epochBytes[] = {message.buf[0], message.buf[1], message.buf[2], message.buf[3]};
        uint32_t rxTime = unchunk32(epochBytes);
#ifdef STM32
#ifndef STNODE103       
        rtc.setEpoch(rxTime); /** set onboard RTC */
#endif
#endif
      }

      if (message.len > 0) { // message contains data, check if it is for us
        if (msgFlag) {
          // CONSOLE.printf("RX: MATCH MSG: %03x DATA: %u\n", message.id, message.len);
        } else {
          CONSOLE.printf("RX: NO MATCH MSG: %03x DATA: u%\n", message.id, message.len);
        }
      } else {
        if (msgFlag) {
          // CONSOLE.printf("RX: MATCH MSG: %03x NO DATA\n", message.id);
        } else {
          CONSOLE.printf("RX: NO MATCH MSG: %03x NO DATA\n", message.id);
        } 
      }
      break;
  }
  
  digitalWrite(LED_BUILTIN, LED_OFF);
  
} // end of handle_rx_message

  
static void loadJSONConfig() {/* 
  CONSOLE.println("Starting JSON Parsing...");

  // Allocate the JsonDocument (use https://arduinojson.org/v6/assistant/ for sizing)
  // StaticJsonDocument doc; // Increased buffer slightly from recommendation

  // Deserialize the JSON input
  DeserializationError error = deserializeJson(doc, jsonInput);

  // Check for parsing errors
  if (error) {
    CONSOLE.print("deserializeJson() failed: ");
    CONSOLE.println(error.c_str());
    return; // Don't continue if parsing failed
  }

  CONSOLE.print("Loading JSON into memory... ");

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
  CONSOLE.println("Finished!\n.\n"); */
}



void recvMsg(uint8_t *data, size_t len){
  CONSOLE.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  CONSOLE.println(d);

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
    CONSOLE.printf("\nTX Rate: %d ms\n", TRANSMIT_RATE_MS);
    // digitalWrite(LED, HIGH); 
  }

  if (d == "FSTR"){
    TRANSMIT_RATE_MS = TRANSMIT_RATE_MS - 250;
     CONSOLE.printf("\nTX Rate: %d ms\n", TRANSMIT_RATE_MS);
     // digitalWrite(LED, HIGH); 
   }

  if (d == "SLOW"){
    TRANSMIT_RATE_MS = 4000;
    CONSOLE.printf("\nTX Rate: %d ms\n", TRANSMIT_RATE_MS);
    // digitalWrite(LED, HIGH); 
  }


  if (d == "RESTART"){
    CONSOLE.println("Restarting...");
    // ESP.restart();
    // digitalWrite(LED, HIGH);
  }

  if (d == "NODEID"){
    CONSOLE.print("Node ID: ");
    for (int i = 0; i < NODE_ID_SIZE; i++) CONSOLE.printf("%02x:", nodeInfo.nodeID[i]); // Print the node ID in hex format with colons between each byte
    CONSOLE.println(); // Print a newline character

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
  #ifdef STNODE103 // TODO how do we automate this?

  nodeInfo.nodeType  = BOX_MULTI_TVA;
  nodeInfo.subModCnt = 3;
  nodeInfo.featureMask[0] = 0;
  nodeInfo.featureMask[1] = 0;

  nodeInfo.subModules[0].modType = NODE_INT_VOLTAGE_SENSOR;
  nodeInfo.subModules[0].txMsgID = DATA_INTERNAL_PCB_VOLTS;
  nodeInfo.subModules[0].modCount = 1;
  nodeInfo.subModules[0].sendFeatureMask = false;
  nodeInfo.subModules[0].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[0].privMsg = false;

  nodeInfo.subModules[1].modType = NODE_CPU_TEMP;
  nodeInfo.subModules[1].txMsgID  = DATA_NODE_CPU_TEMP;
  nodeInfo.subModules[1].modCount = 1;
  nodeInfo.subModules[1].sendFeatureMask = false;
  nodeInfo.subModules[1].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[1].privMsg = false;

  nodeInfo.subModules[2].modType = BUTTON_ANALOG_KNOB;
  nodeInfo.subModules[2].txMsgID  = SENSOR_RESERVED_72C;
  nodeInfo.subModules[2].modCount = 1;
  nodeInfo.subModules[2].sendFeatureMask = false;
  nodeInfo.subModules[2].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[2].privMsg = false;

  introMsgCnt = nodeInfo.subModCnt + 1; /** number of intro messages */
  introMsgPtr = 0; // start at zero

#ifdef STM32
#ifndef STNODE103
  rtc.begin(); // initialize RTC 24H format
#endif
#endif

  #elif STNODE432
  nodeInfo.nodeType  = BOX_MULTI_4X4IO;
  nodeInfo.subModCnt = 3;
  nodeInfo.featureMask[0] = 0;
  nodeInfo.featureMask[1] = 0;

  nodeInfo.subModules[0].modType = NODE_INT_VOLTAGE_SENSOR;
  nodeInfo.subModules[0].txMsgID = DATA_INTERNAL_PCB_VOLTS;
  nodeInfo.subModules[0].modCount = 1;
  nodeInfo.subModules[0].sendFeatureMask = false;
  nodeInfo.subModules[0].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[0].privMsg = false;

  nodeInfo.subModules[1].modType = NODE_CPU_TEMP;
  nodeInfo.subModules[1].txMsgID  = DATA_NODE_CPU_TEMP;
  nodeInfo.subModules[1].modCount = 1;
  nodeInfo.subModules[1].sendFeatureMask = false;
  nodeInfo.subModules[1].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[1].privMsg = false;

  nodeInfo.subModules[2].modType = BUTTON_ANALOG_KNOB;
  nodeInfo.subModules[2].txMsgID  = SENSOR_RESERVED_72C;
  nodeInfo.subModules[2].modCount = 1;
  nodeInfo.subModules[2].sendFeatureMask = false;
  nodeInfo.subModules[2].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[2].privMsg = false;
  #elif STNODE303
  nodeInfo.nodeType  = BOX_MULTI_IO;
  nodeInfo.subModCnt = 3;
  nodeInfo.featureMask[0] = 0;
  nodeInfo.featureMask[1] = 0;

  nodeInfo.subModules[0].modType = NODE_INT_VOLTAGE_SENSOR;
  nodeInfo.subModules[0].txMsgID = DATA_INTERNAL_PCB_VOLTS;
  nodeInfo.subModules[0].modCount = 1;
  nodeInfo.subModules[0].sendFeatureMask = false;
  nodeInfo.subModules[0].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[0].privMsg = false;

  nodeInfo.subModules[1].modType = NODE_CPU_TEMP;
  nodeInfo.subModules[1].txMsgID  = DATA_NODE_CPU_TEMP;
  nodeInfo.subModules[1].modCount = 1;
  nodeInfo.subModules[1].sendFeatureMask = false;
  nodeInfo.subModules[1].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[1].privMsg = false;

  nodeInfo.subModules[2].modType = BUTTON_ANALOG_KNOB;
  nodeInfo.subModules[2].txMsgID  = SENSOR_RESERVED_72C;
  nodeInfo.subModules[2].modCount = 1;
  nodeInfo.subModules[2].sendFeatureMask = false;
  nodeInfo.subModules[2].dataSize   = DATA_SIZE_16BITS;
  nodeInfo.subModules[2].privMsg = false;
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
  SendTimer->setOverflow(1, HERTZ_FORMAT); // 1 Hz
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  SendTimer->attachInterrupt(1, SendData);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  SendTimer->attachInterrupt(nodeCheckStatus);
#endif
#endif

  analogReadResolution(12);

  #ifdef STM32
  CONSOLE.begin(512000);
 #else
  CONSOLE.begin(115200);
 #endif

  pinMode(LED_BUILTIN, OUTPUT); /** Setup LED for diagnostics */

  delay(5000);

  #ifdef STM32 
  /** Setup STM32 specific CAN bus interface */
  can1.begin(); // begin CAN bus with no auto retransmission
  can1.setBaudRate(250000);  //250KBPS
  // can1.setMBFilter(ACCEPT_ALL); // accept all messages
  // can1.enableLoopBack(false); // disable loopback mode
  // can1.enableFIFO(false); // enable FIFO mode

  can1.setMBFilterProcessing( MB0, MSG_CTRL_SWITCHES, MASK_BITS_11_TO_8, STD ); // 0x780 watch the four MSB of the ID 
  can1.setMBFilterProcessing( MB1, MSG_REQ_INTRO, MASK_BITS_11_TO_8, STD );

  #endif

  uint8_t* myNodeID = getNodeID();                 /** Get nodeID from unique hardware id. */
  for (uint8_t i = 0; i < sizeof(myNodeID); i++) { /** Copy temporary array into the struct. */
    nodeInfo.nodeID[i] = myNodeID[i];
  }
  FLAG_SEND_INTRODUCTION = true;

  #ifdef STM32
  SendTimer->resume(); /** STM32 start timer interrupt */
  #elif TEENSY01
  /** Teensy specific CAN bus interface */
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
    CONSOLE.print(".");
    digitalToggle(LED_BUILTIN); // toggle LED
    
    nodeCheckStatus(); // handle node status
    
    #ifdef STM32
    // Print out the value read
    int32_t VRef = readVref();                       // get the voltage reference value
    nodeInfo.subModules[0].i32Value = VRef;                   // store vref reading
    nodeInfo.subModules[1].i32Value = readTempSensor(VRef);   // store cpu temp reading
    nodeInfo.subModules[2].i32Value = readVoltage(VRef, A0);  // store value of analog 0
    #endif
  }

  #ifdef STM32
  while (can1.read(CAN_RX_msg) ) {
    // CONSOLE.printf("RX: MSG: %03x DATA: %u\n", CAN_RX_msg.id, CAN_RX_msg.len);
    handle_rx_message(CAN_RX_msg); // handle received message}
  }
  #endif

} // end of loop