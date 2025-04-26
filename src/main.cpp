#include <Arduino.h>
#include <stdio.h>

// Load FastLED
// #include <FastLED.h>

// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_MY_IFACE_TYPE IFACE_TOUCHSCREEN_TYPE_A
#define CAN_SELF_MSG 1

#include <eXoCAN.h>
#include <IWatchdog.h>

eXoCAN can(STD_ID_LEN, BR500K, PORTB_8_9_XCVR); // constructor

template <typename T>
void cpArray(T from[], T to[], int len) // copies one array to another
{
  for (int i = 0; i < len; i++)
    to[i] = from[i];
}

struct msgFrm frame[1]; // an array of tx structures

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

unsigned long previousMillis = 0;  // will store last time a message was send
String texto;

static const char *TAG = "can_control";

// Calls = 0;

// setup the ARGB led
#define NUM_LEDS 1
#define DATA_PIN PC14

int period = 1000;
int8_t ipCnt = 0;

unsigned long time_now = 0;

// CRGB leds[NUM_LEDS];

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


/* static void handle_rx_message(twai_message_t &message) {
  // static twai_message_t altmessage;
  static bool msgFlag = false;
  
  leds[0] = CRGB::Green;
  FastLED.show();
  // Process received message
  // if (message.extd) {
  //   Serial.println("Message is in Extended Format");
  // } else {
  //   Serial.println("Message is in Standard Format");
  // }
  if (message.data_length_code > 0) {
    Serial.printf("RECV ID: 0x%x Bytes:", message.identifier);
    if (!(message.rtr)) {
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.printf(" %d = 0x%02x", i, message.data[i]);
      }
      Serial.println("");
    }
  } else {
    Serial.printf("RECV ID: 0x%x\n", message.identifier);
  }


  switch (message.identifier) {
    case SW_SET_OFF:            // set output switch off
      setSwitchState(message.data, 0);
      break;
    case SW_SET_ON:             // set output switch on
      setSwitchState(message.data, 1);
      break;
    case SW_MOM_PRESS:          // set output momentary
      setSwitchState(message.data, 2);
      break;
    case SW_SET_MODE:           // setup output switch modes
      setSwitchMode(message.data);
      break;
    case SW_SET_PWM_DUTY:          // set output switch pwm duty
      setPWMDuty(message.data);  
      break;
    case SW_SET_PWM_FREQ:          // set output switch pwm frequency
      setPWMFreq(message.data);
      break;
    case SW_SET_MOM_DUR:          // set output switch momentary duration
      setSwMomDur(message.data);
      break;
    case SW_SET_BLINK_DELAY:          // set output switch blink delay
      setSwBlinkDelay(message.data);
      break;
    case SW_SET_STROBE_PAT:          // set output switch strobe pattern
      setSwStrobePat(message.data);
      break;
    case SET_DISPLAY_OFF:          // set display off
      setDisplayMode(message.data, 0); 
      break;
    case SET_DISPLAY_ON:          // set display on
      setDisplayMode(message.data, 1); 
      break;    
    case SET_DISPLAY_CLEAR:          // clear display
      setDisplayMode(message.data, 2); 
      break;
    case SET_DISPLAY_FLASH:          // flash display
      setDisplayMode(message.data, 3); 
      break;
    case REQ_INTERFACES:
      Serial.println("Interface intro request, responding with 0x702");
      FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      sendIntroduction(); // send our introduction message
      break;

    case ACK_INTRODUCTION:
      Serial.println("Received introduction acknowledgement, starting over!\n");    
      FLAG_SEND_INTRODUCTION = false; // stop sending introduction messages
      break;
    
    default:
      if ((message.identifier & MASK_INTERFACE) == INTRO_INTERFACE) { // received an interface introduction
        Serial.printf("Received introduction 0x%x\n", message.identifier);
        sendIntroack();
      }
  
      break;
  }

} // end of handle_rx_message
*/

/* 
void TaskTWAI(void *pvParameters) {
  // give some time at boot the cpu setup other parameters
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NO_ACK);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;


  for (;;) {
    if (!driver_installed) {
      // Driver not installed
      vTaskDelay(1000);
      return;
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      Serial.println("Alert: The Transmission failed.");
      Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
      Serial.printf("TX error: %d\t", twaistatus.tx_error_counter);
      Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
      leds[0] = CRGB::Green;
      FastLED.show();
      // Serial.println("Alert: The Transmission was successful.");
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      leds[0] = CRGB::Red;
      FastLED.show();
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      leds[0] = CRGB::Yellow;
      FastLED.show();
      // Serial.println("Testing line");
      // One or more messages received. Handle all.
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handle_rx_message(message);
      }
    }
    // Send message
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
      leds[0] = CRGB::Blue;
      FastLED.show();
      previousMillis = currentMillis;
      send_message(REQ_INTERFACES, NULL, 0); // send our introduction request
    }
    vTaskDelay(10);
  }
}

*/

uint8_t frmData[5][8] = {
    // arrays of tx msg data fields
    {0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff}, // [0][]
    {0x01, 0xa5, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0xff}, // [1][]
    {0x02, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7}, // [2][]
    {0x03, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}, // [3][]
    {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // [4][]
};

uint8_t frameIdx = 0; // index of tx frame[] to send. In this example it isn't useful as there is only one 'frame[1]'
uint8_t msgNum = 1;   // index of tx data to send   <<<<<<----------<<<<<<

void initFrame()
{
  int frmDataIdx = 0; // index of desired frmData[frmDataIdx]
  switch (msgNum)
  {
  case 0:
    frmDataIdx = 0;
    frame[frameIdx].txMsgID = 0x05;
    frame[frameIdx].idLen = STD_ID_LEN;
    frame[frameIdx].txDly = 500;
    frame[frameIdx].busConfig = PORTA_11_12_WIRE_PULLUP; //PORTB_8_9_WIRE_PULLUP;
    break;
  case 1:
    frmDataIdx = 1;
    frame[frameIdx].txMsgID = 0x7ff; //daf110; // 0x69;
    frame[frameIdx].idLen = STD_ID_LEN;
    frame[frameIdx].txDly = 1000;
    frame[frameIdx].busConfig = PORTA_11_12_XCVR; //PORTA_11_12_XCVR;
    break;
  case 2:
    frmDataIdx = 2;
    frame[frameIdx].txMsgID = 0x69;
    frame[frameIdx].idLen = STD_ID_LEN;
    frame[frameIdx].txDly = 1500;
    frame[frameIdx].busConfig = PORTB_8_9_XCVR;
    break;
  case 3:
    frmDataIdx = 3;
    frame[frameIdx].txMsgID = 0x69;
    frame[frameIdx].idLen = EXT_ID_LEN;
    frame[frameIdx].txDly = 4000;
    frame[frameIdx].busConfig = PORTB_8_9_XCVR;
    break;
  case 4:
    frmDataIdx = 4;
    frame[frameIdx].txMsgID = 0x00232461; //0x69;
    frame[frameIdx].idLen = EXT_ID_LEN;
    frame[frameIdx].txDly = 1100;
    frame[frameIdx].busConfig = PORTA_11_12_XCVR;
    break;

  default:
    break;
  }
  int mLen = sizeof(frmData[0]) / sizeof(frmData[0][0]);
  cpArray(frmData[frmDataIdx], frame[frameIdx].txMsg.bytes, mLen); // copy '8' values
}

char cBuff[46]; // holds formatted string

inline void canSend(msgFrm frm, bool prt = false) //send a CAN Bus message
{
  bool status = can.transmit(frm.txMsgID, frm.txMsg.bytes, frm.txMsgLen);
  if (prt)
  {
    if (status == false)
    {
      Serial.println("   *** TX ERROR ***");
      Serial.println("   mailbox not empty ");
    }
    else
    {
      if (can.getIDType())
        sprintf(cBuff, "tx @%08x #%u \t", frm.txMsgID, frm.txMsgLen);
      else
        sprintf(cBuff, "tx @%03x #%u \t", frm.txMsgID, frm.txMsgLen);
      Serial.print(cBuff);
      for (u_int8_t j = 0; j < frm.txMsgLen; j++)
      {
        sprintf(cBuff, "%02X ", frm.txMsg.bytes[j]);
        Serial.print(cBuff);
      }
      Serial.println();
    }
  }
}

inline void canRead(bool print = false) //check for a CAN Bus message
{
  //len = can.receive(id, fltIdx, rxMsg.bytes);  // comment out when using rx interrupt
  uint8_t cnt = can.getRxMsgFifo0Cnt();
  uint8_t full = can.getRxMsgFifo0Full();
  uint8_t ovr = can.getRxMsgFifo0Overflow();

  if (can.rxMsgLen >= 0)
  {
    if (print)
    {
      if (can.getRxIDType())
        sprintf(cBuff, "RX @%08x #%d  %d\t", can.id, can.rxMsgLen, can.fltIdx);
      else
        sprintf(cBuff, "RX @%03x #%d  %d\t", can.id, can.rxMsgLen, can.fltIdx);

      Serial.print(cBuff);
      for (uint8_t j = 0; j < can.rxMsgLen; j++)
      {
        sprintf(cBuff, "%02x ", can.rxData.bytes[j]);
        Serial.print(cBuff);
      }
      if (can.getRxMsgFifo0Cnt()) // check if loop is able to keep up with bus messages
      {
        sprintf(cBuff, " cnt %d, full %d, overflow %d ", cnt, full, ovr);
        Serial.print(cBuff);
      }
      Serial.println();
    }
    digitalToggle(PC13); // blink LED
    can.rxMsgLen = -1;
  }
  // return id;
}

void canISR() // get bus msg frame passed by a filter to FIFO0
{
  can.rxMsgLen = can.receive(can.id, can.fltIdx, can.rxData.bytes); // get CAN msg
}


void setup() {
  pinMode(PC13, OUTPUT); // blue pill LED
  delay(5000);

  can.begin(frame[frameIdx].idLen, BR500K, frame[frameIdx].busConfig); // CAN was constructed with user parms, this sets new parms
  can.setAutoTxRetry(true);                                            // CAN hw keeps sending last tx until someone ACK's it
  can.attachInterrupt(canISR);

/*   FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();
 */
  Serial.begin(115200);

}

int lastPending = 0;
uint32_t last = 0;
void loop()
{
  //----------------------------------tx-------------------/-------------
  if (millis() / frame[frameIdx].txDly != last)
  {
    last = millis() / frame[frameIdx].txDly;
    frame[0].txMsg.int32[1] = millis(); // last 4 bytes of CAN msg
    frame[0].txMsg.int32[0] = 0;        // first four bytes = 0
    // frame[frameIdx].txMsg.int64 = 0x0807060504030201;
    Serial.println();
    canSend(frame[frameIdx], true);
    Serial.println();
  }

  delay(120);
  // ----------------------------------rx----------------------------
  canRead(true);
  // IWatchdog.reload();
} // end of loop