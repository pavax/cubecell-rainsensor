#include "LoRaWan_APP.h"
#include <simple_logger.h>
#include "softSerial.h"
#include "credentials.h"

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
//    60000  ->   1 min
//   120000  ->   2 min
//   180000  ->   3 min
//   300000  ->   5 min
//   600000  ->  10 min
//   900000  ->  15 min
//  1200000  ->  20 min
//  1800000  ->  30 min
//  3600000  ->  60 min
uint32_t appTxDutyCycle = 3600000 * 12;  // every 12h

const uint32_t RAINING_INTERUPT_COOLDOWN_TIME = 180000;  // 3min -> while it's raining (aka the rain-interrupt is triggered), wake up at this interval

const uint32_t RAINING_FINISHED_TIME = 1200000 + RAINING_INTERUPT_COOLDOWN_TIME;  //  ~20min -> after the last drop detected check at this interval to wait for the eventAcc to reset

const uint32_t RAINING_FINISHED_MAX_TIME = 12 * RAINING_FINISHED_TIME;  //  ~4h -> max time to wait for the eventAcc to reset

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)* 4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

enum WakeupReason { NOTHING = 0,
                    USER = 1,
                    RAIN = 2,
                    DOWNLINK_COMMAND = 3 };

WakeupReason wakeupReason = NOTHING;

#define DEFAULT_LOG_LEVEL logger::Debug  // DEBUG: set to Debug for more logging statements or to None for no logs

#define WAKE_UP_PIN USER_KEY

#define RAIN_INTERUPT_PIN GPIO5

#define TX_PIN GPIO2

softSerial softwareSerial(TX_PIN /*TX pin*/, GPIO1 /*RX pin*/);

#define PATTERN "%*s %s %[^,] , %*s %s %*s %*s %s %*s %*s %s"

#define MAX_SENSOR_RETRY 3

boolean isRaining = false;

unsigned int uptimeCount, batteryVoltage, rainEventCounter;

unsigned int serialCode;

double acc, eventAcc, totalAcc, rInt;

unsigned long lastRainDetectionTime = 0;

unsigned long lastTxCycleTime = 0;

static const int MAX_DATA_LENGTH_BYTES = 80;

char buffer[MAX_DATA_LENGTH_BYTES];

char scheduledCommand = 0;

void blinkRGB(uint32_t color, int times = 3, int blinkTime = 500) {
  if (!LoRaWAN.isRgbEnabled()) {
    return;
  }
  for (int i = 0; i < times; i++) {
    turnOnRGB(color, blinkTime);
    turnOnRGB(0, blinkTime);
  }
}

void sendCommand(char command) {
  logger::debug(F("send command: %c"), command);
  softwareSerial.printf(";TEST\r\n%c\r\n", command);
  softwareSerial.flush();
}

boolean processSerialInput() {
  int len = 0;
  while (len = softwareSerial.readBytesUntil('\n', buffer, MAX_DATA_LENGTH_BYTES)) {
    buffer[len] = 0;
    logger::debug(buffer);
    if (strncmp(buffer, "Acc", 3) == 0 && strstr(buffer, "TotalAcc") != NULL) {
      processDataLine(buffer);
      return true;
    } else if (strncmp(buffer, "LensBad", 7) == 0) {
      logger::debug(F("Lense Bad Info Found (Code: 1)"));
      serialCode = 1;
    } else if (strncmp(buffer, "Reset", 5) == 0) {
      logger::debug(F("Rain Sensor was reset (Code: 2)"));
      serialCode = 2;
    } else if (strncmp(buffer, "PwrDays", 7) == 0) {
      logger::debug(F("Rain Sensor restarted (Code: 3)"));
      serialCode = 3;
    } else if (strncmp(buffer, "Event", 5) == 0) {
      logger::debug(F("Rain Event was detected (Code: 4)"));
      serialCode = 4;
      lastRainDetectionTime = millis();
    } else if (strncmp(buffer, "EmSat", 5) == 0) {
      logger::debug(F("EmSat detected"));
      serialCode = 5;
    }
  }
  return false;
}

void readRainsensor() {
  // reset data
  serialCode = 0;

  if (scheduledCommand != 0) {
    turnOnRGB(0x00FFFF, 500);  // cyan
    logger::info(F("Execute scheduled command: %c"), scheduledCommand);
    sendCommand(scheduledCommand);
    scheduledCommand = 0;
    delay(5000);
    turnOffRGB();
  }

  int retryCount = 0;
  bool successRead = false;
  while (!successRead && retryCount <= MAX_SENSOR_RETRY) {
    sendCommand('r');
    successRead = processSerialInput();
    if (successRead) {
      turnOnRGB(0xCD00FF, 500);  // purple
      break;
    }
    retryCount++;
    if (LoRaWAN.isRgbEnabled()) {
      blinkRGB(COLOR_SEND, 4, 250);  // blink red
      turnOnRGB(COLOR_SEND, 0);      // red
    } else {
      delay(500);
    }
  }

  if (!successRead) {
    logger::warn(F("Read Rain Sensor Timed-out"));
    serialCode = 99;
  } else {
    logger::info(F("Read Rain Sensor success"));
  }

  if ((millis() - lastRainDetectionTime >= RAINING_FINISHED_MAX_TIME) && isRaining) {
    logger::debug(F("No rain detected since a long time: disable serial"));
    isRaining = false;
    rainEventCounter = 0;
    serialCode = 88;
  }
}

void processDataLine(char dataLine[]) {
  char accB[7], eventAccB[7], totalAccB[7], rIntB[7], unit[4];
  sscanf(dataLine, PATTERN, &accB, &unit, &eventAccB, &totalAccB, &rIntB);
  if (strncmp(unit, "in", 2) == 0) {
    logger::warn(F("wrong unit - expected metric"));
    return;
  }

  acc = atof(accB);

  double currrentEventAcc = atof(eventAccB);
  if (eventAcc > 0 && currrentEventAcc == 0) {
    logger::debug(F("EventAcc was reset"));
    isRaining = false;
    rainEventCounter = 0;
  } else if (currrentEventAcc > 0) {
    isRaining = true;
  }

  eventAcc = currrentEventAcc;
  totalAcc = atof(totalAccB);
  rInt = atof(rIntB);
}

void setupRainsensor() {
  softwareSerial.begin(9600);
  softwareSerial.setTimeout(5000);
  delay(5000);  // wait for sensor to start up
  sendCommand('k');
  processSerialInput();
  softwareSerial.setTimeout(2000);
  sendCommand('p');
  sendCommand('l');
  sendCommand('m');
  processSerialInput();
}

static void prepareTxFrame(uint8_t port) {
  detachInterrupt(WAKE_UP_PIN);
  batteryVoltage = getBatteryVoltage();
  attachInterrupt(WAKE_UP_PIN, onUserWakeUp, RISING);

  //Serial.println(F("fetch rain sensor data"));
  logger::debug(F("fetch rain sensor data"));
  delay(10);
  readRainsensor();

  appDataSize = 20;

  // counter
  appData[0] = highByte(uptimeCount);
  appData[1] = lowByte(uptimeCount);

  logger::debug(F("UptimeCount: %d"), uptimeCount);

  // battery-voltate
  appData[2] = highByte(batteryVoltage);
  appData[3] = lowByte(batteryVoltage);
  logger::debug(F("BatteryVoltage: %d"), batteryVoltage);

  int tmp = acc * 100;
  appData[4] = highByte(tmp);
  appData[5] = lowByte(tmp);
  logger::debug(F("Acc: %d"), tmp);

  tmp = eventAcc * 100;
  appData[6] = highByte(tmp);
  appData[7] = lowByte(tmp);
  logger::debug(F("Event-Acc: %d"), tmp);

  tmp = totalAcc * 100;
  appData[8] = highByte(tmp);
  appData[9] = lowByte(tmp);
  logger::debug(F("Total-Acc: %d"), tmp);

  tmp = rInt * 100;
  appData[10] = highByte(tmp);
  appData[11] = lowByte(tmp);
  logger::debug(F("RInt: %d"), tmp);

  // serial code
  appData[12] = serialCode;
  logger::debug(F("Serial-Code: %d"), serialCode);

  // isRaining
  appData[13] = isRaining;
  logger::debug(F("Is-Raining Active: %d"), isRaining);

  // lastRainDetectionTime
  long lastRainAgo = (millis() - lastRainDetectionTime) / 1000;
  appData[14] = lastRainAgo >> 24;
  appData[15] = lastRainAgo >> 16;
  appData[16] = lastRainAgo >> 8;
  appData[17] = lastRainAgo & 0xFF;
  logger::debug(F("Last Rain detection: %d min ago"), (int)(lastRainAgo / 60.0));

  // rain detection counter
  appData[18] = highByte(rainEventCounter);
  appData[19] = lowByte(rainEventCounter);
  logger::debug(F("Rain Counter: %d"), rainEventCounter);

  uptimeCount++;
}

void onRainDetected() {
  if (deviceState == DEVICE_STATE_SLEEP) {
    if (millis() - lastTxCycleTime >= RAINING_INTERUPT_COOLDOWN_TIME) {
      logger::debug(F("Rain detected -> prepare next tx-cycle"));
      wakeupReason = RAIN;
    } else {
      logger::debug(F("Rain detected but it is too early for another tx-cycle"));
    }
    isRaining = true;
    rainEventCounter++;
    lastRainDetectionTime = millis();
    delay(50);
  }
}

void onUserWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH) {
    logger::debug(F("Woke up due to User Wake up Button Press press"));
    wakeupReason = USER;
    delay(50);
  }
}


void initManualRun() {
  logger::set_level(logger::Debug);
  LoRaWAN.enableRgb();
  turnOnRGB(0x005050, 500);
  turnOnRGB(0x002450, 500);
  turnOnRGB(0x000050, 500);
  turnOffRGB();
}

void prepareBeforeSleep() {
  if (!isTxConfirmed) {
    LoRaWAN.disableRgb();
    digitalWrite(Vext, HIGH);
  }
  if (isRaining) {
    logger::debug(F("Keep Serial enabled in low power mode"));
    digitalWrite(TX_PIN, HIGH);
  } else {
    logger::debug(F("Disable Serial in low power mode"));
    digitalWrite(TX_PIN, LOW);
  }
  logger::set_level(DEFAULT_LOG_LEVEL);
  delay(50);
}

void setup() {
  Serial.begin(115200);

  logger::set_serial(Serial);

  pinMode(Vext, OUTPUT);

  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onUserWakeUp, RISING);

  pinMode(RAIN_INTERUPT_PIN, INPUT_PULLUP);
  attachInterrupt(RAIN_INTERUPT_PIN, onRainDetected, RISING);

#if (AT_SUPPORT)
  enableAt();
#endif

  initManualRun();

  setupRainsensor();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop() {

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if (AT_SUPPORT)
        getDevParam();
#endif
        logger::debug(F("DEVICE_STATE_INIT"));
        printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        logger::debug(F("DEVICE_STATE_JOIN"));
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        logger::debug(F("DEVICE_STATE_SEND"));
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        logger::debug(F("DEVICE_STATE_CYCLE"));
        uint32_t cycleTimeToUse = appTxDutyCycle;
        if (isRaining) {
          cycleTimeToUse = RAINING_FINISHED_TIME;
        }
        txDutyCycleTime = cycleTimeToUse + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        logger::debug(F("Go to sleep for: %d sec"), (int)(txDutyCycleTime / 1000.0));
        prepareBeforeSleep();
        lastTxCycleTime = millis();
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (wakeupReason) {
          if (wakeupReason == USER) {
            initManualRun();
          } else if (wakeupReason == RAIN) {
            delay(1000);
          }
          logger::debug(F("Start Sending Cylcle due to wakeupReason: %d"), wakeupReason);
          LoRaWAN.txNextPacket();
          wakeupReason = NOTHING;
        } else {
          if (isTxConfirmed && LoRaWAN.hasReceivedAck()) {
            LoRaWAN.disableRgb();
            LoRaWAN.resetReceivedAck();
          }
          LoRaWAN.sleep();
        }
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  if (mcpsIndication->Port == 4) {
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    appTxDutyCycle = newSleepTime * 1000;
    saveDr();
    Serial.print(F("new DutyCycle received: "));
    Serial.print(appTxDutyCycle);
    Serial.println(F("ms"));
    delay(10);
  } else if (mcpsIndication->Port == 5) {
    char command = char(mcpsIndication->Buffer[0]);
    scheduledCommand = command;
    wakeupReason = DOWNLINK_COMMAND;
    Serial.print(F("scheduled command: "));
    Serial.println(scheduledCommand);
    delay(10);
  } else if (mcpsIndication->Port == 9) {
    Serial.println(F("Reset"));
    HW_Reset(0);
  }
}
