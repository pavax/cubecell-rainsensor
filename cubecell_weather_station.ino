#include "LoRaWan_APP.h"
#include "HT_SH1107Wire.h"
#include <simple_logger.h>
#include "credentials.h"

extern SH1107Wire display;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
//    60000  ->   1 min
//   120000  ->   2 min
//   300000  ->   5 min
//   600000  ->  10 min
//   900000  ->  15 min
//  1200000  ->  20 min
//  1800000  ->  30 min
//  3600000  ->  60 min
uint32_t appTxDutyCycle = 3600000 * 12; // every 12h

const uint32_t RAINING_INTERUPT_COOLDOWN_TIME = 300000;   // 5 min  -> while it's raining (aka the rain-interrupt is triggered), wake up at this interval

const uint32_t RAINING_FINISHED_TIME = 1210000;           // 20 min -> after the last drop detected check at this interval to wait for the eventAcc to reset

const uint32_t RAINING_FINISHED_MAX_TIME = 3 * 3600000;   // 3h -> max time to wait for the eventAcc to reset

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

enum WakeupReason { NOTHING = 0, USER = 1, RAIN = 2, DOWNLINK_COMMAND = 3};

WakeupReason wakeupReason = NOTHING;

#define DEFAULT_LOG_LEVEL logger::None // DEBUG: set to Debug for more logging statements or to None for no logs

#define WAKE_UP_PIN USER_KEY

#define RAIN_INTERUPT_PIN GPIO14

#define KEEP_SERIAL_ACTIVE_PIN GPIO12

#define PATTERN "%*s %s %[^,] , %*s %s %*s %*s %s %*s %*s %s"

boolean keepSerialActive = false;

unsigned int uptimeCount, batteryVoltage;

unsigned int serialCode;

double acc, eventAcc, totalAcc, rInt;

unsigned long lastRainDetectionTime = 0;

unsigned long lastTxCycleTime = 0;

static const int MAX_DATA_LENGTH_BYTES = 80;

char buffer[MAX_DATA_LENGTH_BYTES];

boolean rainEventStartedDetected = false;

char scheduledCommand = 0;

void sendCommand(char command) {
  logger::debug(F("send command: %c\r\n"), command);
  // it seems like the first command after serial is restarted is swallowed.
  // Serial1.printf("%c\r\n", command);
  // as a workarround send a dummy-command before the actual command
  Serial1.printf(";TESTCOMMAND\r\n%c\r\n", command);
}

boolean processSerialInput() {
  int len = 0;
  while (len = Serial1.readBytesUntil('\n', buffer, MAX_DATA_LENGTH_BYTES)) {
    buffer[len] = 0;
    logger::debug(buffer);
    if (strncmp(buffer, "Acc", 3) == 0 && strstr(buffer, "TotalAcc") != NULL) {
      processDataLine(buffer);
      return true;
    } else if (strncmp(buffer, "LensBad", 7) == 0) {
      logger::debug(F("Lense Bad Info Found"));
      serialCode = 1;
    } else if (strncmp(buffer, "Reset", 5) == 0) {
      logger::debug(F("Rain Sensor was reset"));
      serialCode = 2;
    } else if (strncmp(buffer, "PwrDays", 7) == 0) {
      logger::debug(F("Rain Sensor restarted"));
      serialCode = 3;
    } else if (strncmp(buffer, "Event", 5) == 0) {
      logger::debug(F("Rain Event was detected"));
      serialCode = 4;
      rainEventStartedDetected = true;
      keepSerialActive = true;
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
  rainEventStartedDetected = false;

  // TODO: Not sure if we can delete the following block
  if (Serial1.available() && processSerialInput()) {
    logger::debug(F("Read Rain Sensor success: (Code: 1)"));
    return;
  }

  if (scheduledCommand != 0) {
    logger::info(F("Execute scheduled command: %c"), scheduledCommand);
    sendCommand(scheduledCommand);
    scheduledCommand = 0;
    delay(5000);
  }

  sendCommand('r');
  if (processSerialInput()) {
    logger::info(F("Read Rain Sensor success: (Code: 2)"));
  } else {
    serialCode = 99;
    logger::warn(F("Read Rain Sensor Timed-out"));
  }

}

void processDataLine(char dataLine[]) {
  char accB[7], eventAccB[7], totalAccB[7], rIntB[7], unit[4];
  sscanf(dataLine, PATTERN, &accB, &unit, &eventAccB, &totalAccB, &rIntB);
  boolean metric = !(unit[0] == 'i' && unit[1] == 'n');
  if (!metric) {
    logger::warn(F("wrong unit - expected metric"));
    return;
  }

  acc = atof(accB);

  double currrentEventAcc = atof(eventAccB);
  if (eventAcc > 0 && currrentEventAcc == 0) {
    logger::debug(F("EventAcc was reset"));
    keepSerialActive = false;
  } else if (!rainEventStartedDetected && currrentEventAcc == 0) {
    logger::debug(F("Rain Event Line was not detected and EventAcc is 0.00"));
    keepSerialActive = false;
  }
  eventAcc = currrentEventAcc;

  totalAcc = atof(totalAccB);

  rInt = atof(rIntB);
}

void setupRainsensor() {
  Serial1.begin(9600);
  Serial1.setTimeout(5000);
  delay(5000);
  sendCommand('k');
  processSerialInput();
  sendCommand('p');
  sendCommand('l');
  sendCommand('m');
  processSerialInput();
  Serial1.setTimeout(2000);
}

static void prepareTxFrame( uint8_t port ) {
  detachInterrupt(WAKE_UP_PIN);
  batteryVoltage = getBatteryVoltage();
  attachInterrupt(WAKE_UP_PIN, onUserWakeUp, RISING);

  //Serial.println(F("fetch rain sensor data"));
  logger::debug(F("fetch rain sensor data"));
  delay(10);
  readRainsensor();

  if ((millis() - lastRainDetectionTime >= RAINING_FINISHED_MAX_TIME) && keepSerialActive) {
    logger::debug(F("No rain detected since a long time: disable serial"));
    keepSerialActive = false;
    serialCode = 88;
  }

  appDataSize = 18;

  // counter
  appData[0] = highByte(uptimeCount);
  appData[1] = lowByte(uptimeCount);

  logger::debug(F("UptimeCount: %d"), uptimeCount);

  // battery-voltate
  appData[2] = highByte(batteryVoltage);
  appData[3] = lowByte(batteryVoltage);
  logger::debug(F("BatteryVoltage: %d"), batteryVoltage);

  int temp = acc * 100;
  appData[4] = highByte(temp);
  appData[5] = lowByte(temp);
  logger::debug(F("Acc: %d"), temp);

  temp = eventAcc * 100;
  appData[6] = highByte(temp);
  appData[7] = lowByte(temp);
  logger::debug(F("Event-Acc: %d"), temp);

  temp = totalAcc * 100;
  appData[8] = highByte(temp);
  appData[9] = lowByte(temp);
  logger::debug(F("Total-Acc: %d"), temp);

  temp = rInt * 100;
  appData[10] = highByte(temp);
  appData[11] = lowByte(temp);
  logger::debug(F("RInt: %d"), temp);

  // serial code
  appData[12] = serialCode;
  logger::debug(F("Serial-Code: %d"), serialCode);

  // keepSerialActive
  appData[13] = keepSerialActive;
  logger::debug(F("Keep-Serial Active: %d"), keepSerialActive);

  // lastRainDetectionTime
  long lastRainAgo =  (millis() - lastRainDetectionTime ) / 1000;
  appData[14] = lastRainAgo >> 24;
  appData[15] = lastRainAgo >> 16;
  appData[16] = lastRainAgo >> 8;
  appData[17] = lastRainAgo & 0xFF;
  logger::debug(F("Last Rain detection: %d min ago"), (int) (lastRainAgo / 60.0));

  if (LoRaWAN.isDisplayEnabled()) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.clear();

    sprintf(buffer, "Acc: %d [mm]", (int)(acc + 0.5));
    display.drawString(64, 0, buffer);

    sprintf(buffer, "Event-Acc: %d [mm]", (int)(eventAcc + 0.5));
    display.drawString(64, 15, buffer);

    sprintf(buffer, "Total-Acc: %d [mm]", (int)(totalAcc + 0.5));
    display.drawString(64, 30, buffer);

    sprintf(buffer, "RInt: %d [mmph]", (int)(rInt + 0.5));
    display.drawString(64, 45, buffer);

    display.display();
    delay(2000);
    display.clear();

    sprintf(buffer, "Serial-Code: %d", serialCode);
    display.drawString(64, 0, buffer);

    sprintf(buffer, "Last-Rain: %d [min]", (int) (lastRainAgo / 60.0));
    display.drawString(64, 15, buffer);

    sprintf(buffer, "Serial-Active: %d", keepSerialActive);
    display.drawString(64, 30, buffer);

    sprintf(buffer, "Batt: %d [mV]", batteryVoltage);
    display.drawString(64, 45, buffer);

    display.display();
    delay(2000);
  }
  uptimeCount ++;
}

void displayUpTimeCount() {
  if (LoRaWAN.isDisplayEnabled()) {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(58, 5, F("Daten messen..."));
    display.drawHorizontalLine(0, 24, 128);
    display.setFont(ArialMT_Plain_16);
    sprintf(buffer, "%d", uptimeCount);
    display.drawString(58, 33, buffer);
    display.display();
    delay(1000);
  }
}

void initDisplay(boolean showText = false) {
  digitalWrite(Vext, LOW);
  delay(50);
  LoRaWAN.enableDisplay();
  LoRaWAN.enableRgb();
  if (LoRaWAN.isDisplayEnabled()) {
    //display.screenRotate(ANGLE_180_DEGREE);
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    if (showText) {
      display.clear();
      display.drawString(58, 12, F("Weather Station"));
      display.setFont(ArialMT_Plain_10);
      display.drawHorizontalLine(0, 33, 128);
      display.drawString(58, 40, F("Version 1.0"));
      display.drawString(58, 52, F("(c) Patrick Dobler"));
      display.display();
      delay(3000);
    }
  }
}

void onRainDetected() {
  if (deviceState == DEVICE_STATE_SLEEP ) {
    if (millis() - lastTxCycleTime >= RAINING_INTERUPT_COOLDOWN_TIME) {
      logger::debug(F("Rain detected -> prepare next tx-cycle"));
      wakeupReason = RAIN;
    } else {
      logger::debug(F("Rain detected but it is too early for another tx-cycle"));
    }
    keepSerialActive = true;
    lastRainDetectionTime = millis();
  }
}

void onUserWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH ) {
    logger::debug(F("Woke up due to User Wake up Button Press press"));
    wakeupReason = USER;
    delay(10);
  }
}

void prepareBeforeSleep() {
  if (isTxConfirmed == false) {
    LoRaWAN.disableRgb();
    LoRaWAN.disableDisplay();
    digitalWrite(Vext, HIGH);
    delay(50);
  }
  if (keepSerialActive) {
    logger::debug(F("Enable Serial in low power mode"));
    digitalWrite(KEEP_SERIAL_ACTIVE_PIN, HIGH);
  } else {
    logger::debug(F("Disable Serial in low power mode"));
    digitalWrite(KEEP_SERIAL_ACTIVE_PIN, LOW);
  }
  logger::set_level(DEFAULT_LOG_LEVEL);
  delay(20);
}

void setup() {
  Serial.begin(115200);

  logger::set_serial(Serial);
  logger::set_level(logger::Debug);

  pinMode(Vext, OUTPUT);

  pinMode(KEEP_SERIAL_ACTIVE_PIN, OUTPUT);
  digitalWrite(KEEP_SERIAL_ACTIVE_PIN, LOW);

  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onUserWakeUp, RISING);

  pinMode(RAIN_INTERUPT_PIN, INPUT_PULLUP);
  attachInterrupt(RAIN_INTERUPT_PIN, onRainDetected, RISING);

#if(AT_SUPPORT)
  enableAt();
#endif

  initDisplay(true);

  setupRainsensor();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop() {

  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#if(LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
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
        LoRaWAN.displayJoining();
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        logger::debug(F("DEVICE_STATE_SEND"));
        digitalWrite(KEEP_SERIAL_ACTIVE_PIN, LOW);
        displayUpTimeCount();
        prepareTxFrame( appPort );
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        logger::debug(F("DEVICE_STATE_CYCLE"));
        uint32_t cycleTimeToUse = appTxDutyCycle;
        if (keepSerialActive) {
          cycleTimeToUse = RAINING_FINISHED_TIME;
        }
        txDutyCycleTime = cycleTimeToUse + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        logger::debug(F("Go to sleep for: %d sec"), (int) (txDutyCycleTime / 1000.0));
        delay(10);
        prepareBeforeSleep();
        lastTxCycleTime = millis();
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (wakeupReason) {
          if (wakeupReason == USER) {
            logger::set_level(logger::Debug);
            initDisplay();
          } else if (wakeupReason == RAIN) {
            delay(1000);
          }
          logger::debug(F("Start Sending Cylcle due to wakeupReason: %d"), wakeupReason);
          LoRaWAN.txNextPacket();
          wakeupReason = NOTHING;
        } else {
          LoRaWAN.displayAck();
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
    appTxDutyCycle  = newSleepTime * 1000;
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
  }
}
