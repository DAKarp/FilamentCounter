#include "config.h"
#include "bitmaps.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <time.h>           // for time calculations
#include <WiFi.h>           // for wifi
#include <WiFiUdp.h>        // for udp via wifi
#include <U8g2lib.h>        // see https://github.com/olikraus/u8g2/wiki/u8g2reference
#include <HTTPClient.h>     // mechanism to send http get requests for prowl notifications
#include <elapsedMillis.h>  // for various timeouts
#include <EEPROM.h>         // to persist count
#include <ESPmDNS.h>        // so you don't need a static IP address for OTA

// Constants

#define NTP_DELAY_COUNT   20   // delay count for ntp update
#define NTP_PACKET_LENGTH 48 // ntp packet length
#define UDP_LISTEN_PORT   4000

#define FONT_6x10_HEIGHT  8    // font height in pixels
#define YPOS_CLOCK        1
#define YPOS_STATUS       16
#define YPOS_COUNT        30

#define PROWL_URL "https://api.prowlapp.com/publicapi/add?apikey=%s&application=%s&event=%s"

#define WIFI_TIMEOUT_MILLIS     10000
#define DEBOUNCE_TIME_MILLIS    10
#define ZERO_BUTTON_HOLD_MILLIS 3000

#define BUFFER_SIZE 128

// Variables

elapsedMillis wifiTimer;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t zeroButtonDebounceTimeout = 0;
volatile int zeroButtonInterruptTriggered = 0;
volatile bool zeroButtonPriorState;

char chBuffer[BUFFER_SIZE];                                   // general purpose character buffer
bool bTimeReceived = false;                                   // time has not been received
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(rotate180 ? U8G2_R2 : U8G2_R0, U8X8_PIN_NONE, OLED_CLOCK, OLED_DATA); // OLED graphics
int nWifiStatus = WL_IDLE_STATUS;                             // wifi status
WiFiUDP Udp;
long priorTime = 0;

long position = 0;
long priorPosition = 0;
long positionWhenStarted = 0;

time_t currentStopSessionBegan = 0;
time_t currentMovementSessionBegan = 0;
time_t movementMostRecentlyDetected = 0;

time_t timeStoppedConfirmed = 0;
time_t timeStartedConfirmed = 0;

enum encoderState {
  stopped,
  feeding,
  retracting
};
encoderState priorState = stopped;

char rfc3986[256] = {0};
char html5[256] = {0};

TaskHandle_t backgroundTaskHandle;

#define NOTIFICATION_DATA_BUFFER_SIZE  256
struct notificationData {
  char major[NOTIFICATION_DATA_BUFFER_SIZE];
  char minor[NOTIFICATION_DATA_BUFFER_SIZE];
  bool queued = false;
};
notificationData queuedData[5];

// functions

void persistPosition() {
  EEPROM.begin(sizeof(position));
  EEPROM.put(0, position);
  EEPROM.commit();
}

void recoverPosition() {
  long persistedPosition;
  EEPROM.begin(sizeof(persistedPosition));
  EEPROM.get(0, persistedPosition);
  position = persistedPosition;
}

void reboot() {
  persistPosition();
  ESP.restart();
}

void setUpSerial() {
  Serial.begin(9600);
  while (!Serial) {
    Serial.print('.');
  }
}

void setUpOLED() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void setUpWiFi() {
  wifiTimer = 0;

  // reset esp32 wifi
  // https://github.com/espressif/arduino-esp32/issues/1212
  Serial.printf("WIFI status = %i\n", WiFi.getMode());
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.printf("WIFI status = %i\n", WiFi.getMode());

  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  // show "connecting to" on OLED
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_6x10_tr);
  sprintf(chBuffer, "%s", "Connecting to:");
  u8g2.drawStr((u8g2.getDisplayWidth() / 2) - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);
  sprintf(chBuffer, "%s", chSSID);
  u8g2.drawStr((u8g2.getDisplayWidth() / 2) - (u8g2.getStrWidth(chBuffer) / 2), 31 - (FONT_6x10_HEIGHT / 2), chBuffer);
  u8g2.sendBuffer();

  // connect
  Serial.print("connecting to wifi");
  WiFi.begin(chSSID, chPassword);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    if (wifiTimer > WIFI_TIMEOUT_MILLIS) {
      // not connected yet - reboot and try again - uncomment if your board has trouble connecting reliably
      // reboot();
    }
  }

  Serial.printf("\n\nWiFi connected to %s\n", chSSID);

  // Display successful connection status on OLED
  u8g2.clearBuffer();
  sprintf(chBuffer, "%s", "WiFi Status:");
  u8g2.drawStr((u8g2.getDisplayWidth() / 2) - (u8g2.getStrWidth(chBuffer) / 2), 0, chBuffer);

  char chIp[BUFFER_SIZE];
  WiFi.localIP().toString().toCharArray(chIp, BUFFER_SIZE - 1);
  Serial.printf("IP Address: %s\n", chIp);
  sprintf(chBuffer, "IP: %s", chIp);
  u8g2.drawStr(0, FONT_6x10_HEIGHT * 2, chBuffer);

  sprintf(chBuffer, "SSID: %s", chSSID);
  u8g2.drawStr(0, FONT_6x10_HEIGHT * 3, chBuffer);

  sprintf(chBuffer, "RSSI: %d", WiFi.RSSI());
  u8g2.drawStr(0, FONT_6x10_HEIGHT * 4, chBuffer);

  u8g2.drawStr(0, FONT_6x10_HEIGHT * 6, "Obtaining NTP time...");

  u8g2.sendBuffer();
}

void setUpMDNS() {
	boolean success = MDNS.begin(mdnsName);
	Serial.print("mDNS responder: ");
	if (success) {
		MDNS.addService("workstation", "tcp", 80);
		Serial.printf("added as %s.local\n", mdnsName);
	} else {
		Serial.println("failed");
	}
}

void otaStatus(char* status) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, YPOS_CLOCK, status);
  u8g2.sendBuffer();
}

void setUpOTA() {
	ArduinoOTA.setHostname(otaHostname);
	ArduinoOTA.setPassword(otaPassword);

	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    otaStatus(chBuffer);
	});
	ArduinoOTA.onEnd([]() {
    otaStatus((char *)"OTA End");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    sprintf(chBuffer, "OTA Progress: %u%%\r", (progress / (total / 100)));
    otaStatus(chBuffer);
	});
	ArduinoOTA.onError([](ota_error_t error) {
		if (error == OTA_AUTH_ERROR)
      sprintf(chBuffer, "OTA Error: Auth Failed (%i)", error);
		else if (error == OTA_BEGIN_ERROR)
      sprintf(chBuffer, "OTA Error: Begin Failed (%i)", error);
		else if (error == OTA_CONNECT_ERROR)
      sprintf(chBuffer, "OTA Error: Connect Failed (%i)", error);
		else if (error == OTA_RECEIVE_ERROR)
      sprintf(chBuffer, "OTA Error: Receive Failed (%i)", error);
		else if (error == OTA_END_ERROR)
      sprintf(chBuffer, "OTA Error: End Failed (%i)", error);
    otaStatus(chBuffer);
	});

	ArduinoOTA.begin();
	Serial.printf("OTA ready at %s\n", WiFi.localIP().toString().c_str());
}

void setUpUDP() {
  Udp.begin(UDP_LISTEN_PORT);
}

void getTime() {
  static int nNtpDelay = NTP_DELAY_COUNT;
  static byte chNtpPacket[NTP_PACKET_LENGTH];

  nNtpDelay += 1;
  if (nNtpDelay >= NTP_DELAY_COUNT) {
    nNtpDelay = 0;

    memset(chNtpPacket, 0, NTP_PACKET_LENGTH);    // Zero out chNtpPacket.

    // Set the ll (leap indicator = 0), vvv (version number = 3) and mmm (mode = 3) bits. These bits are
    // contained in the first byte of chNtpPacker and are in the following format:  llvvvmmm

    chNtpPacket[0] = 0b00011011;

    // Send the ntp packet.

    IPAddress ipNtpServer(129, 6, 15, 28);
    Udp.beginPacket(ipNtpServer, 123);
    Udp.write(chNtpPacket, NTP_PACKET_LENGTH);
    Udp.endPacket();

    Serial.println("ntp packet sent to ntp server.");
    Serial.println("awaiting response from ntp server");
  }

  Serial.print(".");

  if (nNtpDelay == (NTP_DELAY_COUNT - 1)) {
    // Time to check for a server response

    if (Udp.parsePacket()) {
      // Server responded, read the packet
      Udp.read(chNtpPacket, NTP_PACKET_LENGTH);

      // Obtain the time from the packet, convert to Unix time, and adjust for the time zone
      struct timeval currentTime = {0, 0};

      currentTime.tv_sec = ((unsigned long)chNtpPacket[40] << 24) + // bits 24 through 31 of ntp time
                           ((unsigned long)chNtpPacket[41] << 16) + // bits 16 through 23 of ntp time
                           ((unsigned long)chNtpPacket[42] << 8) +  // bits  8 through 15 of ntp time
                           ((unsigned long)chNtpPacket[43]) -       // bits  0 through  7 of ntp time
                           (((70UL * 365UL) + 17UL) * 86400UL) +    // ntp to unix conversion
                           (TIME_ZONE * 3600UL) +                   // time zone adjustment
                           (5);                                     // transport delay fudge factor

      // Set the ESP32 rtc.
      settimeofday(&currentTime, NULL);
      bTimeReceived = true;

      // Output date and time
      struct tm *localTimeComponents = localtime(&currentTime.tv_sec);
      strftime(chBuffer, BUFFER_SIZE, "%a, %d %b %Y %H:%M:%S", localTimeComponents);
      Serial.println();
      Serial.print("response received, time written to ESP32 rtc: ");
      Serial.println(chBuffer);

    } else {
      // Server did not respond.
      Serial.println("NTP packet not received.");
    }
  }
}

void IRAM_ATTR encoderPinDidChange() {
  // If noise is suspected, try this:
  // https://www.best-microcontroller-projects.com/rotary-encoder.html#Taming_Noisy_Rotary_Encoders

  if (digitalRead(ENCODER_PIN_1) == digitalRead(ENCODER_PIN_2)) {
    position += 1;
  } else {
    position -= 1;
  }
}

void IRAM_ATTR zeroButtonPressed() {
  // https://www.switchdoc.com/2018/04/esp32-tutorial-debouncing-a-button-press-using-interrupts/

  portENTER_CRITICAL_ISR(&mux); // critical block - disable interrupts for quasi thread safety
  zeroButtonInterruptTriggered++;
  zeroButtonPriorState = digitalRead(ZERO_BTN_PIN);
  zeroButtonDebounceTimeout = xTaskGetTickCount(); //version of millis() that works from interrupt
  portEXIT_CRITICAL_ISR(&mux); // end of critical block
}

void zeroButtonTask(void * parameter) {
  Serial.printf("\nzero button monitor task running on core %i\n", xPortGetCoreID());

  bool triggeredAtLeastOnce;
  uint32_t savedTimeout;
  bool savedState;
  uint32_t buttonDownMillis = 0;

  // enter RTOS Task Loop
  while (1) {
    portENTER_CRITICAL_ISR(&mux); // critical block - disable interrupts for quasi thread safety
    triggeredAtLeastOnce = zeroButtonInterruptTriggered != 0;
    savedTimeout = zeroButtonDebounceTimeout;
    savedState = zeroButtonPriorState;
    portEXIT_CRITICAL_ISR(&mux); // end of critical block

    bool currentState = digitalRead(ZERO_BTN_PIN);

    if (triggeredAtLeastOnce
        && (currentState == savedState) // pin is still in the same state as when intr triggered
        && (millis() - savedTimeout > DEBOUNCE_TIME_MILLIS )) {
      
      if (currentState == LOW) {
        Serial.println("Zero button down");
        buttonDownMillis = millis();

      } else {
        if (millis() - buttonDownMillis > ZERO_BUTTON_HOLD_MILLIS) {
          Serial.println("Zero button released after long press");

          vTaskDelay(1000 / portTICK_PERIOD_MS);
          reboot();

        } else {
          Serial.println("Zero button released after short press");

          portENTER_CRITICAL_ISR(&mux); // critical block - disable interrupts for quasi thread safety
          position = 0;
          portEXIT_CRITICAL_ISR(&mux); // end of critical block
        }
      }
            
      portENTER_CRITICAL_ISR(&mux); // critical block - disable interrupts for quasi thread safety
      zeroButtonInterruptTriggered = 0; // reset interrupt counter
      portEXIT_CRITICAL_ISR(&mux); // end of critical block

      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setUpPins() {
  pinMode(ENCODER_PIN_1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2, INPUT_PULLUP);
  pinMode(ZERO_BTN_PIN, INPUT_PULLUP);

  attachInterrupt(ENCODER_PIN_1, encoderPinDidChange, CHANGE);
  attachInterrupt(ZERO_BTN_PIN, zeroButtonPressed, CHANGE);
}

char *urlEncode(char *table, const char *s, char *enc) {
  for (; *s; s++) {
    if (table[(unsigned int)*s])
      sprintf(enc, "%c", table[(unsigned int)*s]);
    else
      sprintf(enc, "%%%02X", *s);
    while (*++enc)
      ;
  }
  return (enc);
}

void sendNotificationBackgroundTask(void *dataPointer) {

  char majorEncoded[NOTIFICATION_DATA_BUFFER_SIZE * 3]; // prowl "application"
  char minorEncoded[NOTIFICATION_DATA_BUFFER_SIZE * 3]; // prowl "event"

  // enter RTOS Task Loop
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(3000 / portTICK_RATE_MS);
      continue;
    }

    int queuedSlot = -1;
    for (size_t i = 0; i < 5; i++) {
      if (queuedData[i].queued) {
        // process this filled slot
        queuedSlot = i;
      }
    }

    if (queuedSlot >= 0) {
      // found a queued notification - send it now
      urlEncode(rfc3986, queuedData[queuedSlot].major, majorEncoded);
      urlEncode(rfc3986, queuedData[queuedSlot].minor, minorEncoded);

      char url[256];
      sprintf(url, PROWL_URL, prowlKey, majorEncoded, minorEncoded);

      HTTPClient http;
      http.begin(url);
      int responseCode = http.GET();

      if (responseCode > 0) {
        Serial.println("Notification sent");

        // success - remove it from queue
        queuedData[queuedSlot].major[0] = 0;
        queuedData[queuedSlot].minor[0] = 0;
        queuedData[queuedSlot].queued = false;

      } else {
        Serial.println("Error sending notification");

        // failure - leave it in queue
      }
      Serial.println(url);
    }

    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void setUpBackgroundTasks() {
  xTaskCreate(zeroButtonTask, "zeroButtonTask", /* stack size: */ 10000, NULL, /* priority: */ 1, NULL);
  xTaskCreate(sendNotificationBackgroundTask, "sendNotificationBackgroundTask", /* stack size: */ 20000, NULL, /* priority: */ 1, NULL);
}

void setUpRFCTables() {
  int i;
  for (i = 0; i < 256; i++) {
    rfc3986[i] = isalnum(i) || i == '~' || i == '-' || i == '.' || i == '_' ? i : 0;
    html5[i] = isalnum(i) || i == '*' || i == '-' || i == '.' || i == '_' ? i : (i == ' ') ? '+' : 0;
  }
}

void setup() {
  recoverPosition();
  setUpSerial();
  setUpOLED();
  setUpPins();
  setUpBackgroundTasks();
  setUpRFCTables();

  setUpWiFi();
  setUpMDNS();
  setUpOTA();
  setUpUDP();
}

float correctedPosition() {
  return (float)position * (wheelRadius / encoderPPR);
}

void sendNotification(const char* major, const char* minor) {

  int slotUsed = -1;
  for (size_t i = 0; i < 5; i++) {
    if (! queuedData[i].queued) {
      // use this empty slot
      slotUsed = i;
    }
  }

  if (slotUsed == -1) {
    Serial.println("Notification queue is full - skipping");
    
  } else {
      strcpy(queuedData[slotUsed].major, major);
      strcpy(queuedData[slotUsed].minor, minor);
      queuedData[slotUsed].queued = true;
  }
}

void confirmStopped() {
  // bail if this session already confirmed or if movement has never started
  if (timeStoppedConfirmed == currentStopSessionBegan || timeStartedConfirmed == 0) return;

  timeStoppedConfirmed = currentStopSessionBegan;

  struct tm *components = localtime(&timeStoppedConfirmed);
  strftime(chBuffer, BUFFER_SIZE, "%l:%M:%S %p", components);
  Serial.printf("Stopped at %s\n", chBuffer);

  char major[BUFFER_SIZE], minor[BUFFER_SIZE], timeBuffer[BUFFER_SIZE];

  time_t elapsed = timeStoppedConfirmed - timeStartedConfirmed;
  struct tm *elapsedComponents = localtime(&elapsed);
  elapsedComponents->tm_yday--;
  strftime(timeBuffer, BUFFER_SIZE, (elapsedComponents->tm_yday > 0) ? "%jd %Hh %Mm %Ss" : "%Hh %Mm %Ss", elapsedComponents);

  sprintf(major, "Print Complete at %s", chBuffer);
  sprintf(minor, "Printing time: %s, %d mm used", timeBuffer, (int)correctedPosition());

  sendNotification(major, minor);
}

void confirmStarted() {
  // bail if this session already confirmed
  if (timeStartedConfirmed == currentMovementSessionBegan) return;

  timeStartedConfirmed = currentMovementSessionBegan;
  position -= positionWhenStarted; // reset position as though it had been zero'd at the start time

  struct tm *components = localtime(&timeStartedConfirmed);
  strftime(chBuffer, BUFFER_SIZE, "%l:%M:%S %p", components);
  Serial.printf("Started at %s\n", chBuffer);

  char minor[BUFFER_SIZE];
  sprintf(minor, "Started at %s", chBuffer);

  sendNotification("3D Printer started printing", minor);
}

void drawBitmap(const uint8_t *bitmap) {
  u8g2.drawXBM(u8g2.getDisplayWidth() - BITMAP_WIDTH, u8g2.getDisplayHeight() - BITMAP_HEIGHT, BITMAP_WIDTH, BITMAP_HEIGHT, bitmap);
}

void drawWheel(int i) {
  switch (i) {
  case 0:
    drawBitmap(bitmap0);
    break;
  case 1:
    drawBitmap(bitmap1);
    break;
  case 2:
    drawBitmap(bitmap2);
    break;
  case 3:
    drawBitmap(bitmap3);
    break;
  case 4:
    drawBitmap(bitmap4);
    break;
  case 5:
    drawBitmap(bitmap5);
    break;

  default:
    break;
  }
}

int modulo(int a, int b) {
  int r = a % b;
  return r < 0 ? r + b : r;
}

void loop() {
  
  if (WiFi.status() != WL_CONNECTED) {
    setUpWiFi();
    setUpUDP();
    return;
  }

	ArduinoOTA.handle();

  if (!bTimeReceived) {
    getTime();
    return;
  }

  // get time
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  struct tm *localTimeComponents = localtime(&currentTime.tv_sec);

  // let's see what the rotary encoder is up to
  encoderState state = stopped;
  if (position > priorPosition) {
    state = feeding;
  } else if (position < priorPosition) {
    state = retracting;
  }

  if (state == priorState &&
      state == stopped) {

    if (currentStopSessionBegan == 0) {
      currentStopSessionBegan = currentTime.tv_sec;

    } else if (currentTime.tv_sec - movementMostRecentlyDetected >= minStoppedSession) {
      // stopped for real
      currentMovementSessionBegan = 0;
      confirmStopped();
    }
  }

  if (state != stopped) {
    movementMostRecentlyDetected = currentTime.tv_sec;
    if (currentMovementSessionBegan == 0) {
      currentMovementSessionBegan = currentTime.tv_sec;
      positionWhenStarted = position;
      
    } else if (currentTime.tv_sec >= currentMovementSessionBegan + minMovementDetected) {
      // looks like it's really moving
      currentStopSessionBegan = 0;
      confirmStarted();
    }
  }

  priorPosition = position;
  priorState = state;

  // update display  
  if (currentTime.tv_sec != priorTime || state != stopped) {
    priorTime = currentTime.tv_sec;

    // clear display
    u8g2.clearBuffer();

    // date
    strftime(chBuffer, BUFFER_SIZE, "%a, %b %d", localTimeComponents);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, YPOS_CLOCK, chBuffer);

    // time
  
    strftime(chBuffer, BUFFER_SIZE, "%l:%M:%S", localTimeComponents);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(u8g2.getDisplayWidth() - u8g2.getStrWidth(chBuffer), YPOS_CLOCK, chBuffer);

    // status
    u8g2.setFont(u8g2_font_6x10_tr);
    switch (state) {
    case stopped:
      u8g2.drawStr(0, YPOS_STATUS, "stopped");
      break;
    case feeding:
      u8g2.drawStr(0, YPOS_STATUS, "feeding");
      break;
    case retracting:
      u8g2.drawStr(0, YPOS_STATUS, "retracting");
      break;
    
    }

    // wheel

    drawWheel(modulo(position, BITMAP_COUNT));

    // count

    sprintf(chBuffer, "%d mm", (int)correctedPosition());
    if (u8g2.getStrWidth(chBuffer) > u8g2.getDisplayWidth()) {
      sprintf(chBuffer, "%d", (int)correctedPosition());
    }

    u8g2.setFont(u8g2_font_maniac_tf);    // other fonts: // https://github.com/olikraus/u8g2/wiki/fntlistall
    u8g2.drawStr(max((u8g2.getDisplayWidth() / 2) - (u8g2.getStrWidth(chBuffer) / 2), 0), YPOS_COUNT, chBuffer);

    // set display
    u8g2.sendBuffer();
  }
}
