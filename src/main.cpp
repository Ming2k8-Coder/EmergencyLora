#include <Arduino.h>
#include <Arduino_JSON.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
//-------------------------------------------------
// Global Var
int flags = 0;
String reportedNode;
int lastflags = 0;
String incoming;
int alaram = 0;
bool triggeralarm = false;
//---------------------------
#define nodeaddres "Nha2"
#define mode1 10
#define mode2 4000
//-------------------------
int modeadc;
#define modeselect 34
#define led1 35
#define alarmpin 32
#define button 12


#define ss 5
#define rst 4
#define dio0 2


#define PRESS_TIME 2000 // 2000 milliseconds

int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;
//------------------------------------------------
TaskHandle_t mainloop;
TaskHandle_t mainalarm_t;
TaskHandle_t alteralarm_t;
//------------------------------------------------

void receivedCallback(String &msg);
void sendMessage();
void alteralarm(void *pvParameter);
void mainalarm(void *pvParameter);
// RTOS
void alteralarm(void *pvParameter)
{
  while (true)
  {
    delay(2000);
    digitalWrite(alarmpin, 1);
    delay(1000);
    digitalWrite(alarmpin, 0);
  }
}
void mainalarm(void *pvParameter)
{
  while (true)
  {
    digitalWrite(alarmpin, 1);
    delay(2000);
  }
}
void resetalarm()
{
  triggeralarm = false;
  vTaskSuspend(alteralarm_t);
  vTaskSuspend(mainalarm_t);
  digitalWrite(alarmpin, 0);
}
//--------------------------
String getreading()
{
  JSONVar jread;
  jread["flags"] = flags;
  jread["ReportedNode"] = nodeaddres;
  String readings = JSON.stringify(jread);
  return readings;
}

void sendMessage()
{
  String msg = getreading();
  Serial.print("LRMessage: ");
  Serial.print(msg);
  Serial.println();
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
}
void update(void *pvParameter)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500;
  BaseType_t xWasDelayed;
  xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
    // lrdebug
    modeadc = analogRead(modeselect);
    Serial.println(modeadc);
    Serial.print("RSSI:");
    Serial.println(LoRa.rssi());
    Serial.print("LastPackRSSI:");
    Serial.println(LoRa.packetRssi());
    Serial.print("LastPackSNR:");
    Serial.println(LoRa.packetSnr());
    // if recv packet
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
      // received a packet
      if (LoRa.available())
      {
        incoming = LoRa.readString();
      }
      receivedCallback(incoming);
    }
    if (modeadc < mode1)
    {
      flags = 1;
    }
    if (modeadc > mode2)
    {
      flags = 2;
    }
    if (modeadc > mode1 && modeadc < mode2)
    {
      flags = 0;
    }
    // Button dêtch program
    currentState = digitalRead(button);

    if (lastState == HIGH && currentState == LOW)
    { // button is pressed
      pressedTime = millis();
      isPressing = true;
      isLongDetected = false;
    }
    else if (lastState == LOW && currentState == HIGH)
    { // button is released
      isPressing = false;
      releasedTime = millis();

      long pressDuration = releasedTime - pressedTime;

      if (pressDuration < PRESS_TIME)
      {
        Serial.println("A short press is detected");
        if (flags != 0)
        {
          triggeralarm = true;
          Serial.println("TRIGGERED");
        }
      }
    }

    if (isPressing == true && isLongDetected == false)
    {
      long pressDuration = millis() - pressedTime;

      if (pressDuration > PRESS_TIME)
      {
        Serial.println("A long press is detected");
        isLongDetected = true;
        if (flags == 0)
        {
          resetalarm();
          for (int i = 0; i < 6; i++)
          {
            sendMessage();
          }
        }
        if (flags == 1)
        {
          vTaskResume(mainalarm_t);
          delay(5000);
          resetalarm();
        }
        if (flags == 2)
        {
          vTaskResume(alteralarm_t);
          delay(5000);
          resetalarm();
        }
      }
    }

    // save the the last state
    lastState = currentState;

    // Flag check
    Serial.print(flags);
    Serial.println(triggeralarm);
    if (flags == 1 && triggeralarm)
    {
      Serial.println("modmain");
          for (int i = 0; i < 6; i++)
          {
            sendMessage();
          }
      vTaskSuspend(alteralarm_t);
      vTaskResume(mainalarm_t);
    }
    if (flags == 2 && triggeralarm)
    {
      Serial.println("modalter");
          for (int i = 0; i < 6; i++)
          {
            sendMessage();
          }
      vTaskSuspend(mainalarm_t);
      vTaskResume(alteralarm_t);
    }
  }
}
void receivedCallback(String &msg)
{
  Serial.print("RECIVED:");Serial.println(msg);
  JSONVar myObject = JSON.parse(msg.c_str());
  int flagsrecv = myObject["flags"];
  String node = JSON.stringify(myObject["ReportedNode"]);
  if (flagsrecv == 0 && reportedNode == node)
  {
    resetalarm();
  }
  else if (flagsrecv == 1)
  {
    vTaskSuspend(alteralarm_t);
    vTaskResume(mainalarm_t);
    reportedNode = node;
  }
  else if (flagsrecv == 2)
  {
    vTaskSuspend(mainalarm_t);
    vTaskResume(alteralarm_t);
    reportedNode = node;
  }
}

void setup()
{
  pinMode(alarmpin, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("UART INITED");
  Serial.println("Lora INIT COMPLETE....");
//setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  while(!LoRa.begin(433E6)) {
    Serial.println("Failed.");
    delay(500);
  }
  LoRa.setSpreadingFactor(8);
  LoRa.setTxPower(7);
  LoRa.setSyncWord(0xF3);
   //LoRa.setSignalBandwidth(250E3);
//------------------------------------
  Serial.println("[APP] Free mem @ Startup: " + String(esp_get_free_heap_size()) + " bytes");
  Serial.println();

  xTaskCreatePinnedToCore(update, "Main", 65536, NULL, 3, &mainloop, 1);
  xTaskCreatePinnedToCore(alteralarm, "NonEmerAlm", 4096, NULL, 1, &alteralarm_t, 0);
  vTaskSuspend(alteralarm_t);
  xTaskCreatePinnedToCore(mainalarm, "EmerAlm", 4096, NULL, 1, &mainalarm_t, 0);
  vTaskSuspend(mainalarm_t);
  Serial.println("DOne Setup");
  delay(1000);
  resetalarm();
  // initialize Serial Monitor
}

void loop()
{
}
