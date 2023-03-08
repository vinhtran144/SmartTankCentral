
// Import required libraries
#include <DNSServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <mutex>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "AsyncJson.h"
#include "SPIFFS.h"

#define ONE_WIRE_BUS 5
#define NUMBER_OF_TEMPERATURE_SENSOR 2
#define ENABLE_OUTPUT 26
#define SERIAL_DATA_INPUT  25 // DS
#define CLOCK_PIN 32 // SHCP
#define LATCH_PIN 33 // STCP
const unsigned long MAX_EPOCH_VALUE = 2147483647; // Maximum value of epoch

// Configs data, stored a separated JSON in /data
const char* wifi_ssid;
const char* wifi_password;
const char* ap_ssid;
const char* ap_password;
int timezone;
float dechlorinator_dose;
float second_per_ml;

// Tank status
static int tankStatus;          
// Status for tank with
  // 0 for idle
  // 1 for draining
  // 2 for filling
static int reserveStatus;
  // 0 for idle
  // 1 for filling

// Flags
static bool scheduleChange = true; // For schedule manager to know when to update
static bool isBusy = true;      // Stop chaning valves command while shift register is busy


// NTP server
const char* ntpServer = "pool.ntp.org";

// Sensor readings
float tempSensorReading[NUMBER_OF_TEMPERATURE_SENSOR];

// Queue handles
static QueueHandle_t cmd_queue;  // web interface command queue
static QueueHandle_t valve_ctrl_queue;

// Mutex handles
static SemaphoreHandle_t scheduleMutex;
static SemaphoreHandle_t configMutex;
static SemaphoreHandle_t flagsMutex;
static SemaphoreHandle_t statusMutex;
static SemaphoreHandle_t sensorMutex;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dnsServer;

// file paths
const char* schedulePath = "/schedule.json";
const char* configPath = "/config.json";

// Set handling task to 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// =====================================Functions=====================================

// Read config files and update global variables
void readConfig(){
  File openfile = SPIFFS.open(configPath, "r"); 
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, openfile);
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.f_str());
  }
  openfile.close();
  wifi_ssid = doc["wifi_ssid"].as<const char* >();
  wifi_password =  doc["wifi_password"].as<const char* >();
  ap_ssid = doc["ap_ssid"].as<const char* >();
  ap_password = doc["ap_password"].as<const char* >();
  timezone = atoi(doc["timezone"].as<const char* >());
  // timezone=0;
  dechlorinator_dose = atof(doc["dechlorinator_dose"].as<const char* >());
  second_per_ml =  atof(doc["second_per_ml"].as<const char* >());
 
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// For debug, print epoch in readable format
void printTriggerTime(unsigned long epoch){
  time_t rawtime = epoch;
  struct tm  ts;
  char       buf[80];

  // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
  ts = *localtime(&rawtime);
  strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", &ts);
  Serial.print(buf);
}

// Function return trigger next based on schedule epoch
unsigned long getNextTrigger(int offSet, bool schedule[7]){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return(0);
  }
  unsigned long currentDay = getTime();
  char buffer[3];
  strftime(buffer,3,"%u",&timeinfo); int currentWeekday = atoi(buffer);
  strftime(buffer,3,"%H",&timeinfo); const int currentHour = atoi(buffer);
  strftime(buffer,3,"%M",&timeinfo); const int currentMinute = atoi(buffer);
  strftime(buffer,3,"%S",&timeinfo); const int currentSecond = atoi(buffer);
  int currentOffset = currentHour*3600 + currentMinute*60 + currentSecond + timezone;
  
  currentDay = currentDay - ( currentHour*3600 + currentMinute*60 + currentSecond);

  // Adjust due to timezone
  if (currentOffset>86400){
    currentOffset = currentOffset-86400;
    currentDay=currentDay-86400;
    currentWeekday--;
    if (currentWeekday==0) currentWeekday = 7;
  } 
  if (currentOffset<0){
    currentOffset = currentOffset+86400;
    currentDay=currentDay+86400;
    currentWeekday++;
    if (currentWeekday==8) currentWeekday = 1;
  } 
    int index = currentWeekday-1;
    // check for same day trigger, won't activate if it's within 2 mins
    if (currentOffset<offSet && schedule[index]==true){
      return currentDay+offSet;
    }
    // Count till the next day
    int nextDay=1; index++;
    while(nextDay<8){
      if(index>6) index=0;
      if(schedule[index]==true){
        return currentDay+nextDay*24*3600+offSet;
      }
      nextDay++; index++;
    }
    // if it reaches here, then schedule template is empty, so return maximum value
    return MAX_EPOCH_VALUE;
  // }
}

// Function to check if it's active (for light and CO2)
bool isActive (int startOffset, int stopOffset){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return(0);
  }
  char buffer[3];
  strftime(buffer,3,"%H",&timeinfo); const int currentHour = atoi(buffer);
  strftime(buffer,3,"%M",&timeinfo); const int currentMinute = atoi(buffer);
  strftime(buffer,3,"%S",&timeinfo); const int currentSecond = atoi(buffer);
  int currentOffset = currentHour*3600 + currentMinute*60 + currentSecond + timezone;
  if (currentOffset>86400){
    currentOffset = currentOffset-86400;
  } 
  if (currentOffset<0){
    currentOffset = currentOffset+86400;
  } 
  if (startOffset<currentOffset && currentOffset<stopOffset)
    return true;
  else return false;
}

// Funtion to update the shift register
void updateRegister(uint8_t data[],int size){
  for (int i=size-1; i>= 0; i--){
    shiftOut(SERIAL_DATA_INPUT, CLOCK_PIN, LSBFIRST, data[i]);
    digitalWrite(LATCH_PIN, HIGH); 
    digitalWrite(LATCH_PIN, LOW); 
  }
}

// =======================================Tasks=======================================

// handing commands and update shift register state
void shiftRegisterDriver(void *parameter){
  digitalWrite(ENABLE_OUTPUT, LOW);   //Enable shift register out put
  
  uint8_t registerState[2] = {B00000000,B00000000};
  updateRegister(registerState,2);
  Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);

  // PINOUT MASKS
  // 1st shift register
  const uint8_t  PUMP_0_MASK = B00001000;
  const uint8_t  PUMP_1_MASK = B00000100;
  const uint8_t  PUMP_2_MASK = B00000010;
  const uint8_t  PUMP_3_MASK = B00000001;
  // 2nd shift register
  const uint8_t  RESET_ALL = B10101000;
  const uint8_t  START_DRAIN = B01000000;
  const uint8_t  STOP_DRAIN = B10000000;
  const uint8_t  START_FILL = B00010000;
  const uint8_t  STOP_FILL = B00100000;
  const uint8_t  RESERVE_FILL = B00000100;
  const uint8_t  RESERVE_STOP = B00001000;
  const uint8_t  LIGHT_MASK = B00000010;
  const uint8_t  CO2_MASK = B00000001;

  const int DEACTIVATE_VALVE_MS = 8000;                
  const int MS_PER_ML = second_per_ml*1000;
  // get MS per ml from config

  // Start up routine, reseting all the electronic valves
  registerState[1] = RESET_ALL;
  updateRegister(registerState,2);
  Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
  vTaskDelay(DEACTIVATE_VALVE_MS/portTICK_PERIOD_MS);
  registerState[1] = B00000000;
  updateRegister(registerState,2);
  Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);

  char buf[5];

  while(1){
    // Check if queue is empty
    if (!uxQueueMessagesWaiting(cmd_queue)){
      
      if (xSemaphoreTake(flagsMutex,portMAX_DELAY)==pdTRUE){
          isBusy = false;
          xSemaphoreGive(flagsMutex);
        }
      Serial.println("Queue empty");
    }
    // wait at queue indefinitely
    if (xQueueReceive(cmd_queue, (void *)&buf, portMAX_DELAY) == pdTRUE){
      Serial.println(buf);
      if (buf[0] == 'L'){
        // Light command, 0 for off, 1 for on
        if (buf[1]=='0') {
          registerState[1] =  registerState[1] & ~LIGHT_MASK;
        }
        if (buf[1]=='1')  {
          registerState[1] = registerState[1] | LIGHT_MASK;
        }
        updateRegister(registerState,2);
        Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
      }
      if (buf[0] == 'C'){
        // CO2 injection command, 0 for off, 1 for on
        if (buf[1]=='0') {
          registerState[1] =  registerState[1] & ~CO2_MASK;
        }
        if (buf[1]=='1') {
          registerState[1] =  registerState[1] | CO2_MASK;
        }
        updateRegister(registerState,2);
        Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
      }
      if (buf[0] == 'V'){
        // valves command, 0-start drain, 1-stop drain, 2-start fill, 3-stop fill,
        // 4-start fill (reserve), 5-stop fill (reserve)
        if (xSemaphoreTake(flagsMutex,portMAX_DELAY)==pdTRUE){
          isBusy = true;
          xSemaphoreGive(flagsMutex);
        }
        uint8_t action;
        if (buf[1]=='0') action = START_DRAIN;
        if (buf[1]=='1') action = STOP_DRAIN;
        if (buf[1]=='2') action = START_FILL;
        if (buf[1]=='3') action = STOP_FILL;
        if (buf[1]=='4') action = RESERVE_FILL;
        if (buf[1]=='5') action = RESERVE_STOP;

        // Activate valve
        registerState[1] =  registerState[1] | action;
        updateRegister(registerState,2);
        Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
        vTaskDelay(DEACTIVATE_VALVE_MS/portTICK_PERIOD_MS);
        registerState[1] =  registerState[1] & ~action;
        updateRegister(registerState,2);
        Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
        
      }
      if (buf[0] =='D'){
        if (xSemaphoreTake(flagsMutex,portMAX_DELAY)==pdTRUE){
          isBusy = true;
          xSemaphoreGive(flagsMutex);
        }
        //  Dosing pump command, second character is the index of the pump, 3-4th character is the dose in ml
        uint8_t action;
        if (buf[1]=='0') action = PUMP_0_MASK;
        if (buf[1]=='1') action = PUMP_1_MASK;
        if (buf[1]=='2') action = PUMP_2_MASK;
        if (buf[1]=='3') action = PUMP_3_MASK;
        int dosage = (buf[2]-48)*10+(buf[3]-48);
        registerState[0] =  registerState[0] | action;
        updateRegister(registerState,2);
        Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
        vTaskDelay(dosage*MS_PER_ML/portTICK_PERIOD_MS);
        registerState[0] =  registerState[0] & ~action;
        updateRegister(registerState,2);
        Serial.print(registerState[0],BIN);Serial.print(" ");Serial.println(registerState[1],BIN);
        
      }
    }
    
  }
}

// Keep track of schedule and send appropriate commands
void scheduleManager(void *parameter){
  const bool DEBUG = false;

  // for easy assignment and flexibility, the 1st is for starting times, 2nd is for shut off time
  unsigned long triggerEpochs[2][6];        // idividual trigger time 
  int secOffset[2][6];                      // Time of day in seconds

  // templates for the activations
  bool scheduleTemplate[6][7];              // whever device is triggered in each day of the week
  bool enableTemplate[6] = {false,false,false,false,false,false};                   // whever device is enabled or disabled
  int dosage[6]={0,0,0,0,0,0};                            // dose of the dosing pump

  // Assign default values
  for (auto &i : triggerEpochs) {for (auto &j : i){ j = MAX_EPOCH_VALUE; }};
  for (auto &i : secOffset) {for (auto &j : i){ j = 0; }};
  for (auto &i : scheduleTemplate) {for (auto &j : i){ j = false; }};

  unsigned long nextTrigger = MAX_EPOCH_VALUE; // next trigger epoch, basically never trigger
  int nextIndex = 0;

  while(1){
    if (xSemaphoreTake(flagsMutex,0)==pdTRUE) {
      // check flags for changes
      if (scheduleChange == true){
        // Update the flag
        scheduleChange = false;
        xSemaphoreGive(flagsMutex);
        if (xSemaphoreTake(scheduleMutex,portMAX_DELAY)==pdTRUE){
          File openfile = SPIFFS.open(schedulePath, "r"); 
            DynamicJsonDocument doc(1024);
            DeserializationError err = deserializeJson(doc, openfile);
            if (err) {
              Serial.print(F("deserializeJson() failed with code "));
              Serial.println(err.f_str());
            }
          xSemaphoreGive(scheduleMutex);
          
          // In theory, these data would only be adjusted very rarely, so to save memory and less time
          // compairing what changes, the code is kept simple and straight forward.
          for (int i =0; i<6; i++){
            // update enableTemplate
            if (doc["data"][i]["status"].as<int>())
              enableTemplate[i] = true;
            else enableTemplate[i] = false;

            // update offset time, which is in second from midnight
            secOffset[0][i] = doc["data"][i]["startHour"].as<int>()*3600+doc["data"][i]["startMin"].as<int>()*60;
            secOffset[1][i] = doc["data"][i]["endHour"].as<int>()*3600+doc["data"][i]["endMin"].as<int>()*60;

            // update dosage
            dosage[i]=doc["data"][i]["dosage"].as<int>();

            // update schedule template
            const char* schedule = doc["data"][i]["schedule"].as<const char*>();
            for (int j = 0;j<7;j++){
              if (schedule[j]=='1') scheduleTemplate[i][j] =true;
              else scheduleTemplate[i][j] =false;
            }
          }
          // Update the flag
          if (xSemaphoreTake(flagsMutex,portMAX_DELAY)==pdTRUE){
            scheduleChange = false;
            xSemaphoreGive(flagsMutex);
          }
          // Update the triggers epoch arrays
          for (int i = 0;i<2;i++){
            for (int j =0;j<6;j++){
              if (i==1 & j==2) break;
              if(enableTemplate[j]){
                triggerEpochs[i][j] = getNextTrigger(secOffset[i][j],scheduleTemplate[j]);
              }
              else triggerEpochs[i][j] = MAX_EPOCH_VALUE;
            }
          }
          nextTrigger = MAX_EPOCH_VALUE;
          for (int i = 0; i<2; i++){
            for (int j = 0;j<6;j++){
              if (nextTrigger>triggerEpochs[i][j]){
                nextTrigger = triggerEpochs[i][j];
                nextIndex = i*6+j;
              }
            }
          }
          // Activate Light and CO2 if it's in operation hours
          if (isActive(secOffset[0][0],secOffset[1][0])){
            char msg[]="L1";
            if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
              Serial.println("CMD queue is full");
            }
          }
          if (isActive(secOffset[0][1],secOffset[1][1])){
            char msg[]="C1";
            if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
              Serial.println("CMD queue is full");
            }
            
          } 
          // Debug Serialprint
          if (DEBUG){
            Serial.println("Trigger Epochs: ");
            for (auto &i : triggerEpochs) {
              for (auto  &j : i){ 
                printTriggerTime(j);
                Serial.print("     ");
              }
              Serial.println(" ");
            };
            Serial.println(" ");
            Serial.println("Next trigger: ");
            Serial.println(" ");
            printTriggerTime(nextTrigger);
          }
          
        }
      }
      else {
        xSemaphoreGive(flagsMutex);
      }
    }
    unsigned long currentTime = getTime()+timezone;
    if (currentTime>nextTrigger){
      // Get the msg to send
      char msg[5];
      bool sendToCMD = true;
      switch (nextIndex)
      {
        case 0:
          snprintf(msg,5,"L1");
          break;
        case 1:
          snprintf(msg,5,"C1");
          break;
        case 2:
          snprintf(msg,5,"A1");
          sendToCMD = false;
          break;
        case 3:
        case 4:
        case 5:
          {int divided = dosage[nextIndex]/10;
          int remainder = dosage[nextIndex]%10;
          snprintf(msg,5,"D%d%d%d",nextIndex-2,divided,remainder);
          break;}
        case 6:
          snprintf(msg,5,"L0");
          break;
        case 7:
          snprintf(msg,5,"C0");
          break;
       
        default:
          snprintf(msg,5,"E");
          break;
      }
      if (sendToCMD){
        if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
          Serial.println("CMD queue is full");
        }
      } else {
        if (xQueueSend(valve_ctrl_queue, (void *)&msg, 10) != pdTRUE) {
          Serial.println("CMD queue is full");
        }
      }
      

      // Update next trigger epoch
      int i = nextIndex/6;
      int j = nextIndex%6;
      // Serial.print("Updating i= ");
      //   Serial.print(i);Serial.print(" j=");Serial.println(j);
      if(enableTemplate[j]){
        // Serial.print("Current trigger: ");
        printTriggerTime(nextTrigger);
        triggerEpochs[i][j] = getNextTrigger(secOffset[i][j],scheduleTemplate[j]);
        // Serial.println("");
        // Serial.print("Updated trigger: ");
        // printTriggerTime(triggerEpochs[i][j]);
      }
      else triggerEpochs[i][j] = MAX_EPOCH_VALUE;
      nextTrigger=MAX_EPOCH_VALUE;
      for (int i = 0; i<2; i++){
        for (int j = 0;j<6;j++){
          if (nextTrigger>triggerEpochs[i][j]){
            nextTrigger = triggerEpochs[i][j];
            nextIndex = i*6+j;
          }
        }
      }
    }
    else if (nextTrigger-currentTime>60){
      // If the next trigger is more than 1 minutes away, idle and save processing resources
      vTaskDelay(60000/portTICK_PERIOD_MS);
    } 
    else 
    vTaskDelay(1000/portTICK_PERIOD_MS);   
  }
  
}

// Decide states of the valves
void ValvesController(void *parameter){
  #define TANK_HIGH_SENSOR 36
  #define TANK_LOW_SENSOR 39
  #define RESERVE_HIGH_SENSOR 34
  #define RESERVE_LOW_SENSOR 35

  const bool DEBUG = true;

  const int TANK_HIGH_ACTIVATE = LOW;
  const int TANK_LOW_ACTIVATE = LOW;
  const int RESERVE_HIGH_ACTIVATE = HIGH;
  const int RESERVE_LOW_ACTIVATE = LOW;
  // Set up pins
  pinMode(TANK_HIGH_SENSOR,INPUT_PULLUP);
  pinMode(TANK_LOW_SENSOR,INPUT_PULLUP);
  pinMode(RESERVE_HIGH_SENSOR,INPUT_PULLUP);
  pinMode(RESERVE_LOW_SENSOR,INPUT_PULLUP);

  // local flags, keeping track of the actual states of the valves
  // tankState is desired state, while these flags are the actual state of the valves
  bool tankFilling = false;
  bool tankDraining = false;
  bool reserveFilling = false;
  bool isAutoChange = false;
  bool stateChange = false;

  int dosage = dechlorinator_dose;
  
  int delayTime = 0;
  

  while(1){
    if (tankFilling || tankDraining || reserveFilling || isAutoChange || stateChange){
      delayTime = 100;
    } 
    else {
      Serial.println("Waiting at queue indefinitely");
      delayTime = portMAX_DELAY;
    }
    char buf[5];
    if (xQueueReceive(valve_ctrl_queue, (void *)&buf, delayTime) == pdTRUE){
      // Only need to receive A0/A1 for autochange routine, M1 for manual drain, M2 for manual fill
      Serial.println(buf);
      if (buf[0] == 'A'){
        

        if (xSemaphoreTake(statusMutex,portMAX_DELAY)==pdTRUE){
          if (buf[1]=='0'){
            if (tankStatus != 0){
              // stop any actions, should be inputed from the user
              stateChange = true;
              isAutoChange = false;
              tankStatus = 0;
            }
            xSemaphoreGive(statusMutex);
          }
          if (buf[1]=='1'){
              if (tankStatus == 0){
              // start auto change routine only when tank is idle
              stateChange = true;
              isAutoChange = true;
              tankStatus = 1;
            }
            xSemaphoreGive(statusMutex);
          }
          
        }
      }
      if (buf[0] == 'M'){
        stateChange = true;
        if (xSemaphoreTake(statusMutex,portMAX_DELAY)==pdTRUE){
          isAutoChange = false; //override auto change routine
          if (buf[1]=='1'){
            // drain manual change mode
            switch (tankStatus){
              case 0:
                tankStatus = 1; //if idle, start draining
                break;
              case 1:
              case 2: {
                tankStatus = 0; //if active, cancel everything and idle
                break;
              }
              default:
                break;
            }
          
          }
          if (buf[1]=='2'){
            // fill manual change mode
            switch (tankStatus){
              case 0:
                tankStatus = 2; //if idle, start filling
                break;
              case 1:
              case 2: {
                tankStatus = 0; //if active, cancel everything and idle
                break;
              }
              default:
                break;
            }
          }
          Serial.println(tankStatus);
          xSemaphoreGive(statusMutex);
        }
      }
    }
    if (reserveFilling){
      // in case the reserve is filling, check when the reserve is full
      // no other tank activities while reserve is filling
      int sensorReading = digitalRead(RESERVE_HIGH_SENSOR);
      if (sensorReading == RESERVE_HIGH_ACTIVATE){
        if (DEBUG) Serial.println("RESERVE FULL");
        char msg[]="V5";  //stop reserve fill
        reserveFilling = false; stateChange = true;
        if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
          Serial.println("CMD queue is full");
        } 
        if (xSemaphoreTake(statusMutex,portMAX_DELAY)==pdTRUE){
          reserveStatus = 0;
          xSemaphoreGive(statusMutex);
        }

      }
    } else {
      if (stateChange) {

        // attempt to change state
        if (xSemaphoreTake(statusMutex,1000/portTICK_PERIOD_MS)==pdTRUE){
          if (!isBusy && !reserveFilling){
            // Send valves commands only when shift register is not busy and reserve is idle
            stateChange = false;
            switch (tankStatus)
            {
            case 0:
              if (tankDraining) {
                char msg[]="V1";  //stop drain
                tankDraining = false;
                if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
                  Serial.println("CMD queue is full");
                }
              }
              if (tankFilling) {
                char msg[]="V3"; // stop fill
                tankFilling = false;
                if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
                  Serial.println("CMD queue is full");
                }
              }
              break;
            case 1:
              if (!tankDraining){
                Serial.println("Started draining");
                char msg[]="V0";   //start drain
                tankDraining = true;
                if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
                  Serial.println("CMD queue is full");
                }
              }
              if (tankFilling) {
                char msg[]="V3"; // stop fill
                tankFilling = false;
                if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
                  Serial.println("CMD queue is full");
                }
              }
              break;
            case 2:
              if (!tankFilling){
                char msg[]="V2";   //start fill
                tankFilling = true;
                if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
                  Serial.println("CMD queue is full");
                }
              }
              if (tankDraining) {
                char msg[]="V1";  //stop drain
                tankDraining = false;
                if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
                  Serial.println("CMD queue is full");
                }
              }
              break;
            default:
              break;
            }
            xSemaphoreGive(statusMutex);
          } else xSemaphoreGive(statusMutex);
        }
      }
      if (tankDraining) {
        int sensorReading = digitalRead(TANK_LOW_SENSOR);

        if (sensorReading == TANK_LOW_ACTIVATE){
          if (DEBUG) Serial.println("TANK LOW");
          char msg[]="V1";  //stop drain
          tankDraining = false;
          if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
            Serial.println("CMD queue is full");
          } 
          stateChange = true;
          if (xSemaphoreTake(statusMutex,portMAX_DELAY)==pdTRUE){
            if (isAutoChange) {
              tankStatus = 2; //start filling
              isAutoChange = false;
            }
            else tankStatus = 0; //go idle
            xSemaphoreGive(statusMutex);
          }

        }
      }
      if (tankFilling) {
        int sensorReading = digitalRead(RESERVE_LOW_SENSOR);
        if (sensorReading == RESERVE_LOW_ACTIVATE){
          if (DEBUG) Serial.println("RESERVE LOW");
          tankFilling = false;
          stateChange = true;
          reserveFilling = true;
          // queue commands
          char stopMSG[]="V3";  //stop fill
          if (xQueueSend(cmd_queue, (void *)&stopMSG, 10) != pdTRUE) {
            Serial.println("CMD queue is full");
          } 
          char startMSG[]="V4"; //start fill reserve
          if (xQueueSend(cmd_queue, (void *)&startMSG, 10) != pdTRUE) {
            Serial.println("CMD queue is full");
          } 
          char doseMSG[5];
          int divided = dosage/10;
          int remainder = dosage%10;
          snprintf(doseMSG,5,"D%d%d%d",0,divided,remainder);
          if (xQueueSend(cmd_queue, (void *)&doseMSG, 10) != pdTRUE) {
            Serial.println("CMD queue is full");
          } 
          if (xSemaphoreTake(statusMutex,portMAX_DELAY)==pdTRUE){
            reserveStatus = 1;
            xSemaphoreGive(statusMutex);
          }
        }
        sensorReading = digitalRead(TANK_HIGH_SENSOR);
        if (sensorReading == TANK_HIGH_ACTIVATE){
          if (DEBUG) Serial.println("TANK FULL");
          char msg[]="V3";  //stop fill
          tankFilling = false;
          if (xQueueSend(cmd_queue, (void *)&msg, 10) != pdTRUE) {
            Serial.println("CMD queue is full");
          } 
          stateChange = true;
          if (xSemaphoreTake(statusMutex,portMAX_DELAY)==pdTRUE){
            tankStatus = 0; //go idle
            xSemaphoreGive(statusMutex);
          }
        }
      }
    }
  }
}

// Reading DS18B20 temperature sensor
void sensorManager(void *parameter){
  // calling variables for temperature sensors
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  DeviceAddress sensorAddress[NUMBER_OF_TEMPERATURE_SENSOR];
  DeviceAddress tempDeviceAddress;
  sensors.begin();
  int numberOfDevices = sensors.getDeviceCount();
  for(int i=0;i<numberOfDevices; i++){
    if(sensors.getAddress(tempDeviceAddress, i)){
      for (int j=0;j<8; j++){
        sensorAddress[i][j]=tempDeviceAddress[j];
      }
    }
    else Serial.println("Ghost address");
  }

  // main loop
  while(1){
    sensors.requestTemperatures(); 
    if (xSemaphoreTake(sensorMutex,portMAX_DELAY)==pdTRUE){
      // Get temperature sensor data
      float averageTemp = 0;
      for(int i=0;i<numberOfDevices; i++){
        tempSensorReading[i]= sensors.getTempC(sensorAddress[i]);
      }
      
      xSemaphoreGive(sensorMutex);
    }
    // delay 1 second before repeating
    vTaskDelay(10000/portTICK_PERIOD_MS);
  }
  

}

// Manage capture portal
void dnsServerTask(void *parameter){
  while(1){
   dnsServer.processNextRequest();

  }
}


// =======================================Web Processors=======================================

// Schedule attribute strings
String scheduleLoader(const String& var){
  // Wait for 5 seconds to get Mutex. If for some reason it's not avaliable, the website will be empty, shouldn't be a big deal
  if (xSemaphoreTake(scheduleMutex,5000/portTICK_RATE_MS)==pdTRUE){
    // Read schedule config in SPIFFS
    File openfile = SPIFFS.open(schedulePath, "r"); 
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, openfile);
    if (err) {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(err.f_str());
    }
    openfile.close();
    xSemaphoreGive(scheduleMutex);

    // Generate attribute values with schedule data
    char attr [150];
    if (var == "LightSchedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' endHour = '%d' endMin = '%d'' schedule= '%s'",doc["data"][0]["status"].as<int>(),doc["data"][0]["startHour"].as<int>(),doc["data"][0]["startMin"].as<int>(),doc["data"][0]["endHour"].as<int>(),doc["data"][0]["endMin"].as<int>(),doc["data"][0]["schedule"].as<const char* >());
    if (var == "CO2Schedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' endHour = '%d' endMin = '%d'' schedule= '%s'",doc["data"][1]["status"].as<int>(),doc["data"][1]["startHour"].as<int>(),doc["data"][1]["startMin"].as<int>(),doc["data"][1]["endHour"].as<int>(),doc["data"][1]["endMin"].as<int>(),doc["data"][1]["schedule"].as<const char* >());
    if (var == "WaterSchedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' schedule= '%s'",doc["data"][2]["status"].as<int>(),doc["data"][2]["startHour"].as<int>(),doc["data"][2]["startMin"].as<int>(),doc["data"][2]["schedule"].as<const char* >());
    if (var == "Pump1Schedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' dosage= '%d' schedule= '%s'",doc["data"][3]["status"].as<int>(),doc["data"][3]["startHour"].as<int>(),doc["data"][3]["startMin"].as<int>(),doc["data"][3]["dosage"].as<int>(),doc["data"][3]["schedule"].as<const char* >());
    if (var == "Pump2Schedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' dosage= '%d' schedule= '%s'",doc["data"][4]["status"].as<int>(),doc["data"][4]["startHour"].as<int>(),doc["data"][4]["startMin"].as<int>(),doc["data"][4]["dosage"].as<int>(),doc["data"][4]["schedule"].as<const char* >());
    if (var == "Pump3Schedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' dosage= '%d' schedule= '%s'",doc["data"][5]["status"].as<int>(),doc["data"][5]["startHour"].as<int>(),doc["data"][5]["startMin"].as<int>(),doc["data"][5]["dosage"].as<int>(),doc["data"][5]["schedule"].as<const char* >());
    return String(attr);
  }
  return String();
}

// load config values
String configLoader(const String& var){
  // Wait for 5 seconds to get Mutex. If for some reason it's not avaliable, the website will be empty, shouldn't be a big deal
  if (xSemaphoreTake(configMutex,5000/portTICK_RATE_MS)==pdTRUE){
    // Read config config in SPIFFS
    File openfile = SPIFFS.open(configPath, "r"); 
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, openfile);
    if (err) {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(err.f_str());
    }
    openfile.close();
    xSemaphoreGive(configMutex);
    return (doc[var].as<const char* >());
  }
  return String();
}

// =================================Setup Functions======================================
class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/ap/index.html", String(), false); 
  }
};

void setupServer(){
  dnsServer.start(53, "*", WiFi.softAPIP());

  // Serves common source files
  server.on("/src/bootstrap.bundle.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/src/bootstrap.bundle.min.js", "text/javascript");
  });
  server.on("/src/jquery-3.3.1.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/src/jquery-3.3.1.min.js", "text/javascript");
  });
  server.on("/src/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/src/bootstrap.min.css", "text/css");
  });
  server.on("/src/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/src/style.css", "text/css");
  });

  // Serves get request to clients on Access Point
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/index.html", String(), false);
  }).setFilter(ON_AP_FILTER);
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/config.html", String(), false, configLoader);
  }).setFilter(ON_AP_FILTER);
  server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/manual.html", String(), false);
  }).setFilter(ON_AP_FILTER);
  server.on("/schedule", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/schedule.html", String(), false, scheduleLoader);
  }).setFilter(ON_AP_FILTER);

  // Serves XMLHttpRequest for dashboard data
  server.on("/dashboard", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument json(1024);
    json["status"] = "ok";
    json["ip_address"] = WiFi.localIP();
    json["wifi_satus"] = WiFi.status();
    json["timezone"] = timezone;
    json["local_time"] = getTime();
    if (xSemaphoreTake(sensorMutex,1000/portTICK_RATE_MS)==pdTRUE){
      JsonArray readings = json.createNestedArray("temp_sensor");
      for (int i=0; i<NUMBER_OF_TEMPERATURE_SENSOR;i++){
        readings.add(tempSensorReading[i]);
      }
      xSemaphoreGive(sensorMutex);
    } else json["status"] = "error";
    if (xSemaphoreTake(statusMutex,1000/portTICK_RATE_MS)==pdTRUE){
      json["tank_status"] = tankStatus;
      json["reserve_status"] = reserveStatus;
      xSemaphoreGive(statusMutex);
    } else json["status"] = "error";
    
    
    serializeJson(json, *response);
    request->send(response);
  });
  // ServesXMLHRequest for manual controls
  server.on("/controls", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument json(1024);
    json["status"] = "ok";
    if (xSemaphoreTake(flagsMutex,1000/portTICK_RATE_MS)==pdTRUE){
      json["tank_busy"] = isBusy;
      xSemaphoreGive(flagsMutex);
    } else json["status"] = "error";
    
    if (xSemaphoreTake(statusMutex,1000/portTICK_RATE_MS)==pdTRUE){
      json["tank_status"] = tankStatus;
      json["reserve_status"] = reserveStatus;
      xSemaphoreGive(statusMutex);
    } else json["status"] = "error";
    
    
    serializeJson(json, *response);
    request->send(response);
  });

  // Serves post request from clients on Access Point
  server.on("/form", HTTP_POST, [](AsyncWebServerRequest *request){
    // For forms that update the schedules
    int params = request->params();
    AsyncWebParameter* indexP = request->getParam(0);
    if ((indexP->name()) == "formNum"){
      const int index = atoi(indexP->value().c_str());
      if (xSemaphoreTake(scheduleMutex,5000/portTICK_RATE_MS)==pdTRUE){
        File openfile = SPIFFS.open(schedulePath, "r"); 
        DynamicJsonDocument doc(1024);
        DeserializationError err = deserializeJson(doc, openfile);
        if (err) {
          Serial.print(F("deserializeJson() failed with code "));
          Serial.println(err.f_str());
        }
        openfile.close();
        for(int i=1;i<params;i++){
          AsyncWebParameter* p = request->getParam(i);
          // Update the data with param
          if (p->name() == "status" || p->name() == "startHour" || p->name() == "startMin" || p->name() == "endHour"  || p->name() == "endMin" || p->name() == "dosage")
            doc["data"][index][p->name()] = atoi(p->value().c_str());
          if (p->name() == "schedule")
            doc["data"][index][p->name()] = p->value().c_str();

        }
        File writefile = SPIFFS.open(schedulePath,"w");
        serializeJson(doc,writefile);
        writefile.close();
        xSemaphoreGive(scheduleMutex);
        if (xSemaphoreTake(flagsMutex, portMAX_DELAY)==pdTRUE){
          // Set schedule Change flag
          scheduleChange = true;
          xSemaphoreGive(flagsMutex);
        }
      }
    }
      
      request->redirect("/schedule");
  }).setFilter(ON_AP_FILTER); 

  // Dealing with manual commands from client
  server.on("/command",HTTP_POST,[](AsyncWebServerRequest *request){
    AsyncWebParameter* type = request->getParam(0);
      if(type->name() == "type"){
        // Send the message to the right command queues
        String cmdType = type->value().c_str();
        AsyncWebParameter* p = request->getParam(1);
        char msg[5];
        strcpy(msg, p->value().c_str());
        if (cmdType == "valve"){
          if (xQueueSend(valve_ctrl_queue,(void *)&msg,10) != pdTRUE){
             Serial.println("CMD queue is full");
          }
        }
        if (cmdType == "dose"){
          if (xQueueSend(cmd_queue,(void *)&msg,10) != pdTRUE){
             Serial.println("CMD queue is full");
          }
        }
      }      
    
  }).setFilter(ON_AP_FILTER); 

  // Update configs files
  server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request){
    // For config values

    int params = request->params();
    if (xSemaphoreTake(configMutex,5000/portTICK_RATE_MS)==pdTRUE){
      File openfile = SPIFFS.open(configPath, "r"); 
      DynamicJsonDocument doc(512);
      DeserializationError err = deserializeJson(doc, openfile);
      if (err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.f_str());
      }
      openfile.close();
      // Check for previous AP password
      AsyncWebParameter* prePass = request->getParam(params-1);
      String input_pass(prePass->value().c_str());
      String previous_pass(doc["ap_password"].as<const char* >());
      if (input_pass == previous_pass){     
        for(int i=0;i<params-1;i++){
          AsyncWebParameter* p = request->getParam(i);
          // Update the data with param
          doc[p->name()] = p->value().c_str();
        }
        // save new configs values
        File writefile = SPIFFS.open(configPath,"w");
        serializeJson(doc,writefile);
        writefile.close();
        vTaskDelay(500/portTICK_PERIOD_MS);
        readConfig();
        xSemaphoreGive(configMutex);
      }  
      else 
        xSemaphoreGive(configMutex);
        // Can be expanded to alert wrong input password to client, but
        // The server will just ignore queries with the wrong password for now
    }
      
    request->redirect("/config");

  }).setFilter(ON_AP_FILTER); 

  // Captive portal
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
  // Start server
  server.begin();
}

void setup(){
  // Disable shift register output to avoid garbage output while device is starting
  pinMode(ENABLE_OUTPUT,OUTPUT);
  digitalWrite(ENABLE_OUTPUT, HIGH);
  pinMode(SERIAL_DATA_INPUT,OUTPUT);
  pinMode(CLOCK_PIN,OUTPUT);
  pinMode(LATCH_PIN,OUTPUT);

  // Serial port for debugging purposes
  Serial.begin(115200);

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  // Create Mutexs
  scheduleMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();
  flagsMutex = xSemaphoreCreateMutex();
  sensorMutex = xSemaphoreCreateMutex();
  statusMutex = xSemaphoreCreateMutex();
  // Create command data queues
  cmd_queue = xQueueCreate(20,5);
  valve_ctrl_queue =xQueueCreate(20,5);

  WiFi.mode(WIFI_AP_STA);  /*ESP32 Access point configured*/
  setupServer();
  if (xSemaphoreTake(configMutex,portMAX_DELAY)==pdTRUE){
    readConfig();
    xSemaphoreGive(configMutex);
    // Give Mutex back
    Serial.println("\n[*] Creating ESP32 AP");
    WiFi.softAP(ap_ssid, ap_password);  /*Configuring ESP32 access point SSID and password*/
    Serial.print("[+] AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());     /*Printing the AP IP address*/
    WiFi.begin(wifi_ssid, wifi_password);  /*Connecting to Defined Access point*/
    Serial.println("\n[*] Connecting to WiFi Network");
    while(WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }
    Serial.print("\n[+] Connected to WiFi network with local IP : ");
    Serial.println(WiFi.localIP());   /*Printing IP address of Connected network*/
  }
  configTime(0, 0, ntpServer);
   
  if(!getTime()){
    // If failed to get local time, restart
    ESP.restart();
  }
  

  xTaskCreatePinnedToCore (sensorManager,"Read sensor values",	2048 , NULL , 1, NULL, app_cpu);
  xTaskCreatePinnedToCore (scheduleManager,"Manage schedules",	4096 , NULL , 1, NULL, app_cpu);
  xTaskCreatePinnedToCore (shiftRegisterDriver,"Drive drivers",	2048 , NULL , 3, NULL, app_cpu);
  xTaskCreatePinnedToCore (ValvesController, "Control valves",	4096 , NULL , 2, NULL, app_cpu);
  xTaskCreatePinnedToCore (dnsServerTask,"HandLe DNS services",	2048 , NULL , 1, NULL, app_cpu);


  
  
}
 
void loop(){
}