
// Import required libraries
#include <DNSServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <mutex>
#include <ESP32Ping.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "AsyncJson.h"
#include "SPIFFS.h"
#include <time.h>

#define ONE_WIRE_BUS 5
#define NUMBER_OF_TEMPERATURE_SENSOR 2
#define TDS_SENSOR ADC1_CHANNEL_6
#define NUM_OF_SAMPLE 30

// Configs data, stored a separated JSON in /data
const char* wifi_ssid;
const char* wifi_password;
const char* ap_ssid;
const char* ap_password;
int timezone;
float high_tds;
float dechlorinator_dose;
float second_per_ml;

// Flags
bool scheduleChange = true;
bool tdsWarning = false;
byte tankStatus = 0;          
// Status for tank with
  // 0 for idle
  // 1 for draining
  // 2 for filling
  // 3 for paused (refilling reserved tank)
  // 4 for halted (high TDS reading in reserved tank)
byte reserveStatus = 0;
  // 0 for idle
  // 1 for filling
  // 2 for error

// NTP server
const char* ntpServer = "pool.ntp.org";

// Sensor readings
float tempSensorReading[NUMBER_OF_TEMPERATURE_SENSOR];
float tdsSensorReading;

// Queue handles
static QueueHandle_t cmd_queue;  // web interface command queue

// Mutex handles
static SemaphoreHandle_t scheduleMutex;
static SemaphoreHandle_t configMutex;
static SemaphoreHandle_t flagsMutex;
static SemaphoreHandle_t sensorMutex;
static SemaphoreHandle_t historyMutex;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dnsServer;

// file paths
const char* schedulePath = "/schedule.json";
const char* configPath = "/config.json";
const char* historyPath = "/history.json";

// Set handling task to 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// =====================================Functions=====================================
// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

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
  high_tds = atof(doc["high_tds"].as<const char* >());
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

void printTriggerTime(unsigned long epoch){
  time_t rawtime = epoch;
  struct tm  ts;
  char       buf[80];

  // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
  ts = *localtime(&rawtime);
  strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", &ts);
  Serial.print(buf);
}

// Function return trigger epoch
unsigned long getNextTrigger(int offSet, bool isDaily, bool schedule[7]){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return(0);
  }
  unsigned long currentDay = getTime();
  char buffer[3];
  strftime(buffer,3,"%u",&timeinfo); const int currentWeekday = atoi(buffer);
  strftime(buffer,3,"%H",&timeinfo); const int currentHour = atoi(buffer);
  strftime(buffer,3,"%M",&timeinfo); const int currentMinute = atoi(buffer);
  strftime(buffer,3,"%S",&timeinfo); const int currentSecond = atoi(buffer);
  const int currentOffset = currentHour*3600 + currentMinute*60 + currentSecond+timezone;
  currentDay = currentDay - ( currentHour*3600 + currentMinute*60 + currentSecond);
  if (isDaily){
    if (currentOffset<offSet){
      // Trigger the same day
      return currentDay+offSet;
    }
    // Otherwise, trigger next day
    return currentDay+24*3600+offSet;
  }
  else {
    int index = currentWeekday-1;
    // check for same day trigger
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
    return 2147483647;
  }
}


// =======================================Tasks=======================================

// Task handing command from web interface
void cmdHandler(void *parameter){
  
  char buf[50];

  while(1){
    // check queue
    if (xQueueReceive(cmd_queue, (void *)&buf, 0) == pdTRUE){
      Serial.println(buf);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void scheduleManager(void *parameter){
  const unsigned long MAX_VAL = 2147483647; // Maximum value of epoch
  const int NUMBER_OF_DEVICES = 6;

  // for easy assignment and flexibility, the 1st is for starting times, 2nd is for shut off time
  unsigned long triggerEpochs[2][6];        // idividual trigger time 
  int secOffset[2][6];                      // Time of day in seconds

  // templates for the activations
  bool scheduleTemplate[6][7];              // whever device is triggered in each day of the week
  bool enableTemplate[6];                   // whever device is enabled or disabled
  int dosage[6];                            // dose of the dosing pump

  // Assign default values
  for (auto &i : triggerEpochs) {for (auto &j : i){ j = MAX_VAL; }};
  for (auto &i : secOffset) {for (auto &j : i){ j = 0; }};
  for (auto &i : scheduleTemplate) {for (auto &j : i){ j = false; }};
  for (int i : enableTemplate) { i = false; };
  for (int i : dosage) { i = 0; };

  unsigned long nextTrigger = MAX_VAL; // next trigger epoch, basically never trigger
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
          for (int i =0; i<NUMBER_OF_DEVICES; i++){
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

          // Update the triggers epoch arrays, 1st row 
          for (int j =0;j<6;j++){
            if(enableTemplate[j]){
              if(j<2) triggerEpochs[0][j] = getNextTrigger(secOffset[0][j],true,scheduleTemplate[j]);
              else triggerEpochs[0][j] = getNextTrigger(secOffset[0][j],false,scheduleTemplate[j]);
            }
            else triggerEpochs[0][j] = MAX_VAL;
          }
          // Update those with stop triggers, 2nd row
          for (int j =0;j<2;j++){
            if(enableTemplate[j]){
              triggerEpochs[1][j] = getNextTrigger(secOffset[1][j],true,scheduleTemplate[j]);
            }
            else triggerEpochs[1][j] = MAX_VAL;
          }

          nextTrigger = MAX_VAL;
          for (int i = 0; i<2; i++){
            for (int j = 0;j<6;j++){
              if (nextTrigger>triggerEpochs[i][j]){
                nextTrigger = triggerEpochs[i][j];
                nextIndex = i*6+j;
              }
            }
          }
          

          // Update the flag
          if (xSemaphoreTake(flagsMutex,portMAX_DELAY)==pdTRUE){
            scheduleChange = false;
            xSemaphoreGive(flagsMutex);
          }

          // Debug Serialprint
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
          printTriggerTime(nextTrigger);
        }
      }
      else {
        xSemaphoreGive(flagsMutex);
      }
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
  
}

// Reading DS18B20 temperature sensor
void sensorManager(void *parameter){
  // Setup for pH sensor
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(TDS_SENSOR,ADC_ATTEN_DB_11);
  int buffer[NUM_OF_SAMPLE];

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
      // Get TDS sensor data
      // for (int i=0;i<NUM_OF_SAMPLE;i++){
      //   buffer[i] = adc1_get_raw(TDS_SENSOR);
      //   vTaskDelay(20/portTICK_PERIOD_MS);
      // }
      // float averageVoltage = getMedianNum(buffer,NUM_OF_SAMPLE)*3.3/4096.0;
      // float compensationCoefficient = 1.0+0.02*(26.0-25.0);
      // //temperature compensation
      // float compensationVoltage=averageVoltage/compensationCoefficient;
      // //convert voltage value to tds value
      // float tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      // Serial.print("TDS reading: ");
      // Serial.println(tdsValue);
      xSemaphoreGive(sensorMutex);
    }
    // delay 1 second before repeating
    vTaskDelay(1000/portTICK_PERIOD_MS);
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
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' endHour = '%d' endMin = '%d'",doc["data"][0]["status"].as<int>(),doc["data"][0]["startHour"].as<int>(),doc["data"][0]["startMin"].as<int>(),doc["data"][0]["endHour"].as<int>(),doc["data"][0]["endMin"].as<int>());
    if (var == "CO2Schedule") 
      snprintf(attr,150,"status = '%d' startHour = '%d' startMin = '%d' endHour = '%d' endMin = '%d'",doc["data"][1]["status"].as<int>(),doc["data"][1]["startHour"].as<int>(),doc["data"][1]["startMin"].as<int>(),doc["data"][1]["endHour"].as<int>(),doc["data"][1]["endMin"].as<int>());
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
    DynamicJsonDocument doc(1024);
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
    if (xSemaphoreTake(flagsMutex,1000/portTICK_RATE_MS)==pdTRUE){
      json["pump_status"] = tankStatus;
      json["reserve_status"] = reserveStatus;
      xSemaphoreGive(flagsMutex);
    } else json["status"] = "error";
    if (xSemaphoreTake(historyMutex,1000/portTICK_RATE_MS)==pdTRUE){
      File openfile = SPIFFS.open(historyPath, "r"); 
      DynamicJsonDocument doc(1024);
      DeserializationError err = deserializeJson(doc, openfile);
      if (err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.f_str());
      }
      openfile.close();
      json["reserve_last_full"]=doc["last_fill_epoch"];
      json["reserve_tsd"]=doc["last_tds_read"];
      xSemaphoreGive(historyMutex);
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
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request){
    // For config values

    int params = request->params();
    if (xSemaphoreTake(configMutex,5000/portTICK_RATE_MS)==pdTRUE){
      File openfile = SPIFFS.open(configPath, "r"); 
      DynamicJsonDocument doc(1024);
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
  historyMutex = xSemaphoreCreateMutex();

  // Create command data queue
  cmd_queue = xQueueCreate(10,50);

  WiFi.mode(WIFI_AP_STA);  /*ESP32 Access point configured*/

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
   
  // Test Ping
  bool pingResult = Ping.ping("www.google.com", 3);
  if (pingResult) Serial.println("Ping successful");
  else Serial.println("Ping failed");

  xTaskCreatePinnedToCore (sensorManager,"Read sensor values",	2048 , NULL , 1, NULL, app_cpu);
  xTaskCreatePinnedToCore (scheduleManager,"Manage schedules",	4096 , NULL , 1, NULL, app_cpu);

  setupServer();
  
}
 
void loop(){
   dnsServer.processNextRequest();
}