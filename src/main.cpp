
// Import required libraries
#include <Arduino.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <mutex>
#include <ESP32Ping.h>

// Configs data, stored a separated JSON in /data
const char* wifi_ssid;
const char* wifi_password;
const char* ap_ssid;
const char* ap_password;
int timezone;
float phOffset;
float dechlorinatorDose;
float secondPerMl;

// Flags
bool scheduleChange = false;

// Queue handles
static QueueHandle_t cmd_queue;  // web interface command queue

// Mutex handles
static SemaphoreHandle_t scheduleMutex;
static SemaphoreHandle_t configMutex;
static SemaphoreHandle_t flagsMutex;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// file paths
const char* schedulePath = "/schedule.json";
const char* configPath = "/config.json";

// Set handling task to 1 core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

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
  // setup the task when first ran
  if (xSemaphoreTake(scheduleMutex,portMAX_DELAY)==pdTRUE){
    // File openfile = SPIFFS.open(schedulePath, "r"); 
    //   DynamicJsonDocument doc(1024);
    //   DeserializationError err = deserializeJson(doc, openfile);
    //   if (err) {
    //     Serial.print(F("deserializeJson() failed with code "));
    //     Serial.println(err.f_str());
    //   }
    xSemaphoreGive(scheduleMutex);

  }
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
  timezone = doc["timezone"].as<int >();
  phOffset = doc["phOffset"].as<float>();
  dechlorinatorDose = doc["dechlorinatorDose"].as<float>();
  secondPerMl =  doc["dechlorinatorDose"].as<float>();
}

// =======================================Web Processors=======================================

// Replaces placeholder with LED state value
String processor(const String& var){
  // Serial.println(var);
  // if(var == "STATE"){
  //   if(digitalRead(ledPin)){
  //     ledState = "ON";
  //   }
  //   else{
  //     ledState = "OFF";
  //   }
  //   Serial.print(ledState);
  //   return ledState;
  // }
  return "test";
}

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

// Config values
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
  
  const String test = "test";
  // Test Ping
  bool success = Ping.ping("www.google.com", 3);
  if (success) Serial.println("Ping successful");
  else Serial.println("Ping failed");

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
    request->send(SPIFFS, "/ap/index.html", String(), false, processor);
  }).setFilter(ON_AP_FILTER);
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/config.html", String(), false, configLoader);
  }).setFilter(ON_AP_FILTER);
  server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/manual.html", String(), false, processor);
  }).setFilter(ON_AP_FILTER);
  server.on("/schedule", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/ap/schedule.html", String(), false, scheduleLoader);
  }).setFilter(ON_AP_FILTER);

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

  

  // Start server
  server.begin();
}
 
void loop(){
  
}