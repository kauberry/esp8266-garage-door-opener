#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <FS.h>
#include <string.h>
#include <ArduinoOTA.h>
#define wifi_ssid "HomeWiFi"
#define wifi_password "navi3com"
#define my_id 1
#define mqtt_server "192.168.1.7"
#define HOSTNAME "esp8266-ota-"



#define DEBUG_MODE 0
#define LISTEN_PORT 80

static int TEMP_PORT = 2; //D6
static int UP_IND_PIN = 15;//D0;
static int DN_IND_PIN = 14;

static int UP_SENSE_PIN = 5;//D1;
static int DN_SENSE_PIN = 4;//D2;
static int DOOR_TRIG_PIN = 13;//D7;
static int LIGHT_TRIG_PIN = 12;
// static int ACTIVITY_LED = 16;

const char* ap_default_ssid = "esp8266"; ///< Default SSID.
const char* ap_default_psk = "esp8266esp8266"; ///< Default PSK.
/// @}

/// Uncomment the next line for verbose output over UART.
// #define SERIAL_VERBOSE

String status_topic = "status";
String temperature_topic = "temperature";

String door_control_topic = "door_trigger";
String light_control_topic = "light_trigger";
String status_request_topic = "status";
String case_temp_request_topic = "case_temperature";
String ambient_temp_request_topic = "ambient_temperature";
String identify_topic = "identify";

String out_topic_prefix = "garage/out/";
String in_topic_prefix = "garage/in/";

String status_out_topic;
String case_temperature_out_topic;
String ambient_temperature_out_topic;
String temperature_out_topic;
String door_triggered_out_topic;
String light_triggered_out_topic;
String subscription_in_topic;


int door_trigger_delay_ms = 500;

// OneWire ds(D6);
OneWire ds(TEMP_PORT);
DallasTemperature DS18B20(&ds);

void callback_handler(char* topic, byte* payload, unsigned int length);


WiFiClient espClient;
PubSubClient client(espClient);

//Variables to expose to the API
float temperature;
String door_status;
String ssid;
String password;


static float temp_delta_trigger = 0.5;
static long forced_reporting_interval = 300000;

int up_value;
int dn_value;
int is_up;
int loop_status = -2;
int entry_state;


long lastForcedReport = 0;
long lastMsg = 0;
long lastTempMsg = 0;
String currentState = "Indeterminate";
bool newState = false;
int motion_direction = 0;

float previousAmbientTemperatureC;
float previousCaseTemperatureC;
float currentAmbientTemperatureC;
float currentCaseTemperatureC;
int door_trigger();

bool loadConfig(String *ssid, String *pass)
{
  // open file for reading.
  File configFile = SPIFFS.open("/cl_conf.txt", "r");
  if (!configFile)
  {
    Serial.println("Failed to open cl_conf.txt.");

    return false;
  }

  // Read content from config file.
  String content = configFile.readString();
  configFile.close();

  content.trim();

  // Check if ther is a second line available.
  int8_t pos = content.indexOf("\r\n");
  uint8_t le = 2;
  // check for linux and mac line ending.
  if (pos == -1)
  {
    le = 1;
    pos = content.indexOf("\n");
    if (pos == -1)
    {
      pos = content.indexOf("\r");
    }
  }

  // If there is no second line: Some information is missing.
  if (pos == -1)
  {
    Serial.println("Invalid content.");
    Serial.println(content);

    return false;
  }

  // Store SSID and PSK into string vars.
  *ssid = content.substring(0, pos);
  *pass = content.substring(pos + le);

  ssid->trim();
  pass->trim();

#ifdef SERIAL_VERBOSE
  Serial.println("----- file content -----");
  Serial.println(content);
  Serial.println("----- file content -----");
  Serial.println("ssid: " + *ssid);
  Serial.println("psk:  " + *pass);
#endif

  return true;
} // loadConfig


/**
 * @brief Save WiFi SSID and PSK to configuration file.
 * @param ssid SSID as string pointer.
 * @param pass PSK as string pointer,
 * @return True or False.
 */
bool saveConfig(String *ssid, String *pass)
{
  // Open config file for writing.
  File configFile = SPIFFS.open("/cl_conf.txt", "w");
  if (!configFile)
  {
    Serial.println("Failed to open cl_conf.txt for writing");

    return false;
  }

  // Save SSID and PSK.
  configFile.println(*ssid);
  configFile.println(*pass);

  configFile.close();

  return true;
} // saveConfig

bool setStatus(){
  up_value = digitalRead(UP_SENSE_PIN);
  dn_value = digitalRead(DN_SENSE_PIN);
  int current_status;
  bool status_changed = false;
  current_status = up_value - dn_value;
  digitalWrite(UP_IND_PIN, up_value);
  digitalWrite(DN_IND_PIN, dn_value);
  if(current_status != is_up){
    status_changed = true;
  }
  is_up = current_status;
  return status_changed;
}

void setup_wifi() {
  String station_ssid = "";
  String station_psk = "";
  // ssid = wifi_ssid;
  // password = wifi_password;
  delay(500);
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(),HEX);
  WiFi.hostname(hostname);
  if(DEBUG_MODE){
    Serial.println("\r\n");
    Serial.println("Hostname: " + hostname);
    Serial.println("\r\n");
    Serial.print("Chip ID: 0x");
    Serial.println(ESP.getChipId(), HEX);
    Serial.println("\r\n");
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  // if (!SPIFFS.begin()){
  //   Serial.println("Failed to mount filesystem");
  //   return;
  // }

  // if (! loadConfig(&station_ssid, &station_psk)){
    station_ssid = wifi_ssid;
    station_psk = wifi_password;
    // Serial.println("No WiFi connection info available");
  // }

  if (WiFi.getMode() != WIFI_STA){
    WiFi.mode(WIFI_STA);
  }
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("PW:");
  Serial.println(password);
  if (WiFi.SSID() != station_ssid || WiFi.psk() != station_psk)
  {
    Serial.println("WiFi config changed.");

    // ... Try to connect to WiFi station.
    WiFi.begin(station_ssid.c_str(), station_psk.c_str());

    // ... Pritn new SSID
    Serial.print("new SSID: ");
    Serial.println(WiFi.SSID());

    // ... Uncomment this for debugging output.
    //WiFi.printDiag(Serial);
  }else{
    // ... Begin with sdk config.
    WiFi.begin();
  }

  Serial.println("Wait for WiFi connection.");

  // ... Give ESP 10 seconds to connect to station.
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000){
    Serial.write('.');
    //Serial.print(WiFi.status());
    delay(500);
  }
  Serial.println();

  // Check connection
  if(WiFi.status() == WL_CONNECTED){
    // ... print IP Address
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }else{
    Serial.println("Can not connect to WiFi station. Go into AP mode.");
    // Go into software AP mode.
    WiFi.mode(WIFI_AP);
    delay(10);
    WiFi.softAP(ap_default_ssid, ap_default_psk);

    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
  }

  // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();


  #ifdef SERIAL_VERBOSE
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  #endif

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback_handler);
}



void callback_handler(char* topic, byte* payload, unsigned int length){
  if(DEBUG_MODE){
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i=0;i<length;i++){
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }
  char buffer[length+1];
  strncpy(buffer, (char*)payload,length);
  buffer[length] = '\0';
  char * topicToken;
  topicToken = strtok(topic, "/");
  topicToken = strtok(NULL, "/");
  topicToken = strtok(NULL, "/");
  uint16_t toaddr = strtol(topicToken, NULL, 10);
  // String newTopic = topicToken;
  topicToken = strtok(NULL, "/");
  if(DEBUG_MODE){
    Serial.print("token: ");
    Serial.println(topicToken);
  }
  topicProcessor(topicToken);
}

void topicProcessor(String topic){
  if(topic == door_control_topic){
    if(DEBUG_MODE) Serial.println("Triggering door controls");
    door_trigger();
  }else if (topic == light_control_topic){
    if(DEBUG_MODE) Serial.println("Triggering door controls");
    light_trigger();
  }else if (topic == status_request_topic){
    if(DEBUG_MODE) Serial.println("Getting Latest Status");
    get_current_door_state();
  }else if (topic == ambient_temp_request_topic){
    if(DEBUG_MODE) Serial.println("Getting Current Ambient Temperature");
    temperature_request(0);
  }else if (topic == case_temp_request_topic){
    if(DEBUG_MODE) Serial.println("Getting Current Case Temperature");
    temperature_request(1);
  }else if (topic == identify_topic){
    if(DEBUG_MODE) Serial.println("Processing Identify Request");
    identify_request();
  }else{

  }
}

void reconnect(){
  while(!client.connected()){
    Serial.print("Attempting MQTT Connection... ");
    if(client.connect("GarageESPClient")){
      Serial.println("Connected");
      client.subscribe(subscription_in_topic.c_str());
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void pubMQTT(String topic, String topic_val){
  if(DEBUG_MODE){
    Serial.print("Newest topic " + topic + " value:");
    Serial.println(topic_val);
  }
  client.publish(topic.c_str(), topic_val.c_str(),true);
}

float getTemperature(uint8_t sensorIndex) {
  //Serial.println("Requesting DS18B20 temperature...");
  float temp;
  do {
    DS18B20.requestTemperatures();
    temp = DS18B20.getTempCByIndex(sensorIndex);
    delay(100);
  } while (temp == 85.0 || temp == (-127.0));
  return temp;
}

void signalStartup(int delayTime){
  for(int i=0;i<=5;i++){
    digitalWrite(UP_IND_PIN,HIGH);
    delay(delayTime);
    digitalWrite(UP_IND_PIN, LOW);
    digitalWrite(DN_IND_PIN,HIGH);
    delay(delayTime);
    digitalWrite(DN_IND_PIN, LOW);
  }
}

void getTempDevices(){
  int devCount;
  DeviceAddress d;
  devCount = 2; //DS18B20.getDeviceCount();
  for(int i=0;i<devCount;i++){
    DS18B20.getAddress(d,i);
    printAddress(d);
  }
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println(' ');
}


int door_trigger(){
  if(is_up != 0){
    digitalWrite(DOOR_TRIG_PIN,HIGH);
    if(is_up == 1){
      digitalWrite(UP_IND_PIN, LOW);
      digitalWrite(DN_IND_PIN, HIGH);
    }else if(is_up == -1){
      digitalWrite(DN_IND_PIN, LOW);
      digitalWrite(UP_IND_PIN, HIGH);
    }
    pubMQTT(door_triggered_out_topic, "triggered");
    delay(door_trigger_delay_ms);
    digitalWrite(DOOR_TRIG_PIN, LOW);
    digitalWrite(DN_IND_PIN,LOW);
    digitalWrite(UP_IND_PIN,LOW);
  }
  newState = setStatus();
}

int light_trigger(){
  digitalWrite(LIGHT_TRIG_PIN,HIGH);
  pubMQTT(light_triggered_out_topic, "triggered");
  delay(1000);
  digitalWrite(LIGHT_TRIG_PIN, LOW);
}

int get_current_door_state(){
  String state = evalIsUp();
  pubMQTT(status_out_topic, String(state));
}

int temperature_request(uint8_t sensorIndex){
  float myTempC = getTemperature(sensorIndex);
  myTempC = round(myTempC*10.0)/10.0;
  String sensorName;
  sensorName = sensorIndex == 0 ? 'ambient' : 'case';
  temperature_out_topic = sensorIndex == 0 ? ambient_temperature_out_topic : case_temperature_out_topic;
  if(DEBUG_MODE){
    Serial.print("Reporting Temperature: ");
    Serial.print(myTempC);
    Serial.print("deg C");
    Serial.print(" (");
    Serial.print(sensorName);
    Serial.println(")");
  }
  pubMQTT(temperature_out_topic, String(myTempC));
}

void identify_request(){
  signalStartup(750);
  return;
}


String evalIsUp(){
  String myState = "indeterminate";
  switch(is_up){
    case 1:
      // myState = "open";
      myState = "0";
      break;
    case -1:
      // myState = "closed";
      myState = "1";
      break;
    case 0:
      if(motion_direction > 0){
        // myState = "opening";
        myState = "2";
      }else if(motion_direction < 0){
        // myState = "closing";
        myState = "3";
      }else{
        // myState = "stopped";
        myState = "4";
      }
      break;
    default:
      myState = "4";
  }
  return myState;
}

void setup() {
  Serial.begin(115200);
  // delay(500);
  // newState = setStatus();
  entry_state = is_up;
  currentState = evalIsUp();
  currentAmbientTemperatureC = getTemperature(0);
  currentCaseTemperatureC = getTemperature(1);

  door_triggered_out_topic = out_topic_prefix + String(my_id) + "/" + door_control_topic;
  light_triggered_out_topic = out_topic_prefix + String(my_id) + "/" + light_control_topic;
  case_temperature_out_topic = out_topic_prefix + String(my_id) + "/" + case_temp_request_topic;
  ambient_temperature_out_topic = out_topic_prefix + String(my_id) + "/" + ambient_temp_request_topic;
  status_out_topic = out_topic_prefix + String(my_id) + "/" + status_request_topic;
  subscription_in_topic = in_topic_prefix + String(my_id) + "/#";

  setup_wifi();

  pinMode(UP_IND_PIN, OUTPUT);
  digitalWrite(UP_IND_PIN,LOW);
  pinMode(DN_IND_PIN, OUTPUT);
  digitalWrite(DN_IND_PIN,LOW);
  pinMode(DOOR_TRIG_PIN, OUTPUT);
  digitalWrite(DOOR_TRIG_PIN,LOW);
  pinMode(LIGHT_TRIG_PIN, OUTPUT);
  digitalWrite(LIGHT_TRIG_PIN,LOW);
  // pinMode(ACTIVITY_LED,OUTPUT);
  // digitalWrite(ACTIVITY_LED,LOW);
  pinMode(UP_SENSE_PIN, INPUT_PULLUP);
  pinMode(DN_SENSE_PIN, INPUT_PULLUP);
  previousAmbientTemperatureC = 4000.0;
  previousCaseTemperatureC = 4000.0;

  signalStartup(250);
  getTempDevices();

  // newState = setStatus();
}

float reportingTemperature;
void loop(){
  if(!client.connected()){
    reconnect();
  }
  newState = setStatus();
  motion_direction = is_up - entry_state;
  long now = millis();
  if(now - lastTempMsg > 2000){
    currentAmbientTemperatureC = getTemperature(0);
    currentCaseTemperatureC = getTemperature(1);
    lastTempMsg = now;
  }
  if(abs(previousAmbientTemperatureC - currentAmbientTemperatureC) > temp_delta_trigger || now - lastForcedReport > forced_reporting_interval){
    reportingTemperature = round(currentAmbientTemperatureC*10.0)/10.0;
    pubMQTT(ambient_temperature_out_topic, String(reportingTemperature));
    previousAmbientTemperatureC = currentAmbientTemperatureC;
    lastForcedReport = now;
  }
  if(abs(previousCaseTemperatureC - currentCaseTemperatureC) > temp_delta_trigger || now - lastForcedReport > forced_reporting_interval){
    reportingTemperature = round(currentCaseTemperatureC*10.0)/10.0;
    pubMQTT(case_temperature_out_topic, String(reportingTemperature));
    previousCaseTemperatureC = currentCaseTemperatureC;
    lastForcedReport = now;
  }
  if(newState && now - lastMsg > 200){
    entry_state = is_up;
    lastMsg = now;
    currentState = evalIsUp();
    if(DEBUG_MODE){
      Serial.print("New State => ");
      Serial.println(currentState);
    }
    // pubMQTT(constructTopic(my_id,status_topic),currentState);
    pubMQTT(status_out_topic, currentState);
  }
  client.loop();
  ArduinoOTA.handle();
  yield();
}
