#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
// #include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <string.h>
// #include <ArduinoOTA.h>
#define wifi_ssid "********"
#define wifi_password "**********"
#define my_id 1
#define mqtt_server "192.168.1.7"



#define DEBUG_MODE 0
#define LISTEN_PORT 80

static int TEMP_PORT = 2; //D6
static int UP_IND_PIN = 14;//D0;
static int DN_IND_PIN = 15;

static int UP_SENSE_PIN = 5;//D1;
static int DN_SENSE_PIN = 4;//D2;
static int DOOR_TRIG_PIN = 13;//D7;
static int LIGHT_TRIG_PIN = 12;
// static int ACTIVITY_LED = 16;


String status_topic = "status";
String temperature_topic = "temperature";

String door_control_topic = "door_trigger";
String light_control_topic = "light_trigger";
String status_request_topic = "status";
String temp_request_topic = "temperature";

String out_topic_prefix = "garage/out/";
String in_topic_prefix = "garage/in/";

String status_out_topic;
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
static long forced_reporting_interval = 60000;

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

float previousTemperatureC;
float currentTemperatureC;
int door_trigger();

bool setStatus(){
  up_value = digitalRead(UP_SENSE_PIN);
  dn_value = digitalRead(DN_SENSE_PIN);
  int current_status;
  bool status_changed = false;
  current_status = up_value - dn_value;
  if(current_status != is_up){
    digitalWrite(UP_IND_PIN, up_value);
    digitalWrite(DN_IND_PIN, dn_value);
    is_up = current_status;
    status_changed = true;
  }
  return status_changed;
}

void setup_wifi() {
  ssid = wifi_ssid;
  password = wifi_password;
  delay(10);
  if(DEBUG_MODE){
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  WiFi.mode(WIFI_STA);
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("PW:");
  Serial.println(password);
  WiFi.begin(ssid.c_str(),password.c_str());
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  if(DEBUG_MODE){
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

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
  }else if (topic == temp_request_topic){
    if(DEBUG_MODE) Serial.println("Getting Current Temperature");
    temperature_request();
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

float getTemperature() {
  //Serial.println("Requesting DS18B20 temperature...");
  float temp;
  do {
    DS18B20.requestTemperatures();
    temp = DS18B20.getTempCByIndex(0);
    delay(100);
  } while (temp == 85.0 || temp == (-127.0));
  return temp;
}

void signalStartup(){
  for(int i=0;i<=5;i++){
    digitalWrite(UP_IND_PIN,HIGH);
    delay(250);
    digitalWrite(UP_IND_PIN, LOW);
    digitalWrite(DN_IND_PIN,HIGH);
    delay(250);
    digitalWrite(DN_IND_PIN, LOW);
  }
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

int temperature_request(){
  float myTempC = getTemperature();
  myTempC = round(myTempC*10.0)/10.0;
  if(DEBUG_MODE){
    Serial.print("Reporting Temperature: ");
    Serial.print(myTempC);
    Serial.println("deg C");
  }
  pubMQTT(temperature_out_topic, String(myTempC));
}


String evalIsUp(){
  String myState = "indeterminate";
  switch(is_up){
    case 1:
      myState = "open";
      break;
    case -1:
      myState = "closed";
      break;
    case 0:
      if(motion_direction > 0){
        myState = "opening";
      }else if(motion_direction < 0){
        myState = "closing";
      }else{
        myState = "moving";
      }
      break;
    default:
      myState = "indeterminate";
  }
  return myState;
}

void storeWebCredentials(){
  //try to read ssid and pw from eeprom first
  if(readWebCredentials()){
    setup_wifi();
  }else{
    ssid = wifi_ssid;
    password = wifi_password;
    Serial.println("Clearing EEPROM");
    for(int i = 0; i < 96; ++i){
      EEPROM.write(i,0);
    }
    Serial.println(ssid);
    Serial.println("");
    Serial.println(password);
    Serial.println("");

    Serial.println("Writing EEPROM SSID:");
    for(int i = 0;i < ssid.length(); ++i){
      EEPROM.write(i, ssid[i]);
      Serial.print("Wrote: ");
      Serial.println(ssid[i]);
    }
    Serial.println("Writing EEPROM Password");
    for(int i = 0; i < password.length(); ++i){
      EEPROM.write(32+i, password[i]);
      Serial.print("Wrote: ");
      Serial.println(password[i]);
    }
    EEPROM.commit();
    delay(10);
    storeWebCredentials();
  }
}

bool readWebCredentials(){
  Serial.println("Reading EEPROM ssid");
  String esid;
  for (int i = 0; i < 32; ++i){
    esid += char(EEPROM.read(i));
  }
  if(DEBUG_MODE){
    Serial.print("SSID: ");
    Serial.println(esid);
    Serial.println("Reading EEPROM password");
  }
  String epass;
  for(int i = 32; i < 96; ++i){
    epass += char(EEPROM.read(i));
  }
  if(DEBUG_MODE){
    Serial.print("Password: ");
    Serial.println(epass);
  }
  if(esid.length() > 1){
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  newState = setStatus();
  entry_state = is_up;
  currentState = evalIsUp();
  currentTemperatureC = getTemperature();

  door_triggered_out_topic = out_topic_prefix + String(my_id) + "/" + door_control_topic;
  light_triggered_out_topic = out_topic_prefix + String(my_id) + "/" + light_control_topic;
  temperature_out_topic = out_topic_prefix + String(my_id) + "/" + temp_request_topic;
  status_out_topic = out_topic_prefix + String(my_id) + "/" + status_request_topic;
  subscription_in_topic = in_topic_prefix + String(my_id) + "/#";

  storeWebCredentials();

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
  previousTemperatureC = 4000.0;

  signalStartup();
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
    currentTemperatureC = getTemperature();
    lastTempMsg = now;
  }
  if(abs(previousTemperatureC - currentTemperatureC) > temp_delta_trigger || now - lastForcedReport > 60000){
    reportingTemperature = round(currentTemperatureC*10.0)/10.0;
    pubMQTT(temperature_out_topic, String(reportingTemperature));
    previousTemperatureC = currentTemperatureC;
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
}
