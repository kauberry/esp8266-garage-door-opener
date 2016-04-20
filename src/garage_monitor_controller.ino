#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <aREST.h>
//#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <ArduinoOTA.h>
#define wifi_ssid "*********"
#define wifi_password "********"
#define mqtt_server "192.168.1.7"
#define status_topic "garage/status"
#define temperature_topic "garage/temperature"
#define door_trigger_topic "garage/door_triggered"


#define DEBUG_MODE 1
#define LISTEN_PORT 80

OneWire ds(D6);
DallasTemperature DS18B20(&ds);


WiFiClient espClient;
PubSubClient client(espClient);
//WiFiServer TelnetServer(8266);
aREST rest = aREST(client,mqtt_server);
WiFiServer RESTServer(LISTEN_PORT);

//Variables to expose to the API
float temperature;
String door_status;

//Declare functions to be exposed to the API
int doorControl(String command);



static int UP_IND_PIN = D0;
static int DN_IND_PIN = D5;

static int UP_SENSE_PIN = D1;
static int DN_SENSE_PIN = D2;
static int DOOR_TRIG_PIN = D7;

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
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid,wifi_password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }


  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


}

void reconnect(){
  while(!client.connected()){
    Serial.print("Attempting MQTT Connection...");
    if(client.connect(mqtt_server)){
      Serial.println("Connected");
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void pubMQTT(String topic, String topic_val){
  Serial.print("Newest topic " + topic + " value:");
  Serial.println(topic_val);
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

String evalIsUp(){
  String myState = "Indeterminate";
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

void setup() {
  Serial.begin(115200);
  newState = setStatus();
  entry_state = is_up;
  currentState = evalIsUp();
  currentTemperatureC = getTemperature();
  rest.variable("temperature",&currentTemperatureC);
  rest.variable("door_status",&currentState);

  rest.function("door",doorControl);
  rest.set_id("1");
  rest.set_name("esp8266");
  setup_wifi();
  //client.setServer(mqtt_server, 1883);
  pinMode(UP_IND_PIN, OUTPUT);
  pinMode(DN_IND_PIN, OUTPUT);
  pinMode(DOOR_TRIG_PIN, OUTPUT);
  digitalWrite(DOOR_TRIG_PIN,LOW);
  pinMode(UP_SENSE_PIN, INPUT_PULLUP);
  pinMode(DN_SENSE_PIN, INPUT_PULLUP);
  previousTemperatureC = 4000.0;

  // ArduinoOTA.onStart([]() {
  //   Serial.println("OTA Start");
  // });
  // ArduinoOTA.onEnd([]() {
  //   Serial.println("\nEnd");
  // });
  //   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  //   Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  // });
  // ArduinoOTA.onError([](ota_error_t error) {
  //   Serial.printf("Error[%u]: ", error);
  //   if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  //   else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  //   else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  //   else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  //   else if (error == OTA_END_ERROR) Serial.println("End Failed");
  // });
  // ArduinoOTA.setHostname("garage_esp");
  // TelnetServer.begin();
  // ArduinoOTA.begin();
  RESTServer.begin();
  signalStartup();
}


void loop(){
  // if(!client.connected()){
  //   reconnect();
  // }
  WiFiClient rest_client = RESTServer.available();
  // if(!rest_client){
  //   return;
  // }
  // while(!rest_client.available()){
  //   delay(1);
  // }
  //ArduinoOTA.handle();
  //client.loop();
  rest.loop(client);
  rest.handle(rest_client);
  newState = setStatus();
  motion_direction = is_up - entry_state;
  long now = millis();
  if(now - lastTempMsg > 2000){
    currentTemperatureC = getTemperature();
    lastTempMsg = now;
  }
  if(abs(previousTemperatureC - currentTemperatureC) > temp_delta_trigger || now - lastForcedReport > 60000){
    pubMQTT(temperature_topic, String(currentTemperatureC));
    previousTemperatureC = currentTemperatureC;
    lastForcedReport = now;
  }
  if(newState && now - lastMsg > 200){
    entry_state = is_up;
    lastMsg = now;
    currentState = evalIsUp();
    Serial.print("New State => ");
    Serial.println(currentState);
    pubMQTT(status_topic,currentState);
  }


}

int doorControl(String command){
  digitalWrite(DOOR_TRIG_PIN,HIGH);
  delay(250);
  digitalWrite(DOOR_TRIG_PIN, LOW);
  pubMQTT(door_trigger_topic, "triggered");
}
