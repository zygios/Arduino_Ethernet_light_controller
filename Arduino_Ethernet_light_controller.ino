#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include "Adafruit_MCP23017.h"
#include <JC_Button.h>
#include <DHT.h>

#define DEBUG

#ifdef DEBUG
  #define debug(x)     Serial.print(x)
  #define debugln(x)   Serial.println(x)
#else
  #define debug(x)     // define empty, so macro does nothing
  #define debugln(x)
#endif

#define ArrayCount(array) byte (sizeof array / sizeof array[0])

//LAN settings
#define ARDUINO_CLIENT_ID "ARD_LIGHT_CONTR"
// Networking details
byte mac[] = {  0x00, 0xAB, 0xXX, 0xXX, 0xXX, 0x02 };     // Ethernet shield MAC address
IPAddress ip (192, 168, xx, xx);                          // Ethernet shield address

EthernetClient ethernetClient;
MqttClient mqttClient(ethernetClient);


// MQTT settings
const char broker[]   = "192.168.xx.xx";
int port = 1883;
const char inTopic[]  = "domoticz/in";  // Default incoming topic in Domoticz is domoticz/in
const char outTopic[] = "domoticz/out"; // Default outgoing topic in Domoticz is domoticz/in
char myOutput[512];
const char switchType[] = "On/Off";

// Interval settings
unsigned long previousMillis = 0;     // Store last time
const long interval          = 60000; // interval (milliseconds)

#define DHTPIN 32     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
const byte GarTempHumIdx = 20;
const long Upd_Period =  60000L;  //30000=30sec 
const int ArduinoLiveCheck = 258; 

Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;

// button pin assignments
#define outputOn 1  
#define outputOff 0
static const struct{const int btn_pin, mcp, relay, dom_idx, deb;}
butt_relay_mapping[] = {
  // btn_pin,  mcp, relay, dom_idx;
  {46,    0,     0,    81,  0},  
  {44,    0,    15,    82,  0},  
  {40,    0,     9,    83,  0},  
  {42,    0,     8,    84,  0},  
  {36,    0,     2,    85,  0},  
  {38,    0,    10,    86,  0},  
  {47,    0,     7,   133,  0},  
  {34,    0,     1,    88,  0},  
  {45,    0,     4,   134,  0},  
  {0,     0,     3,    32,  1000},
  {0,     0,    12,   183,  0},  
  {0,     0,     5,   123,  0},  
  {0,     0,    14,   999,  0},  
  {0,     0,    11,   193,  1000},
  {43,    0,    13,   200,  0},  
  {0,     1,    12,   184,  0},  
  {0,     1,     4,   185,  0},  
  {0,     1,    13,   186,  0},  
  {0,     1,     5,   187,  0},  
  {0,     1,    14,   188,  0},  
  {0,     1,     6,   189,  0},  
  {0,     1,    15,   190,  0},  
  {0,     1,     7,   191,  0},  
  {0,     1,     3,   192,  0},  
  {0,     1,     8,   216,  0},  
  {0,     1,    11,   126,  0},  
  {0,     1,     2,   127,  0},  
  {0,     1,    10,   128,  0},  
  {0,     1,     1,   129,  0},  
  {0,     1,     9,   125,  0},  
  {0,     1,     0,   124,  0}   
  
};

const byte butt_relay_Num = ArrayCount(butt_relay_mapping);

//#define NUM_BUTTONS butt_relay_Num
//const byte BUTTON_PINS[NUM_BUTTONS] = { 2, 3 };
ToggleButton *myBtn[butt_relay_Num];

void setup() {

  Wire.begin();
  
  mcp0.begin(0); //0x20
  mcp1.begin(1); //0x21
  
  // Initialize serial for debug
  Serial.begin(115200);
  
  debug(F("Starting init of device "));
  debugln(ARDUINO_CLIENT_ID);

  // Ethernet shield configuration
  Ethernet.init(53);
  Ethernet.begin(mac, ip);
  debug(F("IP adress:"));
  debugln(Ethernet.localIP());
      
  // Set MQTT client
  mqttClient.onMessage(onMqttMessage);
  mqttClient.subscribe(outTopic); 

  //init relay outputs
  for( byte i = 0; i < 16; i++) 
  {
     mcp0.pinMode(i, OUTPUT);
     mcp0.digitalWrite(i, HIGH);
     mcp1.pinMode(i, OUTPUT);
     mcp1.digitalWrite(i, HIGH);
  }
  
  // initialize the button objects
  for (byte i = 0; i < butt_relay_Num; i++)
  {
    myBtn[i] = new ToggleButton(butt_relay_mapping[i].btn_pin,false);
    debug("Button ");
    debug(butt_relay_mapping[i].btn_pin);
    debug(" status ");
    debugln(myBtn[i]->toggleState());
    writeToRelay(butt_relay_mapping[i].mcp, butt_relay_mapping[i].relay, myBtn[i]->toggleState(), 0);
    myBtn[i]->begin();
  } 

  dht.begin();
  
  delay(500);
  debug(F("End init of device."));
  debugln(ARDUINO_CLIENT_ID);
}


void loop(){
  // If not connected to the broker, try to connect
  while (!mqttClient.connected()) {
    startBroker();
  }

  unsigned long currentMillis = millis();
  DynamicJsonDocument doc(256);
  
  if(currentMillis - previousMillis > Upd_Period){
    previousMillis = currentMillis;
    debugln("Nuskaitom AM2302");
    float AM2302_h = dht.readHumidity();
    float AM2302_t = dht.readTemperature();
//    String payload = "{\"idx\":"+ String(GarTempHumIdx) +",\"svalue\":\" " + String(AM2302_t) + ";" + AM2302_h +";0;\"}";

    // Serialize the message (input state)
      doc.clear();
      doc["command"] = "switchlight";
      doc["idx"] = ArduinoLiveCheck;
      doc["switchcmd"] = "On";
      // Send the message   
      mqttClient.beginMessage(inTopic);
      serializeJson(doc, mqttClient);
      mqttClient.endMessage();
      // show serialization message
      serializeJsonPretty(doc, myOutput);
      debugln(myOutput);
  }

  //cycle for all lights
  for (byte i = 0; i < butt_relay_Num; i++)
  {
    myBtn[i]->read();
    if (myBtn[i]->changed())
    {
      debug(F("Button "));
      debug(butt_relay_mapping[i].btn_pin);
      debug(F(" was pressed, value: "));
      debug(myBtn[i]->toggleState());
      debug(F(" relay number: "));
      debugln(butt_relay_mapping[i].relay);
      writeToRelay(butt_relay_mapping[i].mcp, butt_relay_mapping[i].relay, myBtn[i]->toggleState(), butt_relay_mapping[i].deb);
          // Serialize the message (input state)
      doc.clear();
      doc["command"] = "switchlight";
      doc["idx"] = butt_relay_mapping[i].dom_idx;
      if (myBtn[i]->toggleState()) {
        doc["switchcmd"] = "On";
      } else {
        doc["switchcmd"] = "Off";
      }
      // Send the message   
      mqttClient.beginMessage(inTopic);
      serializeJson(doc, mqttClient);
      mqttClient.endMessage();
      // show serialization message
      serializeJsonPretty(doc, myOutput);
      debugln(myOutput);
    }
  }
  

  mqttClient.poll();
}


void onMqttMessage(int messageSize) {
  // we received a message
  debug(F("Received a message with topic: '"));
  debug(mqttClient.messageTopic());
  debug(F("', length: "));
  debug(messageSize);
  debugln(F(" bytes:"));
  // deserialize the message
  DynamicJsonDocument doc(messageSize);
  DeserializationError err = deserializeJson(doc, mqttClient);
  if (err) {
    debug(F("deserializeJson() failed with code: "));
    debugln(err.f_str());
  } else {
    // show deserialization message
    serializeJsonPretty(doc, myOutput);
    debugln(myOutput);
    // change output state
    for (byte i = 0; i < butt_relay_Num; i++)
    {
      if ((butt_relay_mapping[i].dom_idx == doc["idx"]) and (doc["switchType"] == switchType)) {
        debug(F("Incoming change for: "));
        debug((char*)doc["name"]);
        debug(F(" (idx: "));
        debug((int)doc["idx"]);
        debug(F("), new status: "));
        debugln((int)doc["nvalue"]);
//        digitalWrite(outputArray[pin], doc["nvalue"] ? outputOn : outputOff);
        writeToRelay(butt_relay_mapping[i].mcp, butt_relay_mapping[i].relay, doc["nvalue"] ? outputOn : outputOff, butt_relay_mapping[i].deb);
      }    
    }
  }
}

void writeToRelay(int mcp_num, int relay_num, byte butt_state, int debounce){
  butt_state = !butt_state;
  debug(F("MCP num:"));
  debugln(mcp_num);
  debug(F("Relay num:"));
  debugln(relay_num);
  debug(F("Relay debaunce:"));
  debugln(debounce);
  if(mcp_num == 0){
    mcp0.digitalWrite(relay_num, butt_state);     
    if(debounce > 0){
      delay(debounce);
      mcp0.digitalWrite(relay_num, 1);     
    }
  }else{
    mcp1.digitalWrite(relay_num, butt_state);
    if(debounce > 0){
      delay(debounce);
      mcp0.digitalWrite(relay_num, 1);     
    }
  }
}

void startEthernet() {
  // Initialize network
  ethernetClient.stop();
  debugln(F("Attempting to connect to network: "));
  while (Ethernet.begin(mac) == 0) {
    debug(F("."));
    delay(5000); // Wait 5 seconds, and try again
  }
  debugln(F("Connected to network"));
  debug(F("IP address: "));
  debugln(Ethernet.localIP());
}

void startBroker () {
  // Initialize broker
  mqttClient.stop();
  debug(F("Attempting to connect to broker"));
  unsigned int mqttConnection = 0;
  while (!mqttClient.connect(broker, port)) {
    debug(F("."));
    delay(5000); // Wait 5 seconds, and try again
    mqttConnection +=1;
    if (mqttConnection >= 5) {
      debug(F("MQTT connection failed! Error code = "));
      debugln(mqttClient.connectError());
      startEthernet();
      return false;
    }    
  }
  debugln(F("Connected to broker"));
  mqttClient.subscribe(outTopic);
  return true;
}
