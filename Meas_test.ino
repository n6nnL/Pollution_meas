#include <WiFi.h>
#include "time.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
// WiFi тохируулах
const char* ssid = "n6nnL";
const char* password = "qwerty1234";

// MQTT Broker (Мэдээлэл өгөх хаяг)
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "emqx/esp32";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Хэмжилтийн тогтмолууд
float adc_voltage = 0.0;
float in_voltage = 0.0;

float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5;
int adc_value = 0;

float baseVoltage = 2.5;
float sensitivity = 0.185;

// Хэмжилт эхлэх ба дуусах цаг
const int startHour = 11;
const int endHour = 24;
const unsigned int interval = 20000;

// NTP(Network Time Protocol) хаяг
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 8 * 3600);

unsigned long previousMillis = 0;
// PV Нарны зайн дэлгэцийн класс
class PV {
  public:
    int vPin; // Хүчдэлийн сенсор
    int iPin; // Гүйдлийн сенсор
    int dPin; // Арчигч мотор
    int count;
    float voltage; // Тухайн агшны хүчдэл
    float current; // Тухайн агшны гүйдэл
    float power; // Тухайн агшны чадал
    float pCount[300]; // Чадал өдрийн турш
    float energy; // Тухайн интервалын энерги
    float eCount[60]; // Өдөр бүрийн энерги хадгалагдах
    float dp[60]; //
    
    PV();
    ~PV();
    float vMeas(); // Хүчдэл хэмжих
    float iMeas(); // Гүйдэл хэмжих
    void setPins(int,int,int); // Сенсорын хөл тохируулах
    void fullMeas(int, const char*); // Энерги хүртэл тооцоолж мэдээллийг серверт өгөх
    float calPower(float, float, int); // Чадал тооцох
    float calPolbyPow(float); // Чадлыг харьцуулсан бохирдол
    float calEnergy(int);
    void clearData();
    void clean(); // Цэвэрлэх
};
PV::PV(){
  voltage = 0;
  current = 0;
  power = 0;
  energy = 0;
  count = 0;
}
PV::~PV() { }

void PV::clearData() {
  count = 0;
  voltage = 0;
  current = 0;
  power = 0;
  energy = 0;
}
// Сенсорын хөл тохируулах
void PV::setPins(int i, int v, int d){
  iPin = i;
  vPin = v;
  dPin = d;
}

float PV::calPower(float voltage, float current, int n) {
  float P = voltage * current;
  this->pCount[n] = P;
  return P;
}

float PV::calEnergy(int interval){
  for (int i = 0 ; i < sizeof(pCount) / sizeof(pCount[0]); i++){
    energy += pCount[i] * (interval/1000);
  }
  return energy;
}
float PV::calPolbyPow(float pow){
  return (pow - this->power)/pow;
}
// Цэвэрлэх
void PV::clean() {
  digitalWrite(dPin, HIGH);
  delay(5000);
  digitalWrite(dPin, LOW);
}
 // Хүчдэл хэмжих
float PV::vMeas() {
  adc_value = analogRead(vPin);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage / (R2 / (R1 + R2));
  Serial.print("V = ");
  Serial.println(in_voltage, 7);
  this->voltage = in_voltage;
  return in_voltage;
}
// Гүйдэл хэмжих
float PV::iMeas() {
  float AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0;
  float input_current = 0, real_current = 0;
  for (int x = 0; x < 150; x++) {
    AcsValue = analogRead(iPin);
    Samples += AcsValue;
    delay(3);
  }
  AvgAcs = Samples / 150;
  input_current = (AvgAcs * 5) / 1024.0;
  real_current = (input_current - baseVoltage) / (sensitivity * 2);
  Serial.print("I = ");
  Serial.print(real_current, 7);
  Serial.println(" A");
  this->current = real_current;

  yield();
  return real_current;
}
// Энерги хүртэл тооцоолж мэдээллийг серверт өгөх
void PV::fullMeas(int count, const char* pvID) {
   voltage = this->vMeas();
   current = this->iMeas();
   power = calPower(voltage, current, count);
   energy = calEnergy(interval);
   this->count++;

   // Get the current timestamp
   char timestamp[64];
   snprintf(timestamp, sizeof(timestamp), "%02d:%02d:%02d", 
            timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds());

   // Prepare MQTT message
   char message[256];
   snprintf(message, sizeof(message), 
            "{\"pvID\":\"%s\",\"timestamp\":\"%s\",\"count\":%d,\"voltage\":%.2f,\"current\":%.2f,\"power\":%.2f,\"energy\":%.2f}",
            pvID, timestamp, this->count, voltage, current, power, energy);

   // Publish to MQTT
   client.publish(topic, message);
}

// Function to calculate daily pollution rate from dp[] array

// Recalculate break-even point each day based on new dailyLossRate

// харьцуулах НЗД-үүдийг үүсгэх
PV ref, unit;



// MQTT connection shalgah
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("Reconnecting to MQTT...");
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Reconnected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
// MQTT callback
void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}




void setup() {

  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  // WiFi setup
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // MQTT setup
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Public EMQX MQTT broker connected");
    } else {
        Serial.print("failed with state ");
        Serial.print(client.state());
        delay(2000);
    }
}
  //Timeclient setup
  timeClient.begin();
  timeClient.update();

  //OTA setup
  ArduinoOTA.setHostname("OTA_ESP32");
  ArduinoOTA.begin();

  Serial.println("DC Measure Test");

  float ratePerKWh = 0.10;
  float cleaningCost = 500;
  // Хөлнүүдийг гараар тохируулсан
  ref.setPins(35,32,25);
  unit.setPins(35,34,26);
}


bool energyPublished = false;

void loop() {
  timeClient.update();
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  unsigned long currentMillis = millis();
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop(); // Ensure MQTT client stays active

  if (currentHour >= startHour && currentHour < endHour) {
    if (currentHour == startHour && currentMinute == 0) {
      ref.clean();
    }
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Хэмжилтийг хадгалах
      ref.fullMeas(ref.count, "ref");  
      unit.fullMeas(unit.count, "unit"); 

      Serial.printf("Measurements taken successfully!!!\n Current time: %02d:%02d\n", currentHour, currentMinute);
      yield();
    }
  } else {
    if (!energyPublished) {
      // Нийт үйлдвэрлэсэн энергийг хадгалах
      char totalEnergyMessage[256];
      snprintf(totalEnergyMessage, sizeof(totalEnergyMessage),
               "{\"totalEnergyRef\":%.2f,\"totalEnergyUnit\":%.2f}",
               ref.energy, unit.energy);
      client.publish(topic, totalEnergyMessage);
      Serial.println("Total energy published outside measurement window");

      // Энергийг 0 - лэх
      ref.energy = 0;
      unit.energy = 0;
      energyPublished = true;
    }
    delay(1000); // Prevent spamming
  }

  if (currentHour == startHour && currentMinute == 0) {
    energyPublished = false; // Reset flag for new day
  }
}
