#include <LWiFi.h>
#include <MCS.h>
#include <MCS_debug.h>
#include <LRTC.h>

using namespace std;

// DHT
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);

// Realy
#define RelayPin 12 

// Soil_moist
int SoilPin = 17;

char ssid[] = "winni_4";
char password[] = "073647402";

//MCS
MCSDevice mcs("D0H9tR4n", "yx2ARYJzcuLW0xp6");
MCSDisplayFloat temperature("T");
MCSDisplayFloat humidity("h");
MCSDisplayFloat light("light");
MCSDisplayFloat soil_moist("soil_moist");
MCSControllerOnOff relay("relay");
MCSControllerInteger soil_threshold("soil_threshold");
MCSControllerInteger temp_threshold("temp_threshold");
MCSDisplayInteger water_t("wt");
MCSDisplayInteger Max_water_t("Mwt");
bool watering = 0;
int water_times = 0;
int max_water_times = 30;
int Soil_T = 4000;
float temp_T = 30;

//Json
String msgStr = "";
int n=0;

//online_time
#include <WiFiUdp.h>
#include <ctime>

const char *NTP_server = "time.stdtime.gov.tw";
const int NTP_PACKET_SIZE = 48;                   // NTP time stamp is in the first 48 bytes of the message
static byte packetBuffer[NTP_PACKET_SIZE] = {0};  //buffer to hold incoming and outgoing packets
const unsigned int localPort = 2390;              // local port to listen for UDP packets
static WiFiUDP Udp;                               // A UDP instance to let us send and receive packets over UDP

String getNetworkTime() {
  Udp.begin(localPort);
  sendNTPpacket(NTP_server); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    const unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    const unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    const unsigned long secsSince1900 = highWord << 16 | lowWord;
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    const unsigned long epoch = secsSince1900 - seventyYears;
    // Taiwan is UTC+8 = 8 * 60 * 60 seconds
    const time_t taiwan_time = epoch + (8 * 60 * 60);
    // const tm* pTime = gmtime(&taiwan_time);
    static char time_text[] = "YYYY-MM-DDTHH:MM:SS+08";
    strftime(time_text, sizeof(time_text), "%Y-%m-%dT%H:%M:%S+08", gmtime(&taiwan_time));
    return String((const char*)time_text);
  }

  return String("Connection error");
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(const char* host) {
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(host, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");

  return 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  dht.begin();
  setup_wifi();
  LRTC.begin();
  pinMode(SoilPin,INPUT); //Soil_moisture
  //LRTC.set(2019, 2, 2, 23, 11, 30);
  mcs.addChannel(temperature);
  mcs.addChannel(humidity);
  mcs.addChannel(light);
  mcs.addChannel(soil_moist);
  mcs.addChannel(relay);
  mcs.addChannel(soil_threshold);
  mcs.addChannel(temp_threshold);
  mcs.addChannel(water_t);
  mcs.addChannel(Max_water_t);  

  mcs.connect();
  mcs.process(100);
  Soil_T = soil_threshold.value();
  temp_T = temp_threshold.value();
}

void loop() {
  LRTC.get();
  int hr = LRTC.hour();
  int minute = LRTC.minute();
  int second = LRTC.second(); 
  msgStr = "";
  String NowTime = getNetworkTime();
  if(NowTime != "Connection error"){
    String year,month,day,hour,minu,seco = "";
    hour = hour +NowTime[11];
    hour = hour +NowTime[12];
    minu = minu +NowTime[14];
    minu = minu +NowTime[15];
    seco = seco +NowTime[17];
    seco = seco +NowTime[18];

    year = year+NowTime[0];
    year = year+NowTime[1];
    year = year+NowTime[2];
    year = year+NowTime[3];
    month = month+NowTime[5];
    month = month+NowTime[6];
    day = day +NowTime[8];
    day = day +NowTime[9];
    
    hr = hour.toInt();
    minute = minu.toInt();
    second = seco.toInt();    

    int yyyy = year.toInt();
    int mm = month.toInt();
    int dd = day.toInt(); 
    LRTC.set(yyyy, mm, dd, hr, minute, second);
  }
  //int hr = LRTC.hour();
  //int minute = LRTC.minute();
  //int second = LRTC.second();
  // put your main code here, to run repeatedly:
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int l = analogRead(A0);
  int now_soil_moisture = analogRead(SoilPin);
  msgStr = msgStr+ "[ "+hr+" : "+minute+ " : "+second +" ]\n";
  msgStr = msgStr+ "{\"溫度\":" + t + 
                     ",\"濕度\":" + h + 
                     ",\"光線\":"+ l + 
                     ",\"土壤濕度\":"+ now_soil_moisture +
                     "}";
  Serial.println(NowTime);
  Serial.println(msgStr); 

  Serial.print(LRTC.hour());
  Serial.print(LRTC.minute());
  Serial.println(LRTC.second());
  
  if((hr>=5 and hr<=8) or (hr >= 21 and hr <= 23)){
    watering = is_watering(now_soil_moisture);   
    Serial.println("time1");
  }else if(t>temp_T and (hr>=10 and hr<=12)){
    watering = is_watering(now_soil_moisture); 
    Serial.println("time2");
  }else{
    watering = false;
    water_times = 0;
    Serial.println("time3");
  }

  max_water_times = check_max_water_times(t, h, hr);
  int is_w_mcs = 0;
  if(water_times<max_water_times){
    is_w_mcs = open_watering_relay(watering);
    water_times += is_w_mcs;
  }
  else{
    is_w_mcs = open_watering_relay(false);
  }
  Serial.print("澆水次數/最大次數：");
  Serial.print(water_times);
  Serial.print(" / ");
  Serial.println(max_water_times);

  water_times += upload_mcs(n, h, t, l, now_soil_moisture,water_times,max_water_times, is_w_mcs);
  if(soil_threshold.updated()){
    Soil_T = soil_threshold.value();
    Serial.print("MCS 土壤閾值修改：");
    Serial.println(Soil_T);
  } 
  if(temp_threshold.updated()){
    temp_T = temp_threshold.value();
    Serial.print("MCS 溫度閾值修改：");
  } 
  Serial.println(temp_T);    
  Serial.println(Soil_T);
  Serial.println(water_times);
  delay(1000); //每1秒回傳一次資料
  n++;
}

int upload_mcs(int n, float h, float t, int l, int now_soil_moisture, int water_times,int max_water_times, int is_w_mcs){
  //upload MCS
  int try_time =0;
  int manual_water_time = 0;
  while (!mcs.connected() and try_time<5) {
    mcs.connect();
    try_time++;
  }
  mcs.process(100);
  Serial.print("Relay Value:");
  Serial.println(relay.value());
  if(relay.updated() and relay.value()>0){
    Serial.println("Relay Updated");
    watering = relay.value();
    open_watering_relay(watering);
    Serial.print("MCS啟動：");
    Serial.println(watering);
    temperature.set(t);
    humidity.set(h);
    light.set(l);
    soil_moist.set(now_soil_moisture);
    water_t.set(water_times+5);
    Max_water_t.set(max_water_times);
    manual_water_time += 5;
    delay(120000);
  } 
  if(n%150==0 or is_w_mcs>0){
    temperature.set(t);
    humidity.set(h);
    light.set(l);
    soil_moist.set(now_soil_moisture);
    water_t.set(water_times);
    Max_water_t.set(max_water_times);
    Serial.println("update MCS");
  }
  return manual_water_time;
}

int check_max_water_times(float temp, float humidity, int hour){
  float pre_max;
  pre_max = 3*temp*(0.3+(100 - humidity)/100);
  if(hour>12){
    pre_max *= 0.85;
  }
  return int(pre_max);
}

bool is_watering(int soil_moist){
  bool relay = 0;
  if(soil_moist > Soil_T){
    relay = 1;
  }
  return relay;
}

int open_watering_relay(bool w){
  if(w){
    digitalWrite(RelayPin, HIGH);
    Serial.println("Relay_HIGH");
    delay(20000);
    return 1;
  }
  else{
    digitalWrite(RelayPin, LOW);
    Serial.println("Relay_LOW"); 
    return 0;
  }
}

void setup_wifi() {
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int try_time = 0; 
  while (WiFi.status() != WL_CONNECTED and try_time<80) {
    delay(500);
    Serial.print(".");
    if(try_time==40){
      char ssid[] = "cheno_02";
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      WiFi.begin(ssid, password);   
    }
    try_time++;
  }
  randomSeed(micros());
  Serial.println("Connected to wifi");
  printWifiStatus();
}

void printWifiStatus() {                     //顯示Wifi狀態
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
