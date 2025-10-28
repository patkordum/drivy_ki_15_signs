#include <WiFi.h>

#define PIN_MOTOR_RIGHT_UP 19
#define PIN_MOTOR_RIGHT_DN 18
#define PIN_MOTOR_LEFT_UP 16
#define PIN_MOTOR_LEFT_DN 17

//Attention: Hardcoded WLAN SSID and HOST Ip of socket server, port 5001
//needs DOIT ESP32 DEVKIT V1 package in arduino IDE

// WiFi Credentials
const char* ssid = "OpenWrt";
const char* password = "";

//Socket Server 
const char* host = "192.168.1.101";  // Replace with your Ubuntu IP
const int port = 5001;

WiFiClient client;


int nSleep = 4;
int LEDhr = 23;  // Beleuchtung hinten rechts
int LEDhl = 15;  // Beleuchtung hinten links
int LEDvl = 13;  // Bremslicht vorne links
int LEDvr = 32;  // Bremslicht vorne rechts

int vel0=200;



void setup() {
  Serial.begin(9600);

  pinMode(nSleep, OUTPUT);
  digitalWrite(nSleep,HIGH);

  Serial.println("\nWiFi Connection ... trying...");
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("\n not connected");
  }
  Serial.println("\nWiFi connected");

  // Connect to the server
  Serial.printf("Connecting to %s:%d...\n", host, port);
  if (!client.connect(host, port)) {
    Serial.println("Connection to server failed!");
    return;
  }
  Serial.println("Connected to server.");

  pinMode(PIN_MOTOR_RIGHT_UP, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DN, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_UP, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DN, OUTPUT);

  pinMode(LEDhr, OUTPUT);
  pinMode(LEDhl, OUTPUT);
  pinMode(LEDvr, OUTPUT);
  pinMode(LEDvl, OUTPUT);

  analogWrite(LEDhr,0);
  analogWrite(LEDhl,0);
  analogWrite(LEDvr,0);
  analogWrite(LEDvl,0);


}

void loop() {


  if (client.available()) {
    String msg = client.readStringUntil('\n'); // Assumes newline-terminated messages
    //Serial.print("Received: ");
    //Serial.println(msg);
    int spaceIndex = msg.indexOf(' '); //find index of space in string
    //split string
    String firstPart = msg.substring(0, spaceIndex);
    String secondPart = msg.substring(spaceIndex + 1);
    int vx=firstPart.toInt();
    int vy=secondPart.toInt();
    drive(vx,vy,50);
    
  }
}

void drive(int vx,int vy,int timems)
{
  //0<|vx+vy|<255

  if (vy > 0){

    int vright=vy+vx;
    if (vright>255) {vright=255;}
    else if (vright <0) {vright=0;}
    int vleft=vy-vx;
    if (vleft>255) {vleft=255;}
    else if(vleft<0) {vleft=0;}

    analogWrite(PIN_MOTOR_RIGHT_DN ,vright);
    analogWrite(PIN_MOTOR_RIGHT_UP,0);
    analogWrite(PIN_MOTOR_LEFT_DN,vleft);
    analogWrite(PIN_MOTOR_LEFT_UP,0);
  }
  else if (vy <0 ){
    int vright=-vy-vx;
    if (vright>255) {vright=255;}
    else if (vright <0) {vright=0;}
    int vleft=-vy+vx;
    if (vleft>255) {vleft=255;}
    else if (vleft<0) {vleft=0;}

    analogWrite(PIN_MOTOR_RIGHT_DN,0);
    analogWrite(PIN_MOTOR_RIGHT_UP,vright);
    analogWrite(PIN_MOTOR_LEFT_DN,0);
    analogWrite(PIN_MOTOR_LEFT_UP,vleft);

  }
  else {
    analogWrite(PIN_MOTOR_RIGHT_UP,0);
    analogWrite(PIN_MOTOR_RIGHT_DN,0);
    analogWrite(PIN_MOTOR_LEFT_UP,0);
    analogWrite(PIN_MOTOR_LEFT_DN,0);

  }
  delay(timems);
}

