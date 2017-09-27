//SETUPS
#define LOOP_INTERVAL 100 //

//#define ROLE 0 //0 is A
#define ROLE 1 //1 is B
// set My ip ROLE 0 = 192, 168, 100, 220
// set My ip ROLE 1 = 192, 168, 100, 221


//Tap Threath (0x00 - 0xFF)
#define TAP_THREATH_PARAM 0x22// STD::0x82(130) HIGH::0x64(100) H+ :: 0x5B(91) SUPER.H::0x22(34) <- it is better in assembled 
#define DURATION_PARAM 0x8C//0x90

//#define  LATENT_PARAM 0x50 //Interval for double tap Detection
//#define WINDOW_PARAM 0x50 //Interbal restart detection after doubletap detection
#define  LATENT_PARAM 0x01 //Interval for double tap Detection
#define WINDOW_PARAM 0x01 //Interbal restart detection after doubletap detection



//to using ESP
#include <ESP8266WiFi.h>
#include <mem.h>

//to using WiFiConnection
#include <WiFiUdp.h>

//to using OSC
#include <OSCBundle.h>

//to connect with SPI
#include <SPI.h>
#define CS 5 //デフォルトから設定を変更して再定義(IO15 -> IO5)

//Register macros fore ADXL
#include "adxl.h"
#define RANGE_MAX 127 //normalize range_max




//// Variables ////

//WiFi Connection
WiFiUDP Udp;

//Seup WiFi
char ssid[] = "tone"; //  your network SSID (name)
char pass[] = "isana137";    // your network password

// IP Address of itself
//const IPAddress myIP(192, 168, 100, 220);      //固定IP
const IPAddress myIP(192, 168, 100, 221);      //固定IP
//const IPAddress myIP(192, 168, 43, 111);      //固定IP
const unsigned int receivePort = 57111;      //こちらで受信するポート

// IP Address of Server
//const IPAddress outIp(224, 0, 0, 1);      //相手(PC)のIP
const IPAddress outIp(192, 168, 100, 101);      //相手(PC)のIP
const unsigned int sendPort = 57137;         //こちらから送信するポート

// Router Setup
const IPAddress myGateWay(192, 168, 100, 1);
//const IPAddress myGateWay(192, 168, 43, 1);
const IPAddress mySubnet(255, 255, 255, 0);

// Counter to delay loop
int count;

//// Sensing ////
char values[10];
// RAW Parameter from ADXL
int16_t x, y, z;
//Normalized Values
float normalized_x;
float normalized_y;
float normalized_z;
//Variables to check tap
char test_single[1];
char test_double[1];
char tapType = 0;


void setup() {

  //// START ADXL ////

  //SPI INIT
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);

  // Set CS to High
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  attachInterrupt(0, tapCheck, RISING);
  // tapType = tapCheck();

  // Init ADXL345
  writeRegister(DATA_FORMAT, 0x01); // ±16g 10bit

  //Send the Tap and Double Tap Interrupts to INT1 pin
  writeRegister(INT_MAP, 0x9F);
  //Look for taps on the Z axis(01) only.
  writeRegister(TAP_AXES, 0x01);
  //Set the Tap Threshold to 3g
  writeRegister(THRESH_TAP, TAP_THREATH_PARAM); // The most weak = 0 , Themost Hard = 0xFF
  //Set the Tap Duration that must be reached
  writeRegister(DURATION, DURATION_PARAM);

  //100ms Latency before the second tap can occur.
  writeRegister(LATENT, LATENT_PARAM); //Interval for double tap Detection
  writeRegister(WINDOW, WINDOW_PARAM); //Interbal restart detection after doubletap detection

  //Enable the Single and Double Taps.
  writeRegister(INT_ENABLE, 0xE0);

  writeRegister(POWER_CTL, 0x08);  // 測定モード
  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register.


  //Start WiFi Connection
  wifiStart();


}

void loop() {

  //Reconnect WiFi :: if the connection was interrupts
  if (WiFi.status() == WL_NO_SSID_AVAIL ||
      WiFi.status() == WL_CONNECTION_LOST ||
      WiFi.status() == WL_DISCONNECTED) {

    Udp.stop();
    wifiStart();
  }


  //Increment count for timer
  count++;

  if (count > LOOP_INTERVAL) {
    count = 0;
    tapCheck();

    // DATAX0レジスタから6バイトを取得
    readRegister(DATAX0, 6, values);

    // 2Byteのデータを再構成
    x  = ((int16_t)values[1] << 8) | (int16_t)values[0];
    y  = ((int16_t)values[3] << 8) | (int16_t)values[2];
    z  = ((int16_t)values[5] << 8) | (int16_t)values[4];

    // rescale-xyz
    normalized_x = normalize(x,RANGE_MAX);
    normalized_y = normalize(y,RANGE_MAX);
    normalized_z = normalize(z,RANGE_MAX);


    // ログ出力
    Serial.print(normalized_x);
    Serial.print("\t");
    Serial.print(normalized_y);
    Serial.print("\t");
    Serial.println(normalized_z);
        
    if (tapType > 0) {
      if (tapType == 1) {
        Serial.println("SINGLE TAP");
        //OSCメッセージをつくる
        OSCMessage msg4("/tap");
        //数値をint型＝整数にする（数値はintかfloatのみ
        msg4.add(ROLE);
        //上のOSCメッセージをパケットにして送る
        Udp.beginPacket(outIp, sendPort);
        msg4.send(Udp);
        Udp.endPacket();
        msg4.empty();
        //パケット終わって、メッセージを空っぽにクリア
        delay(100);

      } else {
        Serial.println("DOUBLE TAP");
        //OSCメッセージをつくる
        OSCMessage msg5("/double_tap");
        //数値をint型＝整数にする（数値はintかfloatのみ）
        msg5.add(ROLE);
        //上のOSCメッセージをパケットにして送る
        Udp.beginPacket(outIp, sendPort);
//        msg5.send(Udp);
        Udp.endPacket();
        msg5.empty();
        delay(100);
      }
      detachInterrupt(0);
      delay(10);
      attachInterrupt(0, tapCheck, RISING);
      tapType = 0;
    }

    //sendOSC
    //X
    //OSCメッセージをつくる
    OSCMessage msg1("/ch1");
    //数値をint型＝整数にする（数値はintかfloatのみ）
    msg1.add(0);
    msg1.add((float)normalized_x);
    //上のOSCメッセージをパケットにして送る
    Udp.beginPacket(outIp, sendPort);
//    msg1.send(Udp);
    Udp.endPacket();
    msg1.empty();
    //パケット終わって、メッセージを空っぽにクリア

    //Y
    //OSCメッセージをつくる
    OSCMessage msg2("/ch2");
    //数値をint型＝整数にする（数値はintかfloatのみ）
    msg2.add(0);
    msg2.add((float)normalized_y);
    //上のOSCメッセージをパケットにして送る
    Udp.beginPacket(outIp, sendPort);
//    msg2.send(Udp);
    Udp.endPacket();
    msg2.empty();
    //パケット終わって、メッセージを空っぽにクリア


    //Z
    //OSCメッセージをつくる
    OSCMessage msg3("/ch3");
    //数値をint型＝整数にする（数値はintかfloatのみ）
    msg3.add(0);
    msg3.add((float)normalized_z);
    //上のOSCメッセージをパケットにして送る
    Udp.beginPacket(outIp, sendPort);
//    msg3.send(Udp);
    Udp.endPacket();
    msg3.empty();
    //パケット終わって、メッセージを空っぽにクリア
  }
}

void writeRegister(char registerAddress, char value) {
  // SPI開始時にCSをLOWにする
  digitalWrite(CS, LOW);
  // レジスタアドレス送信
  SPI.transfer(registerAddress);
  // レジスタに設定する値送信
  SPI.transfer(value);
  // SPI終了時にCSをHIGHにする
  digitalWrite(CS, HIGH);
}

void readRegister(char registerAddress, int16_t numBytes, char * values) {
  // 書き込みフラグを立てる
  char address = 0x80 | registerAddress;
  // 複数バイトフラグを立てる
  if (numBytes > 1)address = address | 0x40;
  // SPI開始時にSSをLOWにする
  digitalWrite(CS, LOW);
  // 読み出し先レジスタのアドレスを送信
  SPI.transfer(address);
  // 値の読み出し
  for (int16_t i = 0; i < numBytes; i++) {
    values[i] = SPI.transfer(0x00);
  }
  // SPI終了時にCSをHIGHにする
  digitalWrite(CS, HIGH);
}

void tapCheck(void) {
  //read tap_detection_registor and Clear the interrupts on the ADXL345
  readRegister(INT_SOURCE, 1, values);
  Serial.println(int(values[0]));
  //DOUBLE TAP detection (1<<5) == 0010 0000
  if (values[0] & (1<<5)) {
    tapType = 2;
  //SINGLE TAP detection (1<<6) == 0100 0000
  } else if (values[0] & (1<<6)) {
    tapType = 1;
  } else { // undetected
    tapType = 0;
  }
}

float normalize(float num,float range_max){
  float norm = num/range_max;
  return(norm);
}


//------------------------------------------------//
//Wifiスタート処理

void wifiStart() {
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.config(myIP, myGateWay, mySubnet);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  printWifiStatus();

  Udp.begin(receivePort);
}

//Wifiの状況をシリアルモニタで表示する（USBシリアルをつないだときのデバッグ用）
void printWifiStatus() {
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



//OSC受信の処理
void bundleReceive() {
  OSCMessage msgIN;
  int size;
  if ((size = Udp.parsePacket()) > 0) {
    while (size--)
      msgIN.fill(Udp.read());
    if (!msgIN.hasError()) {
      msgIN.route("/value", handleMessage);
    }
  }
}

//OSC受信したメッセージから、0番目のデータを整数で取り出す
void handleMessage(OSCMessage & msg, int addrOffset ) {
  int val = msg.getInt(0);
  Serial.println(val);
}
