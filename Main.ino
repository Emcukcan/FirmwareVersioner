#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "cert.h"
#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>

//geolocation


//FIREBASE
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#define API_KEY "AIzaSyCuZkZqN7bFkwUNeT8ghLgmC5wfj-mJQiA"
#define DATABASE_URL "https://encap-2f8c2-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define USER_EMAIL "t.susur@gmail.com"
#define USER_PASSWORD "200934005"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
unsigned long count = 0;





//////////////////////////////
//SPIFFS//////////////////////
#include <ArduinoJson.h>
#include "FS.h"
#include "SPIFFS.h"
#include "Free_Fonts.h"
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include "time.h"
#include <DNSServer.h>
#include <Wifi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>
#include <bms_comm.h>
#include <bmsequ_comm.h>
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <AsyncJson.h>

char VALUE [512] = {'\0'};  // 511 chars and the end terminator if needed make larger/smaller
String Output;  //Read output
float VoltageArraySPIFF[300];
float CurrentArraySPIFF[300];
float TempArraySPIFF[300];
String SerialNumber;
String WifiDirect;

double dailyCells[16];

String fileName;
int fileindex;
String Minute;
String Hour;
String Day;
String Month;
String FolderName;
int MinuteIndex;
String PublicIP;
String Coordinates;
////////////////////

#include <Wire.h>
#include <Adafruit_FT6206.h>
Adafruit_FT6206 ts = Adafruit_FT6206();
#include <Preferences.h>

Preferences preferences;


TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite Banner = TFT_eSprite(&tft);
TFT_eSprite BannerFrame = TFT_eSprite(&tft);
TFT_eSprite Dashboard = TFT_eSprite(&tft);
TFT_eSprite maxIcon_Icon = TFT_eSprite(&tft);
TFT_eSprite notificationIcon = TFT_eSprite(&tft);
TFT_eSprite wifiIcon = TFT_eSprite(&tft);
TFT_eSprite timeIcon = TFT_eSprite(&tft);
TFT_eSprite MainMenu = TFT_eSprite(&tft);
TFT_eSprite DashboardIcon = TFT_eSprite(&tft);
TFT_eSprite VoltageGraphIcon = TFT_eSprite(&tft);
TFT_eSprite CurrentGraphIcon = TFT_eSprite(&tft);
TFT_eSprite TempGraphIcon = TFT_eSprite(&tft);
TFT_eSprite CellIcon = TFT_eSprite(&tft);
TFT_eSprite BalanceIcon = TFT_eSprite(&tft);
TFT_eSprite EnergyIcon = TFT_eSprite(&tft);
TFT_eSprite SettingsIcon = TFT_eSprite(&tft);
TFT_eSprite CellMonIcon = TFT_eSprite(&tft);
TFT_eSprite LandingIcon = TFT_eSprite(&tft);
TFT_eSprite EncapIcon = TFT_eSprite(&tft);
TFT_eSprite DischargeIcon = TFT_eSprite(&tft);
TFT_eSprite ChargeIcon = TFT_eSprite(&tft);
TFT_eSprite RTEIcon = TFT_eSprite(&tft);
TFT_eSprite CycleIcon = TFT_eSprite(&tft);

TFT_eSprite eraseIcon = TFT_eSprite(&tft);
TFT_eSprite energyIcon = TFT_eSprite(&tft);

TFT_eSprite SwitchOnIcon = TFT_eSprite(&tft);
TFT_eSprite SwitchOffIcon = TFT_eSprite(&tft);

TFT_eSprite GraphFrame = TFT_eSprite(&tft);
TFT_eSprite CellFrame = TFT_eSprite(&tft);
TFT_eSprite LandingFrame = TFT_eSprite(&tft);
TFT_eSprite LandingIconFrame = TFT_eSprite(&tft);
TFT_eSprite AlarmsFrame = TFT_eSprite(&tft);
TFT_eSprite NumpadFrame = TFT_eSprite(&tft);
TFT_eSprite EnergiesFrame = TFT_eSprite(&tft);
TFT_eSprite AlarmListFrame = TFT_eSprite(&tft);

TFT_eSprite TabFrameGraph = TFT_eSprite(&tft);
TFT_eSprite TabFrameAlarms = TFT_eSprite(&tft);
TFT_eSprite TabIcon = TFT_eSprite(&tft);
TFT_eSprite NotificationFrame = TFT_eSprite(&tft);
TFT_eSprite SOCCalibrateFrame = TFT_eSprite(&tft);
TFT_eSprite ConfirmationFrame = TFT_eSprite(&tft);
TFT_eSprite MosfetsFrame = TFT_eSprite(&tft);
TFT_eSprite InfoFrame = TFT_eSprite(&tft);
TFT_eSprite LoadingFrame = TFT_eSprite(&tft);
TFT_eSprite LoadingBarFrame = TFT_eSprite(&tft);
TFT_eSprite UploadingFirmwareFrame = TFT_eSprite(&tft);


WiFiUDP Udp;
WebServer server(80);

#include "Header_2.h"
#include "maxIcon.h"
#include "minIcon.h"
#include "delta.h"
#include "voltage.h"
#include "current.h"
#include "temp.h"
#include "notification.h"
#include "wifi.h"
#include "nowifi.h"
#include "clock.h"
#include "dashboard.h"
#include "graph.h"
#include "cell.h"
#include "balance.h"
#include "energy.h"
#include "settings.h"
#include "battery.h"
#include "circles.h"
#include "Encap.h"
#include "tab.h"
#include "tabMini.h"
#include "alarms.h"
#include "calibration.h"
#include "erase.h"
#include "energyCD.h"
#include "switchon.h"
#include "switchoff.h"
#include "advance.h"

//ota
#include "serverIndex.h"
#include "loginIndex.h"



/////////////////backlight

const int ledPin = 23;
// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;



int reading = 0; // Value to be displayed
int d = 0; // Variable used for the sinewave test waveform
bool range_error = 0;
uint32_t runTime = -99999;       // time for next update

//randoms
float maxVolt;
float minVolt;

//WIFI SETTINGS

String SSIDName = "ENCAP Controller";
String Port = "2001";
String MAC = "No Connection";
String IP = "192.168.4.1";
bool wifistatus = false;
int packetSize = 0;


String AlarmArray[28] = {"CellHigh", "CellHigh2", "CellLow", "CellLow2", "SumHigh", "SumHigh2", "SumLow", "SumLow2",
                         "ChTHigh", "ChTHigh2", "ChTLow", "ChTLow2", "DsTHigh", "DsTHigh2",
                         "DsTLow", "DsTLow2",
                         "ChOCurr", "ChOCurr2", "DsOCurr", "DsOCurr2", "SOCHigh1", "SOCHigh2", "SOCLow1", "SOCLow2",
                         "DiffVolt1", "DiffVolt2", "DiffTemp1", "DiffTemp2"
                        };

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;   //10 SEC

//graph variables
bool display1 = true;
bool update1 = true;
double VoltageArray[60];
double CurrentArray[60];
double TempArray[60];
#define RED       0xF800
#define WHITE     0xFFFF
#define BLACK     0x0000
#define DKBLUE    0x000D
double ox = -999, oy = -999;
double x, y;

//Numpad variables

float displayValue = 0;
bool updateStarted = false;

//PREFERENCE
float ChargeEnergy;
float DischargeEnergy;
float ChargeEnergyFixed;
float DischargeEnergyFixed;



float DailyChargeEnergy[24];
float DailyDischargeEnergy[24];

// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

//TASKS////////////////////////
TaskHandle_t TOUCH_SCREEN;
TaskHandle_t UDP;
TaskHandle_t BMS_COMM;
TaskHandle_t LOG;
TaskHandle_t FIREBASE;



////TIMER VARIABLES//////////////////
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 14400;
const int   daylightOffset_sec = 3600;

char timeHour[4];
char timeMin[4];
char timeSec[4];
char timeDay[4];
char timeMonth[4];
char timeYear[10];

String timeHourS;
String timeMinS;
String timeSecS;
String timeDayS;
String timeMonthS;
String timeYearS;

String TimeString = "No Data";
String DateString;
String DateDir_Voltage;
String DateDir_Current;
String DateDir_Temp;
String MainDir;

//UI Parameters
int PageNumber = 1;
int PageLevel = 0;
int SleepCounter = 0;
bool EEPROMbusy = false;
bool balancerState = false;

unsigned long previousMillisUI = 0;
unsigned long intervalUI = 250;
unsigned long currentMillisUI = 0;

bool numpadEnable = false;
String numpadValue = "";
bool notificationEnable = false;
int notificationCounter = 0;
bool CommandStatus = false;


//BMS Parameters.....


int restartCounter = 0;
float HCD = 0;
float HCC = 0;
float HVC = 0;
float LVC = 0;
float RC = 0;
float RV = 0;
float BV = 0;
float BD = 0;
float SOC_CAL_VAL = 0;

bool HCDEnable = false;
bool HCCEnable = false;
bool HVCEnable = false;
bool LVCEnable = false;
bool RCEnable = false;
bool RVEnable = false;
bool BVEnable = false;
bool BDEnable = false;
bool SOCEnable = false;

bool HCC_HCD = false;
bool HVC_LVC = false;
bool RC_RV = false;
bool BV_BD = false;
bool SOC_CAL = false;

bool ReadAllParameters = false;

//MOSFET SETTING
bool CHARGE_SET = false;
bool DISCHARGE_SET = false;
bool BALANCE_SET = false;


bool notificationStatus = false;
bool parameterSet = false;
int alarmNo = 0;
bool confirmation = false;
bool loading = false;
int loadingPercentage = 0;

bool WriteCharge = false;
bool WriteDischarge = false;
bool WriteBalance = false;

//equalizer parameters


bool EqualizerMasterInput = true;
bool EqulizerStatus = false;
bool EQUON = false;
bool EQUOFF = false;

bool EQUON2 = false;
bool EQUOFF2 = false;

#define FORMAT_SPIFFS_IF_FAILED true
StaticJsonDocument<6000> doc;
JsonObject Voltage = doc.createNestedObject("Voltage");
/////////////////////////////
//GITHUB UPDATE

String FirmwareVer = {
  "2.7"
};


#define URL_fw_Version "https://raw.githubusercontent.com/Emcukcan/FirmwareVersioner/master/bin_version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/Emcukcan/FirmwareVersioner/master/fw.bin"

//void firmwareUpdate();
//int FirmwareVersionCheck();

unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousMillis_2 = 0;
const long interval = 10000;
const long mini_interval = 1000;







////////////////////////////

void setup() {

  tft.init();
  pinMode(TFT_BL, OUTPUT);


  digitalWrite(TFT_BL, 128);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  Serial.begin(9600);
  while (!Serial) continue;
  Serial.setTimeout(250);
  //BMS SETUP

  RXD3 = 25;
  TXD3 = 26;

  //  //  module
  RXD2 = 32;
  TXD2 = 33;

  Serial1.begin(9600, SERIAL_8N1, RXD3, TXD3);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //Backlight
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 255);




  preferences.begin("my-app", false);
  ChargeEnergyFixed = preferences.getFloat("ChargeEnergyFixed", 0);
  DischargeEnergyFixed = preferences.getFloat("DischargeEnergyFixed", 0);
  ChargeEnergy = preferences.getFloat("ChargeEnergy", 0);
  DischargeEnergy = preferences.getFloat("DischargeEnergy", 0);
  SerialNumber = preferences.getString("SerialNumber", "ENC10482000001");

  HCD = preferences.getFloat("HCD", 0);
  HCC = preferences.getFloat("HCC", 0);
  HVC = preferences.getFloat("HVC", 0);
  LVC = preferences.getFloat("LVC", 0);
  RC = preferences.getFloat("RC", 0);
  RV = preferences.getFloat("RV", 0);
  BV = preferences.getFloat("BV", 0);
  BD = preferences.getFloat("BD", 0);
  restartCounter = preferences.getInt("RC", 0);
  preferences.end();


  preferences.begin("my-app", false);
  restartCounter++;
  preferences.putInt("RC", restartCounter);
  preferences.end();





  //CREATING TASK FOR UDP Comm_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    UDP_CODE,   /* Task function. */
    "UDP",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    97,           /* priority of the task */
    &UDP,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);



  //CREATING TASK FOR BMS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BMS_COMM_CODE,   /* Task function. */
    "BMS_COMM",     /* name of task. */
    30000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    100,           /* priority of the task */
    &BMS_COMM,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);

  drawLandingPage();

  //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    TOUCH_SCREEN_CODE,   /* Task function. */
    "TOUCH_SCREEN",     /* name of task. */
    5000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    96,           /* priority of the task */
    &TOUCH_SCREEN,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR BMS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    LOG_CODE,   /* Task function. */
    "LOG",     /* name of task. */
    30000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    11,           /* priority of the task */
    &LOG,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);
  //
  //CREATING TASK FOR BMS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    FIREBASE_CODE,   /* Task function. */
    "FIREBASE",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    12,           /* priority of the task */
    &FIREBASE,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);

}

void LOG_CODE( void * pvParameters ) {

  for (;;) {
    // repeatedCall();
    delay(500);
  }
}



void FIREBASE_CODE( void * pvParameters ) {

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Firebase task is waiting for internet connection...");
    delay(500);
  }


  String jsonBuffer;

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);

  //WiFi reconnect timeout (interval) in ms (10 sec - 5 min) when WiFi disconnected.
  config.timeout.wifiReconnect = 10 * 1000;

  //Socket connection and SSL handshake timeout in ms (1 sec - 1 min).
  config.timeout.socketConnection = 10 * 1000;

  //Server response read timeout in ms (1 sec - 1 min).
  config.timeout.serverResponse = 10 * 1000;

  //RTDB Stream keep-alive timeout in ms (20 sec - 2 min) when no server's keep-alive event data received.
  config.timeout.rtdbKeepAlive = 45 * 1000;

  //RTDB Stream reconnect timeout (interval) in ms (1 sec - 1 min) when RTDB Stream closed and want to resume.
  config.timeout.rtdbStreamReconnect = 1 * 1000;

  //RTDB Stream error notification timeout (interval) in ms (3 sec - 30 sec). It determines how often the readStream
  //will return false (error) when it called repeatedly in loop.
  config.timeout.rtdbStreamError = 3 * 1000;






  // jsonBuffer = payload
  //  JSONVar myObject = JSON.parse(jsonBuffer);
  //  // JSON.typeof(jsonVar) can be used to get the type of the var
  //  if (JSON.typeof(myObject) == "undefined") {
  //    Serial.println("Parsing input failed!");
  //    return;
  //  }
  //
  //  Serial.print("JSON object = ");
  //  Serial.println(myObject);
  //  Serial.print(": ");
  //  Serial.println(myObject["main"]["temp"]);
  //  Serial.print("Pressure: ");





  for (;;) {

    if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0) && WiFi.status() == WL_CONNECTED)
    {
      vTaskSuspend(UDP);
      vTaskSuspend(LOG);
      sendDataPrevMillis = millis();
      esp_task_wdt_init(60, false);



      String FirebaseTimeStamp = "/Monitoring/Version1/" + SerialNumber + "/TimeStamp";
      String FirebaseDateStamp = "/Monitoring/Version1/" + SerialNumber + "/DateStamp";
      String FirebaseCoordinates = "/Monitoring/Version1/" + SerialNumber + "/Coordinates";
      String FirebaseFirmwareVersion = "/Monitoring/Version1/" + SerialNumber + "/Firmware";
      String FirebaseTerminalVoltage = "/Monitoring/Version1/" + SerialNumber + "/TerminalVoltage";
      String FirebaseTerminalCurrent = "/Monitoring/Version1/" + SerialNumber + "/TerminalCurrent";
      String FirebaseCellTemp = "/Monitoring/Version1/" + SerialNumber + "/CellTemp";
      String FirebaseStateOfCharge = "/Monitoring/Version1/" + SerialNumber + "/StateOfCharge";
      String FirebaseMaxCell = "/Monitoring/Version1/" + SerialNumber + "/MaxCell";
      String FirebaseMinCell = "/Monitoring/Version1/" + SerialNumber + "/MinCell";
      String FirebaseChargeEnergy = "/Monitoring/Version1/" + SerialNumber + "/ChargeEnergy";
      String FirebaseDischargeEnergy = "/Monitoring/Version1/" + SerialNumber + "/DischargeEnergy";



      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseTimeStamp.c_str()), TimeString) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseDateStamp.c_str()), DateString) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseCoordinates.c_str()), Coordinates) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseFirmwareVersion.c_str()), FirmwareVer) ? "ok" : fbdo.errorReason().c_str());

      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseTerminalVoltage.c_str()), String(BMS.sum_voltage * 0.1)) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseTerminalCurrent.c_str()), String((BMS.current - 30000) * 0.1)) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseCellTemp.c_str()), String(BMS.max_cell_temp - 40)) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseStateOfCharge.c_str()), String(BMS.SOC * 0.1)) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseMaxCell.c_str()), String(BMS_equ.max_cell_volt_equ * 0.001)) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseMinCell.c_str()), String(BMS_equ.min_cell_volt_equ * 0.001)) ? "ok" : fbdo.errorReason().c_str());

      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseChargeEnergy.c_str()), String(ChargeEnergy)) ? "ok" : fbdo.errorReason().c_str());
      Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseDischargeEnergy.c_str()), String(DischargeEnergy)) ? "ok" : fbdo.errorReason().c_str());




      Serial.println();

      //      Serial.printf("Set bool... %s\n", Firebase.setBool(fbdo, F("/test/bool"), count % 2 == 0) ? "ok" : fbdo.errorReason().c_str());
      //      Serial.printf("Set float... %s\n", Firebase.setFloat(fbdo, F("/test/float"), count + 10.2) ? "ok" : fbdo.errorReason().c_str());
      //      Serial.printf("Set double... %s\n", Firebase.setDouble(fbdo, F("/test/double"), count + 35.517549723765) ? "ok" : fbdo.errorReason().c_str());
      //      Serial.printf("Set string... %s\n", Firebase.setString(fbdo, F("/test/string"), "Hello World!") ? "ok" : fbdo.errorReason().c_str());


      esp_task_wdt_init(60, true);
      vTaskResume(UDP);
      vTaskResume(LOG);
    }
    delay(100);
  }
}


void loop() {
  currentMillisUI = millis();
  if (currentMillisUI - previousMillisUI >= intervalUI) {
    //Simulated Values
    float maxVolt = random(260, 270) / 100.0,
          SOC = random(80, 91),
          minVolt = random(240, 260) / 100.0,
          maxDiff = maxVolt - minVolt,
          TerminalVolt = 45,
          TerminalCurr = random(1, 31) ,
          TerminalTemp = random(20, 25);




    // not used in this example `


    // BACKLIGHT COTROL
    if (SleepCounter < 360) {
      SleepCounter++;
      if (SleepCounter == 300) {
        ledcWrite(ledChannel, 100);
      }
      if (SleepCounter == 359) {
        ledcWrite(ledChannel, 5);
      }
    }
    if (SleepCounter == 1) {
      ledcWrite(ledChannel, 255);
    }


    //Page manager//

    switch (PageNumber) {
      case 0:
        drawMainMenu();
        drawBanner(TimeString, DateString, "Main Menu", wifistatus, alarmNo);
        break;

      case 1:
        drawBanner(TimeString, DateString, "Dashboard", wifistatus, alarmNo);

        drawDashboard(BMS.SOC * 0.1, BMS_equ.max_cell_volt_equ * 0.001, BMS_equ.min_cell_volt_equ * 0.001,
                      BMS_equ.max_cell_volt_equ * 0.001 - BMS_equ.min_cell_volt_equ * 0.001, BMS.sum_voltage * 0.1,
                      (BMS.current - 30000) * 0.1, BMS.max_cell_temp - 40);
        break;

      case 2:
        drawOptimizerGraph (GraphFrame, VoltageArray, "Graph", BMS.sum_voltage * 0.1);
        drawBanner(TimeString, DateString, "Graphs", wifistatus, alarmNo);
        drawGraphTab(true, false, false);

        break;
      case 3:
        drawOptimizerGraph (GraphFrame, CurrentArray, "Graphs", (BMS.current - 30000) * 0.1);
        drawBanner(TimeString, DateString, "Graphs", wifistatus, alarmNo);
        drawGraphTab(false, true, false);

        break;

      case 4:
        drawOptimizerGraph (GraphFrame, TempArray, "Graphs", BMS.max_cell_temp - 40);
        drawBanner(TimeString, DateString, "Graphs", wifistatus, alarmNo);
        //drawBarGraph(GraphFrame, dailyCells, TOLGA_RED, "Cell Values") ;
        //drawBarGraph(BarGraph, dailyGenerator, TOLGA_GREEN, "Daily Generator Energy (kWh)");

        drawGraphTab(false, false, true);

        break;

      case 5:

        drawBanner(TimeString, DateString, "Cell Volts", wifistatus, alarmNo);
        drawCell();

        break;

      case 6:
        drawBanner(TimeString, DateString, "Alarms", wifistatus, alarmNo); //Current alarms
        drawAlarmFrame(numpadEnable, true, false, false, false, 6, notificationEnable, notificationStatus);
        break;

      case 7:
        drawBanner(TimeString, DateString, "Alarms", wifistatus, alarmNo); //Cell alarms
        drawAlarmFrame(numpadEnable, false, true, false, false, 7, notificationEnable, notificationStatus);
        break;

      case 8:
        drawBanner(TimeString, DateString, "Alarms", wifistatus, alarmNo); //Rates alarms
        drawAlarmFrame(numpadEnable, false, false, true, false, 8, notificationEnable, notificationStatus);
        break;

      case 9:
        drawBanner(TimeString, DateString, "Alarms", wifistatus, alarmNo); //Balance alarms
        drawAlarmFrame(numpadEnable, false, false, false, true, 9, notificationEnable, notificationStatus);
        break;



      case 10:
        drawBanner(TimeString, DateString, "Energy", wifistatus, alarmNo); //Balance alarms
        drawEnergies(ChargeEnergy, DischargeEnergy, confirmation);
        break;

      case 11:
        drawBanner(TimeString, DateString, "Alarm List", wifistatus, alarmNo); //Balance alarms
        drawAlarmsList();
        break;

      case 12:
        drawBanner(TimeString, DateString, "Calibration", wifistatus, alarmNo); //Balance alarms
        drawSOCCalibrate(numpadEnable, notificationEnable);
        break;

      case 13:
        drawBanner(TimeString, DateString, "Mosfets", wifistatus, alarmNo); //Balance alarms
        drawMosfets(notificationEnable, notificationStatus);
        break;


      case 14:
        drawBanner(TimeString, DateString, "Info", wifistatus, alarmNo); //Balance alarms
        drawInfo();
        break;

      case 15:
        ledcWrite(ledChannel, 255);
        drawBanner(TimeString, DateString, "Update", wifistatus, alarmNo); //Balance alarms
        drawUpload();
        break;




      default:
        drawBanner(TimeString, DateString, "Dashboard", wifistatus, alarmNo);
        drawDashboard(BMS.SOC * 0.1, BMS_equ.max_cell_volt_equ * 0.001, BMS.min_cell_volt * 0.001,
                      BMS.max_cell_volt * 0.001 - BMS.min_cell_volt * 0.001, BMS.sum_voltage * 0.1,
                      (BMS.current - 30000) * 0.1, BMS.cell_temperature[0] - 40);
        break;
    }

    previousMillisUI = currentMillisUI;
  }
  server.handleClient();
  delay(100);
}


void BMS_COMM_CODE( void * pvParameters ) {
  unsigned long previousMillisBMS = 0;
  unsigned long intervalBMS = 1000;
  unsigned long currentMillisBMS = 0;

  for (;;) {
    currentMillisBMS = millis();

    if ((currentMillisBMS - previousMillisBMS) >= intervalBMS && (!HCC_HCD)   && (!HVC_LVC)  && (!RC_RV)
        && (!BV_BD) && (!SOC_CAL)   && (!CHARGE_SET) && (!DISCHARGE_SET) && (!BALANCE_SET)) {

      float period = (currentMillisBMS - previousMillisBMS) * 0.001;

      //  Serial.println("Equalizer:" + String(EqulizerStatus));


      //EQUALIZER
      EQUON = (BMS.current - 30000) <= 0;
      getBatteryParameters();
      EQUOFF = (BMS.current - 30000) > 0;

      delay(1000);

      //GET BALANCER STATE
      for (int i = 0; i < 16; i++) {
        balancerState = BMS.cell_balance[i] || balancerState;
      }

      //GET ALARMS
      alarmNo = 0;
      for (int i = 0; i < 28; i++) {
        if (BMS.error[i] == true) {
          alarmNo++;
        }
      }

      if (alarmNo == 13 || alarmNo == 43 || alarmNo == 12 || alarmNo == 14) {
        alarmNo = 0;
      }

      //EQUALIZER2


      if (EQUON > EQUON2 && EqualizerMasterInput) {
        for (int i = 0; i < 20; i++) {
          if (balance_turn_on()) {
            EQUON2 = true;
            EQUOFF2 = false;
            Serial.println("Equalizer is on");
            EqulizerStatus = true;
            for (int i = 0; i < 20; i++) {
              send_balance_cur(8, BMS_Equ);
            }
            break;

          }
        }

      }

      if (EQUOFF > EQUOFF2) {
        for (int i = 0; i < 20; i++) {
          if (balance_turn_off()) {
            Serial.println("Equalizer is off");
            EqulizerStatus = false;
            EQUON2 = false;
            EQUOFF2 = true;
            break;
          }
        }
      }

      //Energy Calculations..........

      if (!EEPROMbusy && period != 0 && rule1 && rule2) {

        if ((BMS.current - 30000) * 0.1 > 1  && BMS.sum_voltage * 0.1 > 40 ) {
          ChargeEnergy = ChargeEnergy + ((BMS.sum_voltage * 0.1 * (BMS.current - 30000) * 0.1) / (3600 / period) * 0.001);
          ChargeEnergyFixed = ChargeEnergyFixed + ((BMS.sum_voltage * 0.1 * (BMS.current - 30000) * 0.1) / (3600 / period) * 0.001);
          //          Serial.println("Charging...");
          //          Serial.println("Energy per sec");
          //          Serial.print((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001, 4);
          //          Serial.println("kW");
        }



        if ((BMS.current - 30000) * 0.1 < -1 && BMS.sum_voltage * 0.1 > 40) {
          DischargeEnergy = DischargeEnergy + ((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001);
          DischargeEnergyFixed = DischargeEnergyFixed + ((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001);
          //          Serial.println("Discharging...");
          //          Serial.println("Energy per sec");
          //          Serial.print((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001);
          //          Serial.println("kW");

        }

        preferences.begin("my-app", false);
        preferences.putFloat("ChargeEnergy", ChargeEnergy);
        preferences.putFloat("DischargeEnergy", DischargeEnergy);
        preferences.putFloat("ChargeEnergyFixed", ChargeEnergyFixed);
        preferences.putFloat("DischargeEnergyFixed", DischargeEnergyFixed);
        preferences.end();
      }
      previousMillisBMS = currentMillisBMS;
    }

    else  //Parameter Set
    {

      if (CHARGE_SET) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_charge_mode(WriteCharge, BMS_Set_blue);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("Command Status:" + String(CommandStatus));
            CommandStatus = false;
            CHARGE_SET = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        notificationEnable = true;
        CHARGE_SET = false;
      }

      ////

      else if (DISCHARGE_SET) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_discharge_mode(WriteDischarge, BMS_Set_blue);

          Serial.println("Command Status:" + String(CommandStatus));
          notificationStatus = CommandStatus;
          if (CommandStatus) {

            CommandStatus = false;
            DISCHARGE_SET = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        notificationEnable = true;
        DISCHARGE_SET = false;
      }

      //
      else if (HCC_HCD) {
        CommandStatus = false;
        delay(100);
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_chargecurrent_BMS(HCC, HCD, BMS_Set);
          CommandStatus = param_save;
          Serial.println("Command Status:" + String(CommandStatus));
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            CommandStatus = false;
            HCC_HCD = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        //        Serial.println("param_save:");
        //        Serial.println(param_save);
        notificationEnable = true;
        HCC_HCD = false;
      }

      //SOC
      else if (SOC_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_soc_BMS(SOC_CAL_VAL, BMS_Set);

          Serial.println("Command Status:" + String(CommandStatus));
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            CommandStatus = false;
            SOC_CAL = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        notificationEnable = true;
        SOC_CAL = false;
      }

      //



      //HVC
      else if (HVC_LVC) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_cellvolt_BMS(HVC, LVC, BMS_Set) ;

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            HVC_LVC = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        notificationEnable = true;
        HVC_LVC = false;
      }

      //


      //RC RV
      else if (RC_RV) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_rated_BMS(RC, RV, BMS_Set);

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            RC_RV = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        notificationEnable = true;
        RC_RV = false;
      }

      //


      //BV-BD
      else if (BV_BD) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_balancevolt_BMS(BV, BD * 0.001, BMS_Set);

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            BV_BD = false;
            break;
          }
          else {
            Serial.println("Error");
          }
        }
        notificationEnable = true;
        BV_BD = false;
      }

      //
    }
    delay(100);
  }
}

void UDP_CODE( void * pvParameters ) {

  unsigned int localUdpPort = 2001;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char Dummy[255];
  char  replyPacket[] = "";  // a reply string to send back
  WiFiServer serverWifi(81);

  WiFiManager wifiManager;
  String isim = "ENCAP Controller";


  // forget network
  //          wifiManager.resetSettings();
  //          Serial.println("forget password");
  //          delay(1000);
  //      ESP.restart();


  //Wifi interface____________________________________________________________________________________________________________
  wifiManager.setCustomHeadElement("<style> h2 {padding: 30px;text-align: center;background: #0099ff;color: white;font-size: 30px; }footer {position: fixed;left: 0; bottom: 0;width: 100%;background-color: #0099ff;color: white;text-align: center;</style><h2>SiriusPlus+ Interface</h2><footer><p>Amber&Waseem Software Development and Design</p><p></p></footer>");
  wifiManager.setTimeout(60);
  if (!wifiManager.autoConnect((const char*)isim.c_str())) {
    Serial.println("failed to connect and hit timeout");
    delay(250);
    Serial.println("re-connecting.....");
    //Serial.println("resetting");
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    WiFi.begin();
    //ESP.restart();
  }
  //if you get here you have connected to the WiFi
  Serial.println("Connection is established:)");
  wifiManager.autoConnect((const char*)isim.c_str());
  //  IPAddress subnet(255, 255, 255, 0);
  //
  //  IPAddress localGateway(192, 168, 60, 1); //hardcoded
  Serial.println((WiFi.status()));
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  delay(250);

  SSIDName = WiFi.SSID();
  IP = WiFi.localIP().toString().c_str();
  Port = localUdpPort;
  MAC = WiFi.macAddress();
  wifistatus = true;
  int len;

  unsigned long previousMillis = 0;
  unsigned long interval = 5000;
  unsigned long currentMillis = 0;


  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();



  if (FirmwareVersionCheck()) {
    firmwareUpdate();
  }

  GetPublicIP();



  for (;;) {
    currentMillis = millis();
    // if WiFi is down, try reconnecting
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
      Serial.print(millis());
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      SSIDName = WiFi.SSID();
      IP = "No Connection";
      Port = localUdpPort;
      MAC = "No Connection";

      previousMillis = currentMillis;

    }
    else {
      if ((millis() - lastTime) > timerDelay) {
        // Check WiFi connection status
        if (printLocalTime()) {
          lastTime = millis();
        }
      }


      //UDP COMMANDS/////////////////////////////


      int packetSize = Udp.parsePacket();
      if (packetSize)
      {
        memset(incomingPacket, 0, sizeof(incomingPacket));
        int len = Udp.read(incomingPacket, 255);
        if (len > 0)
        {
          incomingPacket[len] = 0;
        }


        preferences.begin("my-app", false);
        ChargeEnergyFixed = preferences.getFloat("ChargeEnergyFixed", 0);
        DischargeEnergyFixed = preferences.getFloat("DischargeEnergyFixed", 0);
        ChargeEnergy = preferences.getFloat("ChargeEnergy", 0);
        DischargeEnergy = preferences.getFloat("DischargeEnergy", 0);
        SerialNumber = preferences.getString("SerialNumber", "ENC10482000001");

        HCD = preferences.getFloat("HCD", 0);
        HCC = preferences.getFloat("HCC", 0);
        HVC = preferences.getFloat("HVC", 0);
        LVC = preferences.getFloat("LVC", 0);
        RC = preferences.getFloat("RC", 0);
        RV = preferences.getFloat("RV", 0);
        BV = preferences.getFloat("BV", 0);
        BD = preferences.getFloat("BD", 0);
        restartCounter = preferences.getInt("RC", 0);
        preferences.end();

        //SET SERIAL
        int SR_INDEX_START = String(incomingPacket).indexOf("SETSR");
        int SR_INDEX_END = String(incomingPacket).indexOf("#");
        SerialNumber = String(incomingPacket).substring(SR_INDEX_START + 5, SR_INDEX_END);




        if (SR_INDEX_START != -1 && SR_INDEX_END != -1) {
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.print("Serialnumber:" + SerialNumber);
          Udp.endPacket();
          for (int i = 0; i < 10; i++) {
            incomingPacket[i] = 's';
          }
        }






        //PINGING BMS
        int SLAVE_INDEX_START = String(incomingPacket).indexOf("PING");
        int SLAVE_INDEX_END = String(incomingPacket).indexOf("#");
        if (SLAVE_INDEX_START != -1 && SLAVE_INDEX_END != -1) {
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.print("PONG_BMS");
          Udp.endPacket();
          for (int i = 0; i < 10; i++) {
            incomingPacket[i] = 's';
          }
        }

        //MEASUREMENT BMS
        SLAVE_INDEX_START = String(incomingPacket).indexOf("GET_MEAS");
        SLAVE_INDEX_END = String(incomingPacket).indexOf("#");
        if (SLAVE_INDEX_START != -1 && SLAVE_INDEX_END != -1) {
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());


          String MEAS = "TV" + String(BMS.sum_voltage) +  "/TC" + String(BMS.current) + "/SOC" +  String(BMS.SOC) + "/TEMP" +  String(BMS.max_cell_temp) +
                        "/MAX" + String(BMS_equ.max_cell_volt_equ) + "/MIN" + String(BMS_equ.min_cell_volt_equ) +  "/STT" + String(BMS.state) +
                        "/CH" + String(BMS.charge) + "/DSH" + String(BMS.discharge) + "/LF" + String(BMS.bms_life) +
                        "/RC" + String(BMS.rem_cap) + "/BL" + String(balancerState) + "/#";


          Udp.print(MEAS);
          Udp.endPacket();
          for (int i = 0; i < 10; i++) {
            incomingPacket[i] = 's';
          }
        }


        //CELLS BMS
        SLAVE_INDEX_START = String(incomingPacket).indexOf("GET_CELLS");
        SLAVE_INDEX_END = String(incomingPacket).indexOf("#");
        if (SLAVE_INDEX_START != -1 && SLAVE_INDEX_END != -1) {

          String CellString = "";

          Serial.println(BMS_equ.cell_voltage_equ[0]);


          for (int i = 0; i < 16; i++) {
            CellString = CellString + String(BMS_equ.cell_voltage_equ[i]) + "/";
          }
          CellString = "CELLS" + CellString + "#";

          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.print(CellString);
          Udp.endPacket();
          for (int i = 0; i < 10; i++) {
            incomingPacket[i] = 's';
          }
        }

        //ALARMS BMS
        SLAVE_INDEX_START = String(incomingPacket).indexOf("GET_ALARMS");
        SLAVE_INDEX_END = String(incomingPacket).indexOf("#");
        if (SLAVE_INDEX_START != -1 && SLAVE_INDEX_END != -1) {

          String AlarmsString = "";
          for (int i = 0; i < 28; i++) {
            AlarmsString = AlarmsString + String(BMS.error[i]) + "/";
          }
          AlarmsString = "ALARMS" + AlarmsString + "#";

          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.print(AlarmsString);
          Udp.endPacket();
          for (int i = 0; i < 10; i++) {
            incomingPacket[i] = 's';
          }
        }

        //ENERGY BMS
        SLAVE_INDEX_START = String(incomingPacket).indexOf("GET_ENERGY");
        SLAVE_INDEX_END = String(incomingPacket).indexOf("#");
        if (SLAVE_INDEX_START != -1 && SLAVE_INDEX_END != -1) {

          String EnergyString = "";
          EnergyString = "CH" + String(ChargeEnergy) + "/DS" + String(DischargeEnergy) + "/#";
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.print(EnergyString);
          Udp.endPacket();
          for (int i = 0; i < 10; i++) {
            incomingPacket[i] = 's';
          }
        }

        delay(10);
      }
      wifistatus = (WiFi.status() == WL_CONNECTED);
      delay(100);
    }
  }
}

//Touch Task execution________________________________________________________________________________________________________________
void TOUCH_SCREEN_CODE( void * pvParameters ) {
  int TouchArray[100];
  int TouchArray_vertical[100];
  int touch_iteration = 0;
  bool action_done = false;
  bool button_done = false;
  bool settings_page = false;



  // TOUCH SETUP=============================
  if (!ts.begin(18, 19, 40)) {
    Serial.println("Couldn't start touchscreen controller");
    while (true);
  }
  for (int i = 0; i < 100; i++) {
    TouchArray[i] = 0;
  }
  for (;;) {
    if (ts.touched()) {
      TS_Point p = ts.getPoint();

      p.x = map(p.x, 0, 320, 0, 320);
      p.y = map(p.y, 0, 480, 0, 480);
      int y = tft.height() - p.x;
      int x = p.y;
      int cursor_after = p.y;
      int cursor_after_vertical = p.x;
      TouchArray[touch_iteration] = cursor_after;
      TouchArray_vertical[touch_iteration] = cursor_after_vertical;

      ///SWIPE RIGHT
      if (TouchArray[touch_iteration] - TouchArray[0] > 50 && !action_done && TouchArray[touch_iteration] != 0 && touch_iteration > 10) {
        Serial.println("Swipe Right");
        action_done = true;
      }
      ///SWIPE LEFT
      if (TouchArray[0] - TouchArray[touch_iteration] > 50 && !action_done  && TouchArray[touch_iteration] != 0 && touch_iteration > 10) {
        Serial.println("Swipe Left");
        action_done = true;
      }


      ///Touch wake up
      if (y >= 0 && y <= 320 && x >= 0 && x <= 480) {
        SleepCounter = 0;
        //        Serial.println(y);
        //        Serial.println(x);
        //action_done = true;
      }


      ///KEYSTROKE MENU
      if (y >= 44 && y <= 95 && x >= 200 && x <= 300 && !action_done  ) {
        // Serial.println("Menu activated");
        PageNumber = 0;
        action_done = true;
        numpadEnable = false;
        notificationEnable = false;
      }

      //MainMenuPushButtons/////////////////////////////////////////////////////////////////
      if (PageNumber == 0) {
        //Dashboard
        if (!action_done && y >= 95 && y <= 210 && x >= 0 && x <= 120 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 1;
          action_done = true;
        }

        //VoltageGraph
        if (!action_done && y >= 95 && y <= 210 && x >= 120 && x <= 240 ) {
          PageNumber = 2;
          action_done = true;
        }

        //Alarms //Current Alarms
        if (!action_done && y >= 95 && y <= 210 && x >= 240 && x <= 360 ) {
          PageNumber = 6;
          action_done = true;
        }

        //Calibration
        if (!action_done && y >= 95 && y <= 210 && x >= 360 && x <= 480 ) {
          PageNumber = 12;
          action_done = true;
          Serial.println("Calibration Page");
        }


        //Energies
        if (!action_done && y >= 210 && y <= 319 && x >= 240 && x <= 360 ) {
          PageNumber = 10;

          action_done = true;
        }




        //Cell Monitor
        if (!action_done && y >= 210 && y <= 319 && x >= 0 && x <= 120 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 5;
          action_done = true;
        }
      }


      if (PageNumber == 10) {   //Energybuttons
        if (!action_done && y >= 260 && y <= 319 && x >= 80 && x <= 400 && !confirmation ) {

          confirmation = true;

        }
        //CONFIRMATION PROCESS

        if (!action_done && y >= 190 && y <= 250 && x >= 100 && x <= 200 && confirmation) {
          Serial.println("YES");
          Serial.println("energies are erased");
          EEPROMbusy = true;
          action_done = true;
          preferences.begin("my-app", false);
          preferences.putFloat("ChargeEnergy", ChargeEnergy);
          preferences.putFloat("DischargeEnergy", DischargeEnergy);
          preferences.end();
          ChargeEnergy = 0;
          DischargeEnergy = 0;
          EEPROMbusy = false;
          confirmation = false;
        }
        if (!action_done && y >= 190 && y <= 250 && x >= 280 && x <= 380 && confirmation) {
          Serial.println("NO");
          confirmation = false;
        }
      }


      //AlarmList
      if (!action_done && y >= 60 && y <= 100 && x >= 440 && x <= 480 ) {
        action_done = true;
        PageNumber = 11;
      }


      //Mosfets
      if (!action_done && y >= 210 && y <= 319 && x >= 120 && x <= 240 && PageNumber == 0 ) {
        action_done = true;
        PageNumber = 13;
      }

      //Info
      if (!action_done && y >= 210 && y <= 319 && x >= 360 && x <= 480 && PageNumber == 0 ) {
        action_done = true;
        PageNumber = 14;
        ReadAllParameters = true;
      }





      //GraphPushButtons/////////////////////////////////////////////////////////////////
      if (PageNumber == 2 || PageNumber == 3 || PageNumber == 4) {

        //VoltageGraph
        if (!action_done && y >= 275 && y <= 320 && x >= 0 && x <= 164 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 2;
          action_done = true;
        }

        //CurrentGraph
        if (!action_done && y >= 275 && y <= 320 && x >= 164 && x <= 328 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 3;
          action_done = true;
        }

        //TempGraph
        if (!action_done && y >= 275 && y <= 320 && x >= 328 && x <= 480 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 4;
          action_done = true;
        }
      }

      //AlarmsPushButtons/////////////////////////////////////////////////////////////////
      if (PageNumber == 6 || PageNumber == 7 || PageNumber == 8 || PageNumber == 9) {
        if ( numpadEnable == false) {

          //Current
          if (!action_done && (y >= 290 && y <= 319) && (x >= 0 && x <= 115) ) {
            PageNumber = 6;
            action_done = true;


          }

          //Cells
          if (!action_done && y >= 290 && y <= 320 && x >= 122 && x <= 240 ) {
            PageNumber = 7;
            action_done = true;
          }

          //Rates
          if (!action_done && y >= 290 && y <= 320 && x >= 242 && x <= 360 ) {
            PageNumber = 8;
            action_done = true;
          }

          //Balance
          if (!action_done && y >= 290 && y <= 320 && x >= 360 && x <= 480 ) {
            PageNumber = 9;
            action_done = true;
          }
        }
      }

      //Current alarm  params
      if (PageNumber == 6) {
        if (!action_done && y >= 105 && y <= 165 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HCD
          Serial.println("HCD");
          action_done = true;
          numpadEnable = true;
          HCDEnable = true;
          numpadValue = "";
        }
        if (!action_done && y >= 195 && y <= 255 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HCC
          Serial.println("HCC");
          action_done = true;
          numpadEnable = true;
          HCCEnable = true;
          numpadValue = "";
        }

        if (!action_done && y >= 105 && y <= 255 && x >= 389 && x <= 460 && numpadEnable == false) { //open numpad for HCCH
          Serial.println("SETTED HCD & HCC");
          action_done = true;
          HCC_HCD = true;

        }

      }



      //SOC Calibration
      if (PageNumber == 12) {
        if (!action_done && y >= 105 && y <= 165 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HCD
          Serial.println("SOC Numpad opened");
          action_done = true;
          numpadEnable = true;
          SOCEnable = true;
          numpadValue = "";
        }

        if (!action_done && y >= 105 && y <= 255 && x >= 389 && x <= 460 && numpadEnable == false) { //open numpad for SOC
          Serial.println("SETTED SOC");
          action_done = true;
          SOC_CAL = true;
        }
      }



      //MOSFET OPERATION
      if (PageNumber == 13) {
        if (!action_done && y >= 150 && y <= 198 && x >= 280 && x <= 325) { //Charge write
          Serial.println("charge toggled");
          WriteCharge = !WriteCharge;
          action_done = true;
        }

        if (!action_done && y >= 205 && y <= 248 && x >= 280 && x <= 325) { //Discharge write
          Serial.println("discharge toggled");
          WriteDischarge = !WriteDischarge;
          action_done = true;
        }

        if (!action_done && y >= 260 && y <= 303 && x >= 280 && x <= 325) { //Balance write
          Serial.println("balance toggled");
          WriteBalance = !WriteBalance;
          action_done = true;
        }
        /////////////////////////

        if (!action_done && y >= 150 && y <= 190 && x >= 360 && x <= 440) { //Charge Set
          Serial.println("charge setted ");
          CHARGE_SET = true;

          action_done = true;
        }

        if (!action_done && y >= 200 && y <= 240 && x >= 360 && x <= 440) { //Charge Set
          Serial.println("discharge setted ");
          DISCHARGE_SET = true;
          action_done = true;
        }

        if (!action_done && y >= 250 && y <= 290 && x >= 360 && x <= 440) { //Charge Set
          Serial.println("balance setted ");
          action_done = true;
          BALANCE_SET = true;
        }

      }




      //Cell alarm  params
      if (PageNumber == 7) {
        if (!action_done && y >= 105 && y <= 165 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HVC
          Serial.println("HVC");
          action_done = true;
          numpadEnable = true;
          HVCEnable = true;
          numpadValue = "";
        }
        if (!action_done && y >= 195 && y <= 255 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for LVC
          Serial.println("LVC");
          action_done = true;
          numpadEnable = true;
          LVCEnable = true;
          numpadValue = "";
        }

        if (!action_done && y >= 105 && y <= 255 && x >= 389 && x <= 460 && numpadEnable == false) { //open numpad for HCCH
          Serial.println("SETTED HVC_LVC");
          action_done = true;
          HVC_LVC = true;
        }
      }

      //Rates alarm  params
      if (PageNumber == 8) {
        if (!action_done && y >= 105 && y <= 165 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HVC
          Serial.println("RC");
          action_done = true;
          numpadEnable = true;
          RCEnable = true;
          numpadValue = "";
        }
        if (!action_done && y >= 195 && y <= 255 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for LVC
          Serial.println("RV");
          action_done = true;
          numpadEnable = true;
          RVEnable = true;
          numpadValue = "";
        }


        if (!action_done && y >= 105 && y <= 255 && x >= 389 && x <= 460 && numpadEnable == false) { //open numpad for RC RV
          Serial.println("SETTED RC_RV");
          action_done = true;
          RC_RV = true;
        }

      }

      //Balance alarm  params
      if (PageNumber == 9) {
        if (!action_done && y >= 105 && y <= 165 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HVC
          Serial.println("BV");
          action_done = true;
          numpadEnable = true;
          BVEnable = true;
          numpadValue = "";
        }
        if (!action_done && y >= 195 && y <= 255 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for LVC
          Serial.println("BD");
          action_done = true;
          numpadEnable = true;
          BDEnable = true;
          numpadValue = "";
        }

        if (!action_done && y >= 105 && y <= 255 && x >= 389 && x <= 460 && numpadEnable == false) { //open numpad for BV BD
          Serial.println("SETTED BV_BD");
          action_done = true;
          BV_BD = true;
        }
      }





      //Operation NUMPAD
      if (numpadEnable == true ) {

        if (!action_done && y >= 146 && y <= 186 && x >= 110 && x <= 170 && numpadValue.length() < 5) { //7 BUTTON
          Serial.println("7");
          action_done = true;

          numpadValue = numpadValue + "7";
        }

        if (!action_done && y >= 146 && y <= 186 && x >= 172 && x <= 232 && numpadValue.length() < 5) { //8 BUTTON
          Serial.println("8");
          action_done = true;
          numpadValue = numpadValue + "8";
        }

        if (!action_done && y >= 146 && y <= 186 && x >= 234 && x <= 294 && numpadValue.length() < 5) { //9 BUTTON
          Serial.println("9");
          action_done = true;
          numpadValue = numpadValue + "9";
        }

        if (!action_done && y >= 146 && y <= 186 && x >= 296 && x <= 371 ) { //DEL
          Serial.println("DEL");
          action_done = true;
          int numpadLength = numpadValue.length();
          numpadValue.remove(numpadLength - 1, 1);
        }
        ////////////////////////////////////////////////////

        if (!action_done && y >= 189 && y <= 229 && x >= 110 && x <= 170 && numpadValue.length() < 5 ) { //4 BUTTON
          Serial.println("4");
          action_done = true;
          numpadValue = numpadValue + "4";
        }

        if (!action_done && y >= 189 && y <= 229 && x >= 172 && x <= 232 && numpadValue.length() < 5) { //5 BUTTON
          Serial.println("5");
          action_done = true;
          numpadValue = numpadValue + "5";
        }

        if (!action_done && y >= 189 && y <= 229 && x >= 234 && x <= 294 && numpadValue.length() < 5) { //6 BUTTON
          Serial.println("6");
          action_done = true;
          numpadValue = numpadValue + "6";
        }

        if (!action_done && y >= 189 && y <= 229 && x >= 296 && x <= 371 ) { //CLR
          Serial.println("DEL");
          action_done = true;
          numpadValue = "";
        }
        /////////////////////////////////////////////////////
        if (!action_done && y >= 232 && y <= 272 && x >= 110 && x <= 170 && numpadValue.length() < 5) { //1 BUTTON
          Serial.println("1");
          action_done = true;
          numpadValue = numpadValue + "1";
        }

        if (!action_done && y >= 232 && y <= 272 && x >= 172 && x <= 232 && numpadValue.length() < 5 ) { //2 BUTTON
          Serial.println("2");
          action_done = true;
          numpadValue = numpadValue + "2";
        }

        if (!action_done && y >= 232 && y <= 272 && x >= 234 && x <= 294 && numpadValue.length() < 5) { //3 BUTTON
          Serial.println("3");
          action_done = true;
          numpadValue = numpadValue + "3";
        }

        if (!action_done && y >= 232 && y <= 272 && x >= 296 && x <= 371 ) { //BACK BUTTON
          Serial.println("back");
          numpadEnable = false;
          action_done = true;
        }
        /////////////////////////////////////////////////////////////////////////

        if (!action_done && y >= 275 && y <= 315 && x >= 110 && x <= 230 && numpadValue.length() < 5) { //0 BUTTON
          Serial.println("0");
          action_done = true;
          numpadValue = numpadValue + "0";
        }

        if (!action_done && y >= 275 && y <= 315 && x >= 234 && x <= 294 && numpadValue.length() < 5 ) { //. BUTTON
          Serial.println(".");
          action_done = true;
          numpadValue = numpadValue + ".";
        }

        if (!action_done && y >= 275 && y <= 315 && x >= 296 && x <= 371 ) { //OK BUTTON
          Serial.println("OK");
          numpadEnable = false;
          action_done = true;


          if (SOCEnable) {
            if (numpadValue.toFloat() <= 1000 && numpadValue.toFloat() >= 0) {
              SOC_CAL_VAL = numpadValue.toFloat();
              Serial.println("SOC Value is setted:" + String(SOC_CAL_VAL));
              SOCEnable = false;
            }
            else {
              Serial.println("Maximum value 100 is exceeded!");
            }
          }



          if (HCDEnable) {
            if (numpadValue.toFloat() <= 400) {
              HCD = numpadValue.toFloat();
              //Serial.println("HCD Value is setted:" + String(HCD));
              HCDEnable = false;
            }
            else {
              Serial.println("Maximum value 400A is exceeded!");
            }
          }


          if (HCCEnable) {
            if (numpadValue.toFloat() <= 200) {
              HCC = numpadValue.toFloat();
              HCCEnable = false;
            }
            else {
              Serial.println("Maximum value 200A is exceeded!");
            }
          }

          if (HVCEnable) {
            if (numpadValue.toFloat() <= 4 && numpadValue.toFloat() >= 2.2) {
              HVC = numpadValue.toFloat();
              HVCEnable = false;


            }
            else {
              Serial.println("Maximum value 4v is exceeded!");
            }
          }

          if (LVCEnable) {
            if (numpadValue.toFloat() >= 1.8 && numpadValue.toFloat() <= 4 && LVC < HVC) {
              LVC = numpadValue.toFloat();
              LVCEnable = false;


            }
            else {
              Serial.println("Minimum value 2v is exceeded! or High voltage is lower than low voltage" );
            }
          }

          if (RCEnable) {
            RC = numpadValue.toFloat();
            RCEnable = false;

          }

          if (RVEnable) {
            RV = numpadValue.toFloat();
            RVEnable = false;

          }

          if (BVEnable) {
            if (numpadValue.toFloat() <= 4) {
              BV = numpadValue.toFloat();
              BVEnable = false;

            }
            else {
              Serial.println("Maximum value 4V is exceeded!");
            }
          }

          if (BDEnable) {
            if (numpadValue.toFloat() <= 100) {
              BD = numpadValue.toFloat();
              BDEnable = false;

            }
            else {
              Serial.println("Maximum value 100mv is exceeded!");
            }
          }
        }
      }

      //MainMenu
      if (y >= 44 && y <= 95 && x >= 200 && x <= 300 && !action_done  ) {
        PageNumber = 0;
        action_done = true;
        numpadEnable = false;
      }

      if (touch_iteration < 100)
        touch_iteration++;
    }
    else {
      touch_iteration = 0;
      action_done = false;
      for (int i = 0; i < 100; i++) {
        TouchArray[i] = 0;
      }
    }
  }
}

// #########################################################################
//FUNCTIONS
// #########################################################################

void drawMainMenu() {
  MainMenu.createSprite(480, 225);
  MainMenu.setTextColor(TFT_WHITE);

  DashboardIcon.createSprite(48, 48);
  DashboardIcon.setSwapBytes(true);
  DashboardIcon.pushImage(0, 0, 48, 48, dashboard);
  DashboardIcon.pushToSprite(&MainMenu, 40, 20, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Dashb.", 70, 105);
  MainMenu.setTextDatum(TL_DATUM);

  VoltageGraphIcon.createSprite(48, 48);
  VoltageGraphIcon.setSwapBytes(true);
  VoltageGraphIcon.pushImage(0, 0, 48, 48, graph);
  VoltageGraphIcon.pushToSprite(&MainMenu, 155, 15, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Graphs", 185, 105);
  MainMenu.setTextDatum(TL_DATUM);

  CurrentGraphIcon.createSprite(48, 48);
  CurrentGraphIcon.setSwapBytes(true);
  CurrentGraphIcon.pushImage(0, 0, 48, 48, alarms);
  CurrentGraphIcon.pushToSprite(&MainMenu, 275, 15, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Alarms", 305, 105);
  MainMenu.setTextDatum(TL_DATUM);

  TempGraphIcon.createSprite(48, 48);
  TempGraphIcon.setSwapBytes(true);
  TempGraphIcon.pushImage(0, 0, 48, 48, calibration);
  TempGraphIcon.pushToSprite(&MainMenu, 390, 15, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Calib.", 420, 105);
  MainMenu.setTextDatum(TL_DATUM);

  CellIcon.createSprite(48, 48);
  CellIcon.setSwapBytes(true);
  CellIcon.pushImage(0, 0, 48, 48, cell);
  CellIcon.pushToSprite(&MainMenu, 40, 120, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Cells", 70, 205);
  MainMenu.setTextDatum(TL_DATUM);

  BalanceIcon.createSprite(48, 48);
  BalanceIcon.setSwapBytes(true);
  BalanceIcon.pushImage(0, 0, 48, 48, balance);
  BalanceIcon.pushToSprite(&MainMenu, 155, 120, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Mosfet", 185, 205);
  MainMenu.setTextDatum(TL_DATUM);

  EnergyIcon.createSprite(48, 48);
  EnergyIcon.setSwapBytes(true);
  EnergyIcon.pushImage(0, 0, 48, 48, energy);
  EnergyIcon.pushToSprite(&MainMenu, 275, 120, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Energy", 305, 205);
  MainMenu.setTextDatum(TL_DATUM);

  SettingsIcon.createSprite(48, 48);
  SettingsIcon.setSwapBytes(true);
  SettingsIcon.pushImage(0, 0, 48, 48, settings);
  SettingsIcon.pushToSprite(&MainMenu, 390, 120, TFT_BLACK);
  MainMenu.setTextDatum(BC_DATUM);
  MainMenu.setFreeFont(&Orbitron_Light_32);
  MainMenu.drawString("Info", 420, 205);
  MainMenu.setTextDatum(TL_DATUM);


  MainMenu.pushSprite(0, 95);
  MainMenu.deleteSprite();

}

//x25 y10

void drawCellMainUp(int startXcoor, int startYcoor, float Voltage, int CellNumber, double Alarm) {
  CellFrame.setTextDatum(MC_DATUM);

  float envelope = random(0, 14) * 0.1;
  float anode = random(27, 37) * 0.1;
  float cathode = random(27, 37) * 0.1;

  CellFrame.fillRoundRect(startXcoor , startYcoor , 10, 120, 2, TFT_DARKGREY);
  // CellFrame.fillRoundRect(startXcoor , startYcoor , 10, 50, 2, TOLGA_GREEN);



  CellFrame.fillRoundRect(startXcoor , 40 , 10, ((anode - 2.7)) * 50, 2, TOLGA_GREEN);
  CellFrame.fillRoundRect(startXcoor , 40 + ((anode - 2.7)) * 50 , 10, 2, 2, TFT_WHITE);


  CellFrame.fillRoundRect(startXcoor , 160 - (cathode - 2.7) * 50 , 10, ((cathode - 2.7)) * 50, 2, TOLGA_YELLOW);
  CellFrame.fillRoundRect(startXcoor , 160 - (cathode - 2.7) * 50 , 10, 2, 2, TFT_WHITE);


  CellFrame.setTextFont(GLCD);
  CellFrame.setTextColor(TOLGA_GREEN);
  CellFrame.drawString("Anode", startXcoor - 20, startYcoor + 10);

  CellFrame.setTextColor(TFT_WHITE);
  CellFrame.drawString(String(anode), startXcoor - 20, startYcoor + 23);

  CellFrame.setTextColor(TFT_DARKGREY);
  CellFrame.drawString("Env.", startXcoor - 20, startYcoor + 55);

  CellFrame.setTextColor(TFT_WHITE);
  CellFrame.drawString(String(envelope), startXcoor - 20, startYcoor + 68);

  CellFrame.setTextColor(TOLGA_YELLOW);
  CellFrame.drawString("Cath", startXcoor - 20, startYcoor + 95);

  CellFrame.setTextColor(TFT_WHITE);
  CellFrame.drawString(String(cathode), startXcoor - 20, startYcoor + 108);

  CellFrame.setTextColor(TOLGA_BLUE_2);
  CellFrame.drawString("Cell#" + String(CellNumber), startXcoor , startYcoor + 140);

  CellFrame.setTextColor(TFT_WHITE);
  CellFrame.drawString(String(anode + cathode), startXcoor , startYcoor + 153);

}








void drawCell() {

  CellFrame.createSprite(480, 225);

  for (int t = 0; t < 8; t++) {
    float cellvolt = BMS_equ.cell_voltage_equ[t] * 0.001;
    drawCellMainUp(50 + 57 * t, 40 , cellvolt, (t) + 1, cellvolt == 2.50);

  }


  CellFrame.pushSprite(0, 95);
  CellFrame.deleteSprite();
}


void drawBanner(String Time, String Date, String Page, bool wifistatus, int notification_no) {

  Banner.createSprite(480, 95);
  Banner.setTextColor(TFT_WHITE);
  Banner.setTextDatum(TL_DATUM);
  BannerFrame.createSprite(480, 95);
  BannerFrame.setSwapBytes(true);
  BannerFrame.pushImage(0, 0, 480, 95, Header_2);
  BannerFrame.pushToSprite(&Banner, 0, 0, TFT_BLACK);

  // Banner.setFreeFont(&Orbitron_Light_24);
  timeIcon.createSprite(16, 16);
  timeIcon.setSwapBytes(true);
  timeIcon.pushImage(0, 0, 16, 16, clock1);
  timeIcon.pushToSprite(&Banner, 20, 66, TFT_BLACK);
  Banner.drawString(Time, 45, 68, 2);

  Banner.setTextDatum(BC_DATUM);
  Banner.setFreeFont(&Orbitron_Light_32);
  Banner.drawString(Page, 240, 90);
  Banner.setTextDatum(TL_DATUM);

  //FirmwareVersion
  Banner.setTextFont(GLCD);
  Banner.drawString("Ver:" + FirmwareVer, 430, 5);
  Banner.drawString("R:" + String(restartCounter), 430, 18);
  Banner.setFreeFont(&Orbitron_Light_32);

  //Banner.setFreeFont(&Orbitron_Light_24);
  Banner.drawString(String(notification_no), 460, 68, 2);
  notificationIcon.createSprite(16, 16);
  notificationIcon.setSwapBytes(true);
  notificationIcon.pushImage(0, 0, 16, 16, notification);
  notificationIcon.pushToSprite(&Banner, 440, 66, TFT_BLACK);

  wifiIcon.createSprite(16, 16);
  wifiIcon.setSwapBytes(true);
  if (wifistatus)
    wifiIcon.pushImage(0, 0, 16, 16, wifi);
  else
    wifiIcon.pushImage(0, 0, 16, 16, nowifi);
  wifiIcon.pushToSprite(&Banner, 410, 66, TFT_BLACK);
  Banner.pushSprite(0, 0);
  Banner.deleteSprite();
}

void drawDashboard(int SOC, float MaxVolt, float MinVolt, float MaxDiff, float TerminalVolt, float TerminalCurr, float TerminalTemp) {
  Dashboard.createSprite(480, 225);
  Dashboard.setFreeFont(&Orbitron_Light_24);
  Dashboard.setTextDatum(TL_DATUM);
  ringMeter(SOC, 0, 100, 160, 40, 80, "SOC", TOLGA_BLUE, Dashboard); // Draw analogue meter


  //Dashboard Parameters
  Dashboard.setFreeFont(&Orbitron_Light_24);
  Dashboard.setTextDatum(TL_DATUM);

  //Maximum Voltage
  Dashboard.setTextColor(TFT_WHITE, TFT_BLACK);
  maxIcon_Icon.createSprite(48, 48);
  maxIcon_Icon.setSwapBytes(true);
  maxIcon_Icon.pushImage(0, 0, 48, 48, maxIcon);
  maxIcon_Icon.pushToSprite(&Dashboard, 15, 15, TFT_BLACK);
  Dashboard.drawString("Maximum ", 70, 15, 2);
  Dashboard.drawString("Voltage ", 72, 30, 2);
  Dashboard.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  Dashboard.drawString(String(MaxVolt, 3) + "V", 72, 50, 2);

  //Minimum Voltage
  Dashboard.setTextColor(TFT_WHITE, TFT_BLACK);
  maxIcon_Icon.createSprite(48, 48);
  maxIcon_Icon.setSwapBytes(true);
  maxIcon_Icon.pushImage(0, 0, 48, 48, minIcon);
  maxIcon_Icon.pushToSprite(&Dashboard, 15, 90, TFT_BLACK);
  Dashboard.drawString("Minimum ", 70, 90, 2);
  Dashboard.drawString("Voltage ", 72, 105, 2);
  Dashboard.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  Dashboard.drawString(String(MinVolt, 3) + "V", 72, 125, 2);

  //Diff Voltage
  Dashboard.setTextColor(TFT_WHITE, TFT_BLACK);
  maxIcon_Icon.createSprite(48, 48);
  maxIcon_Icon.setSwapBytes(true);
  maxIcon_Icon.pushImage(0, 0, 48, 48, delta);
  maxIcon_Icon.pushToSprite(&Dashboard, 15, 165, TFT_BLACK);
  Dashboard.drawString("Diffr. ", 70, 165, 2);
  Dashboard.drawString("Voltage ", 72, 180, 2);
  Dashboard.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  Dashboard.drawString(String(MaxDiff, 3) + "V", 72, 200, 2);

  //Total Voltage
  Dashboard.setTextColor(TFT_WHITE, TFT_BLACK);
  maxIcon_Icon.createSprite(48, 48);
  maxIcon_Icon.setSwapBytes(true);
  maxIcon_Icon.pushImage(0, 0, 48, 48, voltage);
  maxIcon_Icon.pushToSprite(&Dashboard, 340, 20, TFT_BLACK);
  Dashboard.drawString("Terminal ", 395, 20, 2);
  Dashboard.drawString("Voltage ", 397, 35, 2);
  Dashboard.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  Dashboard.drawString(String(TerminalVolt) + "V", 397, 55, 2);

  //Terminal Current
  Dashboard.setTextColor(TFT_WHITE, TFT_BLACK);
  maxIcon_Icon.createSprite(48, 48);
  maxIcon_Icon.setSwapBytes(true);
  maxIcon_Icon.pushImage(0, 0, 48, 48, current);
  maxIcon_Icon.pushToSprite(&Dashboard, 340, 90, TFT_BLACK);
  Dashboard.drawString("Terminal ", 395, 90, 2);
  Dashboard.drawString("Current ", 397, 105, 2);
  Dashboard.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  Dashboard.drawString(String(TerminalCurr) + "A", 397, 125, 2);

  //Terminal Temp
  Dashboard.setTextColor(TFT_WHITE, TFT_BLACK);
  maxIcon_Icon.createSprite(48, 48);
  maxIcon_Icon.setSwapBytes(true);
  maxIcon_Icon.pushImage(0, 0, 48, 48, temp);
  maxIcon_Icon.pushToSprite(&Dashboard, 340, 165, TFT_BLACK);
  Dashboard.drawString("Cell ", 395, 165, 2);
  Dashboard.drawString("Temp. ", 397, 180, 2);
  Dashboard.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  Dashboard.drawString(String(TerminalTemp) + "C", 397, 200, 2);


  Dashboard.pushSprite(0, 95);
  Dashboard.deleteSprite();


}


int ringMeter(int value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme , TFT_eSPI & tft)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  x += r; y += r;   // Calculate coords of centre of ring
  int w = r / 3;    // Width of outer ring is 1/4 of radius
  int w2 = r / 5;   // Width of outer ring is 1/4 of radius
  int angle = 150;  // Half the sweep angle of meter (300 degrees)
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring
  // Variable to save "value" text colour from scheme and set default
  int colour = TFT_BLUE;
  // Draw colour blocks every inc degrees
  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc) {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      switch (scheme) {
        case 0: colour = TOLGA_RED; break; // Fixed colour
        case 1: colour = TOLGA_GREEN; break; // Fixed colour
        case 2: colour = TOLGA_BLUE; break; // Fixed colour
        default: colour = TOLGA_BLUE; break; // Fixed colour
      }


      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_DARKGREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_DARKGREY);
    }
  }

  // Convert value to a string
  char buf[10];
  byte len = 3; if (value > 999) len = 5;
  dtostrf(value, len, 0, buf);
  buf[len] = ' ';
  buf[len + 1] = 0; // Add blanking space and terminator, helps to centre text too!

  tft.setFreeFont(&Orbitron_Light_32);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);

  if (r > 84) {
    tft.setTextPadding(55 * 3); // Allow for 3 digits each 55 pixels wide
    tft.setTextColor(TOLGA_YELLOW);
    tft.drawString(String(buf) + "%", x, y); // Value in middle
  }
  else {
    tft.setTextColor(TOLGA_YELLOW);
    tft.setTextPadding(3 * 14); // Allow for 3 digits each 14 pixels wide
    tft.drawString(String(buf) + "%", x, y); // Value in middle
  }

  tft.setTextColor(TFT_WHITE);
  tft.drawString(units, x, 185); // Units display
  return x + r;
}

bool printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return false;
  }

  char timeHour[3];
  strftime(timeHour, 3, "%H", &timeinfo);
  strftime(timeMin, 3, "%M", &timeinfo);
  strftime(timeSec, 3, "%S", &timeinfo);
  strftime(timeDay, 3, "%d", &timeinfo);
  strftime(timeMonth, 3, "%m", &timeinfo);
  strftime(timeYear, 10, "%Y", &timeinfo);

  timeHourS = String(timeHour);
  timeMinS = String(timeMin);
  timeSecS = String(timeSec);

  timeDayS = String(timeDay);
  timeMonthS = String(timeMonth);
  timeYearS = String(timeYear);

  TimeString = timeHourS + ":" + timeMinS;
  DateString = timeDayS + "/" + timeMonthS + "/" + timeYearS;

  DateDir_Voltage = "/Voltage/" + timeDayS + "/" + timeHourS + "/" + timeMinS + "/data.txt";
  DateDir_Current = "/Current/" + timeDayS + "/" + timeHourS + "/" + timeMinS + "/data.txt";
  DateDir_Temp = "/Temp/" + timeDayS + "/" + timeHourS + "/" + timeMinS + "/data.txt";
  return true;

}

void getBatteryParameters() {

  if (!ReadAllParameters) {
    send_read_BMS_equ(0x91);
    BMS_recieve_equ(0x91);
    delay(1);

    send_read_BMS(0x90);
    BMS_recieve(0x90);
    delay(1);


    send_read_BMS(0x92);
    BMS_recieve(0x92);
    delay(1);
    //
    send_read_BMS(0x93);
    BMS_recieve(0x93);
    delay(1);
    //
    send_read_BMS(0x94);
    BMS_recieve(0x94);
    delay(1);


    send_read_BMS(0x97);
    BMS_recieve(0x97);
    delay(1);

    send_read_BMS(0x98);
    BMS_recieve(0x98);
    delay(1);


    send_read_BMS(0x95);
    BMS_recieve(0x95);
    delay(1);


    for (int i = 0; i < 16; i++) {
      dailyCells[i] = BMS.cell_voltage[i];
    }


    send_read_BMS_equ(0x95);
    BMS_recieve_equ(0x95);
    delay(1);

    send_read_BMS_equ(0xD8);
    BMS_recieve_equ(0xD8);
    delay(10);

    send_read_BMS(0xD8);
    BMS_recieve(0xD8);
    delay(500);


  }

  else {
    loading = true;

    ///////////////////////////////////////////////////////////////////trial////////
    for (int i = 0; i < 20; i++) {
      balance_turn_off();
    }



    ///////////////////////////////////////////////////////////////////

    //rated capacity
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x50);
      BMS_recieve(0x50);
      delay(50);
    }
    loadingPercentage = 10;

    //charge discharge high low
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x5B);
      BMS_recieve(0x5B);
      delay(50);
    }
    loadingPercentage = 20;
    //sum highlow
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x5A);
      BMS_recieve(0x5A);
      delay(50);
    }
    loadingPercentage = 30;

    //CHARGE DISCHARGE TEMP
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x5C);
      BMS_recieve(0x5C);
      delay(50);
    }
    loadingPercentage = 40;

    //SOC
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x5D);
      BMS_recieve(0x5D);
      delay(50);
    }
    loadingPercentage = 60;

    //CUMULATIVE
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x52);
      BMS_recieve(0x52);
      delay(50);
    }
    loadingPercentage = 70;

    //vOLT DIFF
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x5E);
      BMS_recieve(0x5E);
      delay(50);
    }
    loadingPercentage = 80;

    //BALANCE START
    for (int i = 1; i < 21; i++) {
      send_read_BMS(0x5F);
      BMS_recieve(0x5F);
      delay(50);
    }

    loadingPercentage = 90;
    //cell volt high low
    for (int i = 0; i < 20; i++) {
      send_read_BMS(0x59);
      BMS_recieve(0x59);
      delay(50);
    }
    loadingPercentage = 100;

    delay(10);
    Serial.print("Charge:");
    Serial.println(BMS.charge_curr2);

    delay(10);
    Serial.print("SUM");
    Serial.println(BMS.sumv_high2);

    delay(10);
    Serial.print("CELL");
    Serial.println(BMS.cell_volthigh2);


    delay(100);

    loading = false;

    ReadAllParameters = false;
    Serial.println("All parameters are read");
  }
}


void DrawBarChartV(TFT_eSPI & d, double x , double y , double w, double h , double loval , double hival , double inc , double curval ,  int dig , int dec, unsigned int barcolor, unsigned int voidcolor, unsigned int bordercolor, unsigned int textcolor, unsigned int backcolor, String label, boolean & redraw)
{

  double stepval, range;
  double my, level;
  double i, data;
  // draw the border, scale, and label once
  // avoid doing this on every update to minimize flicker
  if (redraw == true) {
    redraw = false;

    d.setTextColor(TFT_WHITE);
    d.drawString(label, x, y - h - 30, 2);




    // step val basically scales the hival and low val to the height
    // deducting a small value to eliminate round off errors
    // this val may need to be adjusted
    stepval = ( inc) * (double (h) / (double (hival - loval))) - .001;
    for (i = 0; i <= h; i += stepval) {
      my =  y - h + i;
      d.drawFastHLine(45, my,  5, textcolor);
      // draw lables
      // d.setTextSize(1);
      d.setTextColor(textcolor, backcolor);
      d.setCursor(10, my - 3 );
      data = hival - ( i * (inc / stepval));
      d.println(Format(data, dig, dec));
    }
  }

  //Serial.println(label);
  // compute level of bar graph that is scaled to the  height and the hi and low vals
  // this is needed to accompdate for +/- range
  level = (h * (((curval - loval) / (hival - loval))));
  // draw the bar graph
  // write a upper and lower bar to minimize flicker cause by blanking out bar and redraw on update
  d.fillRect(x, y - h, w, h - level,  voidcolor);
  d.fillRect(x, y - level, w,  level, barcolor);
  // write the current value
  d.setTextColor(textcolor, backcolor);
  //  d.setTextSize(2);
  d.setCursor(x , y + 10);
  // d.println(Format(curval, dig, dec));

}

String Format(double val, int dec, int dig ) {
  int addpad = 0;
  char sbuf[20];
  String condata = (dtostrf(val, dec, dig, sbuf));


  int slen = condata.length();
  for ( addpad = 1; addpad <= dec + dig - slen; addpad++) {
    condata = " " + condata;
  }
  return (condata);
}


void drawBarGraph(TFT_eSprite & tft, double Data[], int graphColor, String Title) {
  bool graph_1 = true;
  tft.createSprite(480, 225);
  if (graph_1) {
    tft.fillSprite(TFT_BACKGROUND4);
  }
  for (int i = 0; i < 16; i++) {
    DrawBarChartV(tft, i * 16 + 60,  250, 10, 210, 0, 100 , 10, Data[i] * 0.001 , 4 , 0, graphColor, TFT_BACKGROUND2, graphColor, WHITE, TFT_BACKGROUND4, Title, graph_1);
  }



  tft.pushSprite(0, 95);
}
















void Graph(TFT_eSPI & tft, double x, double y, byte dp,
           double gx, double gy, double w, double h,
           double xlo, double xhi, double xinc,
           double ylo, double yhi, double yinc,
           char *title, char *xlabel, char *ylabel,
           bool & redraw, unsigned int color) {

  double ydiv, xdiv;
  double i;
  double temp;
  int rot, newrot;


  unsigned int gcolor = TFT_DARKGREY;
  unsigned int acolor = TOLGA_BLUE;
  unsigned int pcolor = TOLGA_YELLOW;
  unsigned int tcolor = WHITE;
  unsigned int bcolor = TFT_BACKGROUND4;

  if (redraw == true) {

    redraw = true;
    tft.setTextDatum(MR_DATUM);

    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        tft.drawLine(gx, temp, gx + w, temp, acolor);
        tft.setTextColor(acolor, bcolor);
        tft.drawString(xlabel, (int)(gx + w) , (int)temp, 2);
      }
      else {
        tft.drawLine(gx, temp, gx + w, temp, gcolor);
      }
      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat(i, dp, gx - 4, temp, 1);
    }

    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        tft.drawLine(temp, gy, temp, gy - h, acolor);
        tft.setTextColor(acolor, bcolor);
        tft.setTextDatum(BC_DATUM);
        tft.drawString(ylabel, (int)temp, (int)(gy - h - 8) , 2);
      }
      else {
        tft.drawLine(temp, gy, temp, gy - h, gcolor);
      }
      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      tft.setTextDatum(TC_DATUM);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat(i, dp, temp, gy + 7, 1);
    }

    //now draw the graph labels
    tft.setTextColor(tcolor, TFT_BACKGROUND4);
    //tft.drawString(title, (int)(gx + w / 6) , (int)(gy - h - 25), 2);
  }

}

void Trace(TFT_eSPI & tft, double x,  double y,  byte dp,
           double gx, double gy,
           double w, double h,
           double xlo, double xhi, double xinc,
           double ylo, double yhi, double yinc,
           char *title, char *xlabel, char *ylabel,
           bool & update1, unsigned int color, double Data)
{
  double ydiv, xdiv;
  double i;
  double temp;
  int rot, newrot;
  int inew;

  //unsigned int gcolor = DKBLUE;   // gcolor = graph grid color
  unsigned int acolor = RED;        // acolor = main axes and label color
  unsigned int pcolor = color;      // pcolor = color of your plotted data
  unsigned int tcolor = WHITE;      // tcolor = text color
  unsigned int bcolor = BLACK;      // bcolor = background color

  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  if (update1) {
    update1 = false;

    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

    if ((ox < gx) || (ox > gx + w)) {
      update1 = true;
      return;
    }
    if ((oy < gy - h) || (oy > gy)) {
      update1 = true;
      return;
    }


    tft.setTextDatum(MR_DATUM);

    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        tft.setTextColor(acolor, bcolor);
        tft.drawString(xlabel, (int)(gx + w) , (int)temp, 2);
      }
      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat((int)i, dp, gx - 4, temp, 1);
    }

    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        tft.setTextColor(acolor, bcolor);
        tft.setTextDatum(BC_DATUM);
        tft.drawString(ylabel, (int)temp, (int)(gy - h - 8) , 2);
      }

      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      tft.setTextDatum(TC_DATUM);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat((int)i, dp, temp, gy + 7, 1);
    }

    //now draw the graph labels
    tft.setTextColor(tcolor, TFT_BACKGROUND4);
    // tft.drawString(title, (int)(gx + w / 6) , (int)(gy - h - 25), 2);;
  }
  // the coordinates are now drawn, plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized above
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

  if ((x < gx) || (x > gx + w)) {
    update1 = true;
    return;
  }
  if ((y < gy - h) || (y > gy)) {
    update1 = true;
    return;
  }


  tft.drawLine(ox, oy, x, y, pcolor);
  // it's up to you but drawing 2 more lines to give the graph some thickness
  tft.drawLine(ox, oy + 1, x, y + 1, pcolor);
  tft.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;
}

/*

  End of graphing function

*/


void drawOptimizerGraph(TFT_eSprite & tft, double DataArray[], char* Title, double Data ) {

  int maxY = 0;
  int minY = 0;
  int grid = 5;
  int grid_count = 5;

  Serial.println("Graph:" + String(Data));


  tft.createSprite(480, 180);


  for (int i = 0; i < 60; i++) {
    if (i != 59) {
      DataArray[i] = DataArray[i + 1];
    }
    else {
      DataArray[i] = Data;
    }
  }

  int a = sizeof(VoltageArray);
  int maxVal = -99999;
  int minVal = 999999;

  for (int i = 0; i < (sizeof(VoltageArray) / sizeof(VoltageArray[0])); i++) {
    if (DataArray[i] > maxVal) {
      maxVal = DataArray[i];


    }
    if (DataArray[i] < minVal) {
      minVal = DataArray[i];
    }
  }

  grid = ((maxVal + 10) - (minVal - 10)) / grid_count;

  if (grid < 2) {
    grid = 2;
  }

  update1 = true;

  for (int i = 0; i < 60; i++) {
    Trace(tft, i, DataArray[i], 1, 70, 140, 350, 120, 0, 60, 5, minVal - 10, maxVal + 10, grid, Title, "", "", update1, TOLGA_YELLOW, Data);

  }
  tft.pushSprite(0, 95);
  tft.fillSprite(TFT_BLACK);
  Graph(tft, x, y, 1, 70, 140, 350, 120, 0, 60, 5, minVal - 10, maxVal + 10, grid, Title, "", "", display1, TOLGA_YELLOW);
}

void drawLandingPage() {

  //SETUP SCREEN------------------------------
  LandingIconFrame.createSprite(172, 172);
  LandingIconFrame.setTextColor(TFT_WHITE);
  LandingIconFrame.fillSprite(TFT_BLACK);
  LandingIconFrame.setFreeFont(&Orbitron_Light_32);


  LandingFrame.createSprite(200, 150);
  LandingFrame.fillSprite(TFT_BLACK);
  LandingFrame.setFreeFont(&Orbitron_Light_32);
  LandingFrame.setTextColor(TFT_WHITE);

  EncapIcon.createSprite(180, 45);
  EncapIcon.setSwapBytes(true);
  EncapIcon.pushImage(0, 0, 180, 45, Encap);


  tft.setPivot(120, 120);
  LandingIcon.createSprite(172, 172);
  LandingIcon.setSwapBytes(true);
  LandingIcon.pushImage(0, 0, 172, 172, circles);
  LandingIcon.pushToSprite(&LandingIconFrame, 0, 0, TFT_BLACK);
  LandingFrame.setFreeFont(&Orbitron_Light_32);
  LandingFrame.setTextColor(TOLGA_YELLOW);

  for (int i = 0; i < 100; i++) {

    LandingFrame.fillSprite(TFT_BLACK);
    LandingFrame.setTextColor(TFT_WHITE);
    LandingFrame.drawString(" Initialization:", 0, 60);
    LandingFrame.setTextColor(TOLGA_YELLOW);
    LandingFrame.drawString(String(i) + "%", 70, 95);


    if (i == 99) {

    }
    else {
      LandingIconFrame.pushRotated(i * 3);
      EncapIcon.pushToSprite(&LandingFrame, 0, 0, TFT_BLACK);
      LandingFrame.pushSprite(250, 150);
      delay(10);

    }
  }
  LandingFrame.deleteSprite();
}


void drawGraphTab(bool Voltage, bool Current, bool Temp ) {


  TabFrameGraph.createSprite(480, 45);
  TabFrameGraph.setTextDatum(BC_DATUM);
  TabFrameGraph.setTextColor(TFT_WHITE);
  TabFrameGraph.fillSprite(TFT_BLACK);
  TabFrameGraph.setFreeFont(&Orbitron_Light_32);

  TabIcon.createSprite(159, 45);
  TabIcon.setSwapBytes(true);
  TabIcon.pushImage(0, 0, 159, 45, tab);


  TabIcon.pushToSprite(&TabFrameGraph, 0, 0, TFT_BLACK);
  TabIcon.pushToSprite(&TabFrameGraph, 164, 0, TFT_BLACK);
  TabIcon.pushToSprite(&TabFrameGraph, 328, 0, TFT_BLACK);

  if (Voltage) {
    TabFrameGraph.setTextColor(TOLGA_BLUE_2);
    TabFrameGraph.drawString("Voltage", 80, 40);
  }
  else {
    TabFrameGraph.setTextColor(TFT_WHITE);
    TabFrameGraph.drawString("Voltage", 80, 40);
  }

  if (Current) {
    TabFrameGraph.setTextColor(TOLGA_BLUE_2);
    TabFrameGraph.drawString("Current", 240, 40);
  }
  else {
    TabFrameGraph.setTextColor(TFT_WHITE);
    TabFrameGraph.drawString("Current", 240, 40);
  }

  if (Temp) {
    TabFrameGraph.setTextColor(TOLGA_BLUE_2);
    TabFrameGraph.drawString("Temp.", 405, 40);
  }
  else {
    TabFrameGraph.setTextColor(TFT_WHITE);
    TabFrameGraph.drawString("Temp", 405, 40);
  }


  TabFrameGraph.setTextDatum(TL_DATUM);
  TabFrameGraph.pushSprite(0, 275);

  TabFrameGraph.deleteSprite();
  TabIcon.deleteSprite();

}

void drawNumpad(TFT_eSprite & tft) {

  NumpadFrame.createSprite(320, 225);
  NumpadFrame.setTextDatum(TL_DATUM);
  NumpadFrame.setTextColor(TFT_WHITE);
  NumpadFrame.fillSprite(TFT_BLACK);
  NumpadFrame.setFreeFont(&Orbitron_Light_32);

  NumpadFrame.fillRoundRect(30, 7, 262, 40, 5, TFT_WHITE);
  NumpadFrame.drawRoundRect(30, 7, 262, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.setTextColor(TFT_RED);
  NumpadFrame.setTextDatum(TR_DATUM);
  NumpadFrame.drawString(numpadValue, 270, 10);
  NumpadFrame.setTextDatum(TL_DATUM);

  NumpadFrame.setTextDatum(MC_DATUM);
  NumpadFrame.setTextColor(TFT_WHITE);

  NumpadFrame.fillRoundRect(30, 51, 60, 40, 5, TFT_BACKGROUND2); //7
  NumpadFrame.drawRoundRect(30, 51, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("7", 55, 67);

  NumpadFrame.fillRoundRect(92, 51, 60, 40, 5, TFT_BACKGROUND2); //8
  NumpadFrame.drawRoundRect(92, 51, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("8", 117, 67);

  NumpadFrame.fillRoundRect(154, 51, 60, 40, 5, TFT_BACKGROUND2); //9
  NumpadFrame.drawRoundRect(154, 51, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("9", 177, 67);

  NumpadFrame.fillRoundRect(216, 51, 75, 40, 5, TFT_BACKGROUND2); //del
  NumpadFrame.drawRoundRect(216, 51, 75, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("DEL", 256, 67);

  //=========================================================================
  NumpadFrame.fillRoundRect(30, 94, 60, 40, 5, TFT_BACKGROUND2); //4
  NumpadFrame.drawRoundRect(30, 94, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("4", 55, 110);

  NumpadFrame.fillRoundRect(92, 94, 60, 40, 5, TFT_BACKGROUND2); //5
  NumpadFrame.drawRoundRect(92, 94, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("5", 117, 110);

  NumpadFrame.fillRoundRect(154, 94, 60, 40, 5, TFT_BACKGROUND2); //6
  NumpadFrame.drawRoundRect(154, 94, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("6", 179, 110);

  NumpadFrame.fillRoundRect(216, 94, 75, 40, 5, TFT_BACKGROUND2); //
  NumpadFrame.drawRoundRect(216, 94, 75, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("CLR", 254, 110);

  //=========================================================================
  NumpadFrame.fillRoundRect(30, 137, 60, 40, 5, TFT_BACKGROUND2); //1
  NumpadFrame.drawRoundRect(30, 137, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("1", 55, 153);

  NumpadFrame.fillRoundRect(92, 137, 60, 40, 5, TFT_BACKGROUND2); //1
  NumpadFrame.drawRoundRect(92, 137, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("2", 117, 153);

  NumpadFrame.fillRoundRect(154, 137, 60, 40, 5, TFT_BACKGROUND2); //1
  NumpadFrame.drawRoundRect(154, 137, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("3", 179, 153);

  NumpadFrame.fillRoundRect(216, 137, 75, 40, 5, TFT_BACKGROUND2); //
  NumpadFrame.drawRoundRect(216, 137, 75, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("BCK", 254, 153);

  //=========================================================================


  NumpadFrame.fillRoundRect(30, 180, 122, 40, 5, TFT_BACKGROUND2); //1
  NumpadFrame.drawRoundRect(30, 180, 122, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("0", 90, 196);

  NumpadFrame.fillRoundRect(154, 180, 60, 40, 5, TFT_BACKGROUND2); //1
  NumpadFrame.drawRoundRect(154, 180, 60, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString(".", 179, 196);

  NumpadFrame.fillRoundRect(216, 180, 75, 40, 5, TFT_BACKGROUND2); //
  NumpadFrame.drawRoundRect(216, 180, 75, 40, 5, TOLGA_BLUE_2);
  NumpadFrame.drawString("OK", 254, 196);

  NumpadFrame.pushToSprite(&tft, 80, 0, TFT_BLACK);
  NumpadFrame.setTextDatum(TL_DATUM);

}

void drawAlarmFrame (bool numpad, bool Current, bool Cells, bool Rates, bool Balance, int pageNumber, bool Notification, bool NotificationStatus) {


  if (Notification) {
    AlarmsFrame.createSprite(480, 225);
    AlarmsFrame.fillSprite(TFT_BLACK);
    drawNotification(AlarmsFrame , NotificationStatus);
    AlarmsFrame.pushSprite(0, 95);
  }

  else {


    if (numpad) {
      AlarmsFrame.createSprite(480, 225);
      AlarmsFrame.fillSprite(TFT_BLACK);
      drawNumpad(AlarmsFrame);
      AlarmsFrame.pushSprite(0, 95);
    }

    else  {
      AlarmsFrame.createSprite(480, 175);
      AlarmsFrame.fillSprite(TFT_BLACK);
      AlarmsFrame.setFreeFont(&Orbitron_Light_32);
      AlarmsFrame.setTextColor(TFT_WHITE);


      switch (pageNumber) {
        case 6:
          AlarmsFrame.drawString("High Curr.", 42, 20);
          AlarmsFrame.drawString("Discharge", 42, 50);
          AlarmsFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(HCD, 1) + "A", 350, 30);

          AlarmsFrame.setTextDatum(TL_DATUM);
          AlarmsFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          AlarmsFrame.setTextColor(TFT_WHITE);
          AlarmsFrame.drawString("SET", 389, 70);


          AlarmsFrame.drawString("High Curr.", 42, 100);
          AlarmsFrame.drawString("Charge", 42, 130);
          AlarmsFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(HCC, 1) + "A", 350, 110);
          AlarmsFrame.setTextDatum(TL_DATUM);





          break;

        case 7:
          AlarmsFrame.drawString("High Volt.", 42, 20 );
          AlarmsFrame.drawString("Cell", 42, 50 );
          AlarmsFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(HVC, 1) + "V", 350, 30);

          AlarmsFrame.setTextDatum(TL_DATUM);
          AlarmsFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          AlarmsFrame.setTextColor(TFT_WHITE);
          AlarmsFrame.drawString("SET", 389, 70);


          AlarmsFrame.drawString("Low Volt.", 42, 100);
          AlarmsFrame.drawString("Cell", 42, 130);
          AlarmsFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(LVC, 1) + "V", 350, 110);


          break;

        case 8:

          AlarmsFrame.drawString("Rated", 42, 20);
          AlarmsFrame.drawString("Capacity", 42, 50);
          AlarmsFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(RC, 1) + "W", 350, 30);

          AlarmsFrame.setTextDatum(TL_DATUM);
          AlarmsFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          AlarmsFrame.setTextColor(TFT_WHITE);
          AlarmsFrame.drawString("SET", 389, 70);

          AlarmsFrame.drawString("Rated", 42, 100);
          AlarmsFrame.drawString("Voltage", 42, 130);
          AlarmsFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(RV, 1) + "V", 350, 110);



          break;

        case 9:

          AlarmsFrame.drawString("Balance", 42, 20);
          AlarmsFrame.drawString("Voltage", 42, 50);
          AlarmsFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(BV , 1) + "V", 350, 30);

          AlarmsFrame.setTextDatum(TL_DATUM);
          AlarmsFrame.setTextColor(TFT_WHITE);
          AlarmsFrame.drawString("Balance", 42, 100);
          AlarmsFrame.drawString("DeltaV", 42, 130);
          AlarmsFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          AlarmsFrame.setTextColor(TOLGA_BLUE);
          AlarmsFrame.setTextDatum(TR_DATUM);
          AlarmsFrame.drawString(String(BD, 1) + "mV", 350, 110);

          AlarmsFrame.setTextDatum(TL_DATUM);
          AlarmsFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          AlarmsFrame.setTextColor(TFT_WHITE);
          AlarmsFrame.drawString("SET", 389, 70);


          break;

        default:

          break;
      }

      TabFrameAlarms.createSprite(480, 50);
      TabFrameAlarms.setTextDatum(BC_DATUM);
      TabFrameAlarms.setTextColor(TFT_WHITE);
      TabFrameAlarms.fillSprite(TFT_BLACK);
      TabFrameAlarms.setFreeFont(&Orbitron_Light_32);

      TabIcon.createSprite(115, 44);
      TabIcon.setSwapBytes(true);
      TabIcon.pushImage(0, 0, 115, 44, tabMini);


      TabFrameAlarms.setTextColor(TFT_WHITE);

      if (Current) {
        TabIcon.pushToSprite(&TabFrameAlarms, 2, 0, TFT_BLACK);
        TabFrameAlarms.setTextColor(TOLGA_BLUE_2);
        TabFrameAlarms.drawString("Curr.", 60, 44);
      }
      else {
        TabIcon.pushToSprite(&TabFrameAlarms, 2, 5, TFT_BLACK);
        TabFrameAlarms.setTextColor(TFT_WHITE);
        TabFrameAlarms.drawString("Curr.", 60, 44);
      }

      /////////////////////////////////////////
      if (Cells) {
        TabIcon.pushToSprite(&TabFrameAlarms, 122, 0, TFT_BLACK);
        TabFrameAlarms.setTextColor(TOLGA_BLUE_2);
        TabFrameAlarms.drawString("Cells", 180, 44);

      }
      else {
        TabIcon.pushToSprite(&TabFrameAlarms, 122, 5, TFT_BLACK);
        TabFrameAlarms.setTextColor(TFT_WHITE);
        TabFrameAlarms.drawString("Cells", 180, 44);
      }

      ////////////////////////////////////////
      if (Rates) {
        TabIcon.pushToSprite(&TabFrameAlarms, 242, 0, TFT_BLACK);
        TabFrameAlarms.setTextColor(TOLGA_BLUE_2);
        TabFrameAlarms.drawString("Rates", 300, 44);
      }
      else {
        TabIcon.pushToSprite(&TabFrameAlarms, 242, 5, TFT_BLACK);
        TabFrameAlarms.setTextColor(TFT_WHITE);
        TabFrameAlarms.drawString("Rates", 300, 44);;
      }

      ////////////////////////////////////////
      if (Balance) {

        TabIcon.pushToSprite(&TabFrameAlarms, 362, 0, TFT_BLACK);
        TabFrameAlarms.setTextColor(TOLGA_BLUE_2);
        TabFrameAlarms.drawString("Blnc.", 420, 44);
      }
      else {

        TabIcon.pushToSprite(&TabFrameAlarms, 362, 5, TFT_BLACK);
        TabFrameAlarms.setTextColor(TFT_WHITE);
        TabFrameAlarms.drawString("Blnc.", 420, 44);
      }

      TabFrameAlarms.setTextDatum(TL_DATUM);

      TabFrameAlarms.pushSprite(0, 270);
      AlarmsFrame.pushSprite(0, 95);

      AlarmsFrame.deleteSprite();
      TabFrameAlarms.deleteSprite();
      TabIcon.deleteSprite();
      AlarmsFrame.setTextDatum(TL_DATUM);
    }
  }
}

void drawNotification(TFT_eSprite & tft , bool Status) {

  NotificationFrame.createSprite(230, 145);
  NotificationFrame.setTextColor(TFT_WHITE);
  NotificationFrame.fillSprite(TFT_BLACK);
  NotificationFrame.setFreeFont(&Orbitron_Light_32);
  NotificationFrame.fillRoundRect(0, 0, 230, 145, 5, TFT_BACKGROUND5);
  NotificationFrame.fillRect(0, 0, 230, 32, TFT_RED);
  NotificationFrame.setTextColor(TFT_WHITE);
  NotificationFrame.setTextDatum(MC_DATUM);

  if (Status) {
    NotificationFrame.drawString("Parameters", 115, 50);
    NotificationFrame.drawString("Setted", 115, 80);
    NotificationFrame.drawString("Successfully", 115, 110);
  }
  else {
    NotificationFrame.drawString("Setting", 115, 50);
    NotificationFrame.drawString("Parameters", 115, 80);
    NotificationFrame.drawString("Failed", 115, 110);
  }


  NotificationFrame.setTextDatum(TL_DATUM);
  NotificationFrame.pushToSprite(&tft, 125, 40, TFT_BLACK);
  notificationCounter++;
  Serial.println("Notification counter:" + String(notificationCounter));

  if (notificationCounter >= 5) {
    notificationEnable = false;
    notificationCounter = 0;
  }

}

void drawEnergies(float ChargeEnergy, float DischargeEnergy, bool confirmation) {

  if (confirmation)
  {
    EnergiesFrame.createSprite(480, 225);
    EnergiesFrame.fillSprite(TFT_BLACK);
    drawConfirmation(EnergiesFrame);
    EnergiesFrame.pushSprite(0, 95);

  }
  else {

    float RTE;
    if (ChargeEnergy != 0) {
      RTE = (DischargeEnergy / ChargeEnergy) * 100;
    }
    else {
      RTE = 0;
    }

    EnergiesFrame.createSprite(480, 225);
    EnergiesFrame.fillSprite(TFT_BLACK);
    energyIcon.createSprite(48, 48);
    energyIcon.setSwapBytes(true);
    energyIcon.pushImage(0, 0, 48, 48, energyCD);


    //Charge Energy
    EnergiesFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    ChargeIcon.createSprite(48, 48);
    ChargeIcon.setSwapBytes(true);
    ChargeIcon.pushImage(0, 0, 48, 48, maxIcon);
    ChargeIcon.pushToSprite(&EnergiesFrame, 15, 15, TFT_BLACK);
    EnergiesFrame.drawString("Charge ", 70, 15, 2);
    EnergiesFrame.drawString("Energy ", 72, 30, 2);
    EnergiesFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    EnergiesFrame.drawString(String(ChargeEnergy, 3) + "kW", 72, 50, 2);

    //Dsicharge Energy
    EnergiesFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    DischargeIcon.createSprite(48, 48);
    DischargeIcon.setSwapBytes(true);
    DischargeIcon.pushImage(0, 0, 48, 48, voltage);
    DischargeIcon.pushToSprite(&EnergiesFrame, 340, 20, TFT_BLACK);
    EnergiesFrame.drawString("Discharge ", 395, 20, 2);
    EnergiesFrame.drawString("Energy ", 397, 35, 2);
    EnergiesFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    EnergiesFrame.drawString(String(DischargeEnergy, 3) + "kW", 397, 55, 2);


    //CycleNumber
    EnergiesFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    CycleIcon.createSprite(48, 48);
    CycleIcon.setSwapBytes(true);
    CycleIcon.pushImage(0, 0, 48, 48, minIcon);
    CycleIcon.pushToSprite(&EnergiesFrame, 15, 90, TFT_BLACK);
    EnergiesFrame.drawString("Cycle ", 70, 90, 2);
    EnergiesFrame.drawString("Number ", 72, 105, 2);
    EnergiesFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    EnergiesFrame.drawString(String(ChargeEnergy / 10, 1) + "", 72, 125, 2);


    //Round Trip Efficincy
    EnergiesFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    RTEIcon.createSprite(48, 48);
    RTEIcon.setSwapBytes(true);
    RTEIcon.pushImage(0, 0, 48, 48, current);
    RTEIcon.pushToSprite(&EnergiesFrame, 340, 90, TFT_BLACK);
    EnergiesFrame.drawString("Round T. ", 395, 90, 2);
    EnergiesFrame.drawString("Efficiency", 397, 105, 2);
    EnergiesFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    EnergiesFrame.drawString(String(RTE, 1) + "%", 397, 125, 2);

    EnergiesFrame.fillRoundRect(230, 35, 20, 20, 5, TFT_LIGHTGREY);
    EnergiesFrame.fillRoundRect(200, 40, 80, 100, 10, TFT_DARKGREY);


    float a = random(0, 100);
    a = 20;

    EnergiesFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    EnergiesFrame.fillRoundRect(200, 140 - a, 80, a, 10, TOLGA_GREEN);
    EnergiesFrame.drawString("Remain. Cap.", 200, 160, 2);
    EnergiesFrame.setTextColor(TFT_YELLOW, TFT_BLACK);
    EnergiesFrame.drawString("9.45Ah", 220, 180, 2);

    for (int i = 0; i < 10; i++) {
      EnergiesFrame.drawLine(190, 40 + (i * 10), 195, 40 + (i * 10), TOLGA_BLUE);
    }
    EnergiesFrame.pushSprite(0, 95);





  }
}

void drawAlarmsList() {

  int xcoor = 40;
  int ycoor = 20;
  int errorno = 0;
  AlarmListFrame.createSprite(480, 225);
  AlarmListFrame.fillSprite(TFT_BLACK);

  AlarmListFrame.setTextDatum(TL_DATUM);
  AlarmListFrame.setFreeFont(&Orbitron_Light_32);

  for (int i = 0; i < 28; i++) {

    if (BMS.error[i]) {


      if (errorno < 5) {
        AlarmListFrame.fillRoundRect(xcoor - 20, ycoor + errorno * 50, 220, 30, 5, TOLGA_RED);
        AlarmListFrame.setTextColor(TOLGA_BLUE);
        AlarmListFrame.drawString(String(errorno + 1) + ".", xcoor, ycoor + errorno * 50);
        AlarmListFrame.setTextColor(TFT_WHITE);
        AlarmListFrame.drawString(AlarmArray[i], xcoor + 30, ycoor + errorno * 50);
      }
      else {
        AlarmListFrame.fillRoundRect(xcoor + 220, ycoor + (errorno - 5) * 50, 220, 30, 5, TOLGA_RED);
        AlarmListFrame.setTextColor(TOLGA_BLUE);
        AlarmListFrame.drawString(String(errorno + 1) + ".", xcoor + 240, ycoor + (errorno - 5) * 50);
        AlarmListFrame.setTextColor(TFT_WHITE);
        AlarmListFrame.drawString(AlarmArray[i], xcoor + 270, ycoor + (errorno - 5) * 50);

      }
      errorno++;
    }
  }
  AlarmListFrame.pushSprite(0, 95);
}



void drawSOCCalibrate(bool numpad, bool Notification) {


  if (Notification) {
    SOCCalibrateFrame.createSprite(480, 225);
    SOCCalibrateFrame.fillSprite(TFT_BLACK);
    drawNotification(SOCCalibrateFrame , true);
    SOCCalibrateFrame.pushSprite(0, 95);
  }
  else {



    if (numpad) {
      SOCCalibrateFrame.createSprite(480, 225);
      SOCCalibrateFrame.fillSprite(TFT_BLACK);
      drawNumpad(SOCCalibrateFrame);
      SOCCalibrateFrame.pushSprite(0, 95);
    }
    else {

      SOCCalibrateFrame.createSprite(480, 225);
      SOCCalibrateFrame.fillSprite(TFT_BLACK);
      SOCCalibrateFrame.setTextDatum(TL_DATUM);
      SOCCalibrateFrame.setFreeFont(&Orbitron_Light_32);

      SOCCalibrateFrame.drawString("SOC", 42, 20);
      SOCCalibrateFrame.drawString("Percent.", 42, 50);
      SOCCalibrateFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
      SOCCalibrateFrame.setTextColor(TOLGA_BLUE);
      SOCCalibrateFrame.setTextDatum(TR_DATUM);
      SOCCalibrateFrame.drawString(String(SOC_CAL_VAL, 1) + "%", 350, 30);

      SOCCalibrateFrame.setTextDatum(TL_DATUM);
      SOCCalibrateFrame.fillRoundRect(380, 18, 70, 60, 5, TFT_RED);
      SOCCalibrateFrame.setTextColor(TFT_WHITE);
      SOCCalibrateFrame.drawString("SET", 389, 30);

      SOCCalibrateFrame.pushSprite(0, 95);
    }
  }

}


void drawConfirmation(TFT_eSprite & tft) {

  ConfirmationFrame.createSprite(480, 225);
  ConfirmationFrame.setTextDatum(TL_DATUM);
  ConfirmationFrame.setTextColor(TFT_WHITE);
  ConfirmationFrame.fillSprite(TFT_BLACK);
  ConfirmationFrame.setFreeFont(&Orbitron_Light_32);


  ConfirmationFrame.drawString("YES", 80, 115);

  ConfirmationFrame.fillRoundRect(80, 20, 320, 180, 5, TFT_BACKGROUND); //8
  ConfirmationFrame.setTextDatum(MC_DATUM);
  ConfirmationFrame.drawString("Are you sure?", 240, 50);
  ConfirmationFrame.setTextDatum(TL_DATUM);

  ConfirmationFrame.fillRoundRect(100, 100, 100, 60, 5, TOLGA_RED); //8
  ConfirmationFrame.drawRoundRect(100, 100, 100, 60, 5, TOLGA_BLUE_2);
  ConfirmationFrame.drawString("YES", 120, 115);


  ConfirmationFrame.fillRoundRect(280, 100, 100, 60, 5, TOLGA_RED); //8
  ConfirmationFrame.drawRoundRect(280, 100, 100, 60, 5, TOLGA_BLUE_2);
  ConfirmationFrame.drawString("NO", 310, 115);

  ConfirmationFrame.pushToSprite(&tft, 0, 0, TFT_BLACK);
  ConfirmationFrame.setTextDatum(TL_DATUM);

}


void drawMosfets(bool Notification, bool Status) {
  if (Notification) {
    MosfetsFrame.createSprite(480, 225);
    MosfetsFrame.fillSprite(TFT_BLACK);
    drawNotification(MosfetsFrame , Status);
    MosfetsFrame.pushSprite(0, 95);
  }
  else {
    MosfetsFrame.createSprite(480, 225);
    MosfetsFrame.setTextDatum(TL_DATUM);
    MosfetsFrame.setTextColor(TFT_WHITE);
    MosfetsFrame.fillSprite(TFT_BLACK);
    MosfetsFrame.setFreeFont(&Orbitron_Light_32);

    MosfetsFrame.fillRoundRect(20, 20, 210, 190, 5, TFT_BACKGROUND);
    MosfetsFrame.fillRoundRect(20, 20, 210, 30, 5, TOLGA_RED);
    MosfetsFrame.setTextColor(TOLGA_BLUE);
    MosfetsFrame.drawString("Read", 100, 20);

    MosfetsFrame.fillRoundRect(250, 20, 210, 190, 5, TFT_BACKGROUND);
    MosfetsFrame.fillRoundRect(250, 20, 210, 30, 5, TOLGA_RED);
    MosfetsFrame.setTextColor(TOLGA_BLUE);
    MosfetsFrame.drawString("Write", 320, 20);

    SwitchOnIcon.createSprite(48, 48);
    SwitchOnIcon.setSwapBytes(true);
    SwitchOnIcon.pushImage(0, 0, 48, 48, switchon);

    SwitchOffIcon.createSprite(48, 48);
    SwitchOffIcon.setSwapBytes(true);
    SwitchOffIcon.pushImage(0, 0, 48, 48, switchoff);

    MosfetsFrame.drawString("Charge", 40, 60);
    MosfetsFrame.drawString("Disch.", 40, 110);
    MosfetsFrame.drawString("Balance", 40, 160);

    if (BMS.charge) {
      SwitchOnIcon.pushToSprite(&MosfetsFrame, 165, 55, TFT_BLACK);
    }
    else {
      SwitchOffIcon.pushToSprite(&MosfetsFrame, 165, 55, TFT_BLACK);
    }

    if (BMS.discharge) {
      SwitchOnIcon.pushToSprite(&MosfetsFrame, 165, 105, TFT_BLACK);
    }
    else {
      SwitchOffIcon.pushToSprite(&MosfetsFrame, 165, 105, TFT_BLACK);
    }

    if (balancerState) {
      SwitchOnIcon.pushToSprite(&MosfetsFrame, 165, 155, TFT_BLACK);
    }
    else {
      SwitchOffIcon.pushToSprite(&MosfetsFrame, 165, 155, TFT_BLACK);
    }

    if (WriteCharge) {
      SwitchOnIcon.pushToSprite(&MosfetsFrame, 280, 55, TFT_BLACK);
    }
    else {
      SwitchOffIcon.pushToSprite(&MosfetsFrame, 280, 55, TFT_BLACK);
    }

    if (WriteDischarge) {
      SwitchOnIcon.pushToSprite(&MosfetsFrame, 280, 105, TFT_BLACK);
    }
    else {
      SwitchOffIcon.pushToSprite(&MosfetsFrame, 280, 105, TFT_BLACK);
    }

    if (WriteBalance) {
      SwitchOnIcon.pushToSprite(&MosfetsFrame, 280, 155, TFT_BLACK);
    }
    else {
      SwitchOffIcon.pushToSprite(&MosfetsFrame, 280, 155, TFT_BLACK);
    }

    MosfetsFrame.fillRoundRect(360, 60, 80, 40, 5, TOLGA_RED);
    MosfetsFrame.drawString("SET", 370, 65);

    MosfetsFrame.fillRoundRect(360, 110, 80, 40, 5, TOLGA_RED);
    MosfetsFrame.drawString("SET", 370, 115);

    MosfetsFrame.fillRoundRect(360, 160, 80, 40, 5, TOLGA_RED);
    MosfetsFrame.drawString("SET", 370, 165);
    MosfetsFrame.pushSprite(0, 95);
  }
}


void drawInfo() {

  if (loading) {
    InfoFrame.createSprite(480, 225);
    InfoFrame.setTextDatum(TL_DATUM);
    InfoFrame.setTextColor(TFT_WHITE);
    InfoFrame.fillSprite(TFT_BLACK);
    drawLoading(InfoFrame, loadingPercentage);
    InfoFrame.pushSprite(0, 95);
  }
  else {
    loadingPercentage = 0;
    InfoFrame.createSprite(480, 225);
    InfoFrame.setTextDatum(TL_DATUM);
    InfoFrame.setTextColor(TFT_WHITE);
    InfoFrame.fillSprite(TFT_BLACK);
    InfoFrame.setFreeFont(&Orbitron_Light_32);

    InfoFrame.drawRoundRect(10, 10, 463, 200, 5, TFT_BACKGROUND);
    InfoFrame.drawLine(10, 39, 469, 39, TFT_BACKGROUND);
    InfoFrame.drawLine(10, 68, 469, 68, TFT_BACKGROUND);
    InfoFrame.drawLine(10, 97, 469, 97, TFT_BACKGROUND);
    InfoFrame.drawLine(10, 126, 469, 126, TFT_BACKGROUND);
    InfoFrame.drawLine(10, 155, 469, 155, TFT_BACKGROUND);
    InfoFrame.drawLine(10, 182, 469, 182, TFT_BACKGROUND);

    InfoFrame.drawLine(164, 10, 164, 209, TFT_BACKGROUND);

    InfoFrame.drawLine(315, 10, 315, 209, TFT_BACKGROUND);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("DischCurHi:", 20, 18, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString (String(BMS.dischar_curr2) + "A", 105, 18, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("ChargeCurHi:", 20, 47, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.charge_curr2) + "A", 105, 47, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("SumVoltHi:", 20, 76, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.sumv_high2 / 10) + "V", 105, 76, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("SumVoltLo:", 20, 105, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.sumv_low2 / 10) + "V", 105, 105, 2);


    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("CellVoltHi:", 20, 134, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.cell_volthigh2 * 0.001, 3) + "V", 105, 134, 2);

    Serial.print("cumu charge");
    Serial.println(BMS.cumilative_charge);
    delay(10);
    Serial.print("cumu discharge");
    Serial.println(BMS.cumilative_discharge);




    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("CellVoltLo:", 20, 163, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.cell_voltlow2 * 0.001, 3) + "V", 105, 163, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("RatedCap:", 20, 190, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.rated_cap, 4) + "Ah", 105, 190, 2);

    Serial.println(BMS.rated_cap, 4);


    /////////////////////////////////////////////

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("BalVolt:", 180, 18, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.balance_volt * 0.001, 3) + "V", 255, 18, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("BalDeltaV:", 180, 47, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.balance_volt_diff) + "mV", 255, 47, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("ChgTempHi:", 180, 76, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.charge_temp_high2) + "C", 255, 76, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("ChgTempLo:", 180, 105, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.charge_temp_low2) + "C", 255, 105, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("DisTempHi:", 180, 134, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.discharge_temp_high1) + "C", 255, 134, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("DisTempLo:", 180, 163, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.discharge_temp_low2) + "C", 255, 163, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("RatedVolt:", 180, 190, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.rated_volt * 0.001, 3) + "V", 255, 190, 2);

    ///////////////////////////////////////////////////////

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("SOCHi", 325, 18, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.SOC_high2) + "%", 398, 18, 2);



    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("SOCLo:", 325, 47, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.SOC_low2) + "%", 398, 47, 2);







    /////////////////////////////////////////


    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("VoltDif:", 325, 76, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(BMS.volt_diff2) + "mV", 400, 76, 2);


    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("TotChrg:", 325, 105, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(ChargeEnergyFixed) + "kWh", 400, 105, 2);


    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("TotDisch:", 325, 134, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( String(DischargeEnergyFixed) + "kWh", 400, 134, 2);


    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("IP:", 325, 163, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( IP , 360, 163, 2);

    InfoFrame.setTextColor(TOLGA_BLUE);
    InfoFrame.drawString("SSID:", 325, 190, 2);
    InfoFrame.setTextColor(TOLGA_RED);
    InfoFrame.drawString( SSIDName , 360, 190, 2);
    InfoFrame.pushSprite(0, 95);
  }
}


void drawLoading(TFT_eSprite & tft, int percentage) {

  LoadingFrame.createSprite(480, 225);
  LoadingFrame.setTextDatum(TL_DATUM);
  LoadingFrame.setTextColor(TFT_WHITE);
  LoadingFrame.fillSprite(TFT_BLACK);
  LoadingFrame.setFreeFont(&Orbitron_Light_32);

  LoadingFrame.fillRoundRect(80, 20, 320, 70, 5, TFT_BACKGROUND); //8
  LoadingFrame.setTextDatum(MC_DATUM);
  LoadingFrame.drawString("Loading please wait", 240, 50);
  LoadingFrame.fillRoundRect(80, 85, 320, 40, 5, TOLGA_RED); //8
  LoadingFrame.drawString(String(percentage) + "%", 240, 100);
  LoadingFrame.setTextDatum(TL_DATUM);

  LoadingFrame.pushToSprite(&tft, 0, 0, TFT_BLACK);
  LoadingFrame.setTextDatum(TL_DATUM);
  LoadingFrame.deleteSprite();
}








///SPIFFS FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
#define FORMAT_SPIFFS_IF_FAILED true

void listDir(fs::FS & fs, const char * dirname, uint8_t levels) {

  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      fileName = file.name();
      Serial.println(fileName);

    }
    file = root.openNextFile();
  }
}

String readFile(fs::FS & fs, const char * path) {
  // Serial.printf("Reading file: %s\r\n", path);
  char buffer[64];

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");

    return "";
  }
  uint16_t i = 0;
  while (file.available()) {
    VALUE [i] = file.read();
    // Serial.print (VALUE [i]); //use for debug
    i++;
  }
  VALUE [i] = '\0';
  Serial.println (VALUE); //use for debug
  file.close();
  Output = String(VALUE);
  return Output;
}

void checkFile(fs::FS & fs, const char * path) {
  Serial.printf("Checking the file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("File is not available, New File is created");
    Serial.println(path);
    writeFile(SPIFFS, path, "");



    return;
  }

  else {
    Serial.println("File is exist");
    file.close();
  }

}

void writeFile(fs::FS & fs, const char * path, const char * message) {
  //Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);

  if (file.print(message)) {
    // Serial.println("- file written");
  } else {
    //Serial.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);

  if (file.print(message)) {
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
  file.close();
}

void renameFile(fs::FS & fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\r\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("- file renamed");
  } else {
    Serial.println("- rename failed");
  }
}

void deleteFile(fs::FS & fs, const char * path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}


void readAll(fs::FS & fs, const char * dirname, uint8_t levels, float DataArray[1440]) {
  //Serial.printf("Listing directory: %s\r\n", dirname);


  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return ;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      // Serial.print("  DIR : ");
      //  Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      fileName = file.name();

      fileName = fileName.substring(1);
      fileindex = fileName.indexOf("/");
      FolderName = fileName.substring(0, fileindex);

      fileName = fileName.substring(fileindex + 1);
      fileindex = fileName.indexOf("/");
      Day = fileName.substring(0, fileindex);


      fileName = fileName.substring(fileindex + 1);
      fileindex = fileName.indexOf("/");
      Hour = fileName.substring(0, fileindex);


      fileName = fileName.substring(fileindex + 1);
      fileindex = fileName.indexOf("/");
      Minute = fileName.substring(0, fileindex);
      MinuteIndex = Hour.toInt() * 60 + Minute.toInt();

      DataArray[MinuteIndex] = readFile(fs, file.name()).toFloat();

    }
    file = root.openNextFile();
  }
  return;
}


void firmwareUpdate(void) {
  updateStarted = true;
  PageNumber = 15;
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  // httpUpdate.setLedPin(LED_BUILTIN, LOW);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
  rtc_wdt_protect_on();
  rtc_wdt_enable();
  updateStarted = false;

}
int FirmwareVersionCheck(void) {
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure * client = new WiFiClientSecure;

  if (client)
  {

    client -> setCACert(rootCACertificate);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
    HTTPClient https;

    if (https.begin( * client, fwurl))
    { // HTTPS


      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      } else {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();


    }
    delete client;

  }

  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer)) {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    }
    else
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}

void repeatedCall() {

  if (WiFi.status() == WL_CONNECTED) {

    esp_task_wdt_init(60, false);
    static int num = 0;
    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis) >= interval) {
      previousMillis = currentMillis;
      if (FirmwareVersionCheck()) {
        firmwareUpdate();
      }
    }
    else {


    }
  }

  esp_task_wdt_init(60, true);
}

void drawUpload() {
  UploadingFirmwareFrame.createSprite(480, 225);
  UploadingFirmwareFrame.setTextDatum(TL_DATUM);
  UploadingFirmwareFrame.setTextColor(TFT_WHITE);
  UploadingFirmwareFrame.fillSprite(TFT_BLACK);
  UploadingFirmwareFrame.setFreeFont(&Orbitron_Light_32);
  UploadingFirmwareFrame.fillRoundRect(80, 20, 320, 120, 5, TFT_BACKGROUND); //8
  UploadingFirmwareFrame.setTextDatum(MC_DATUM);
  UploadingFirmwareFrame.drawString("Firmware is Updating,", 240, 50);
  UploadingFirmwareFrame.drawString("Please Wait....", 240, 90);
  UploadingFirmwareFrame.setTextDatum(TL_DATUM);
  UploadingFirmwareFrame.pushSprite(0, 95);
}

String GetPublicIP() {

  String payload;

  //GET PUBLIC IP
  HTTPClient http;
  http.begin("https://api.ipify.org/?format=json"); //Specify the URL
  int httpCode = http.GET();                        //Make the request

  if (httpCode > 0) { //Check for the returning code

    payload = http.getString();
    JSONVar myObject = JSON.parse(payload);
    PublicIP = myObject["ip"];
    GetCoordinates(PublicIP);




  }
  else {
    Serial.println("Error on HTTP request");
  }

  http.end();
  return payload;
}



String GetCoordinates(String IP) {
  String payload;

  //GET PUBLIC IP
  HTTPClient http;

  String URL = "https://ipapi.co/" + PublicIP + "/latlong";

  http.begin(URL.c_str()); //Specify the URL
  int httpCode = http.GET();                        //Make the request

  if (httpCode > 0) { //Check for the returning code
    payload = http.getString();
    Coordinates = payload;



  }
  else {
    Serial.println("Error on HTTP request");
  }

  http.end();
  return payload;
}
