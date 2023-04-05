//3-22_2023

//selftest
bool SELFTEST[3] = {false, false, false}; //(RTC,SD,CANBUS) B


//osman libraries
#include <gpio.h>
#include <TCA9548A.h>
#include <rtc_data.h>
#include <mcp2515_can.h>
#include <bms_spi.h>
#include <controlsd.h>
#include <bmsequ_comm2.h>
#include <bms_comm2.h>
#include <SC16S740.h>
#include <canbus.h>

//QR code
#include "qrcode.h"
QRCode qrcode;
bool ShowBarcode = false;
/////////


//gpio read
uint8_t GPIO_READ[3] = {0, 0, 0};

//



uint8_t bmsCounter = 0;

uint8_t board1[3] = {0, 0, 0};
bool buzzerActive = false;
bool control_sd = false;
const int SD_CS_PIN = 12;
String SDCard_row[26] = {"TimeStamp", "TerminalV", "TerminalC", "CellT", "MaxV", "MinV", "MaxDiff", "SOC" "ChargeE", "DischargeE",
                         "Cell#1", "Cell#2", "Cell#3", "Cell#4", "Cell#5", "Cell#6", "Cell#7", "Cell#8", "Cell#9", "Cell#10", "Cell#11", "Cell#12", "Cell#13", "Cell#14", "Cell#15", "Cell#16"
                        };
//canbus
uint8_t alarmsr[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 13 alarms



//DRY CONTACT LIST
bool ContactNameActive = false;
bool ContactTypeActive = false;
bool ContactEnableActive = false;
bool ContactDisableActive = false;
bool OperationActive = false;

int ContactNameIndex = 0;
int ContactTypeIndex = 0;
int OperationTypeIndex = 0;

int DRY_VALUES[4] = {0, 0, 0, 0};

String DRYA_REMAINING;
String DRYA_ARRAY[4];

String DRYB_REMAINING;
String DRYB_ARRAY[4];

String DRYC_REMAINING;
String DRYC_ARRAY[4];

String DRYD_REMAINING;
String DRYD_ARRAY[4];


String DRYA = "";
String DRYB = "";
String DRYC = "";
String DRYD = "";



bool mute = false;

bool RTCCongifured = false;
bool RTCFetched = false;
int saat ;
int dakika;
int saniye;
int gun;
int ay;
int yil;


bool SDReset = false;
File rootSD;
int DeletedCount = 0;
int FolderDeleteCount = 0;
int FailCount = 0;
String rootpath = "/";

uint8_t saatUInt = (uint8_t)saat;
uint8_t dakikaUInt = (uint8_t)dakika;
uint8_t saniyeUInt = (uint8_t)saniye;
uint8_t gunUInt = (uint8_t)gun;
uint8_t ayUInt = (uint8_t)ay;
int timeoffset = 3600;

String OperationList[2] = {"<=", ">"};
String ContactNameList[4] = {"Contact A", "Contact B", "Contact C", "Contact D"};
String ContactFunctionList[6] = {"Term.Volt.", "abs(Current)", "Temperature", "SOC", "DISABLE", "ENABLE"};



#include "BluetoothSerial.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "cert.h"
#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>

TaskHandle_t FIREBASE;
TaskHandle_t UDPTEST;
TaskHandle_t TOUCH_SCREEN;
TaskHandle_t BMS_COMM;
TaskHandle_t BT;
TaskHandle_t ENCONNECT;
TaskHandle_t SDCANBUS;

bool WIFIBT = false;

//ENCONNECT

typedef struct
{
  String filename;
  String ftype;
  String fsize;
} fileinfo;

String   webpage, MessageLine;
fileinfo Filenames[200]; // Enough for most purposes!
bool     StartupErrors = false;
int      start, downloadtime = 1, uploadtime = 1, downloadsize, uploadsize, downloadrate, uploadrate, numfiles;



#include "Free_Fonts.h"
//#include "Header_2.h"
#include "maxIcon.h"
#include "minIcon.h"
#include "delta.h"
#include "voltage.h"
#include "current.h"
#include "temp.h"
#include "notification.h"
#include "wifi.h"
#include "bluetooth.h"
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
#include "update.h"
#include "nowifi.h"
#include "drycontacts.h"

#include "offequ.h"
#include "onequ.h"


#include "dryread.h"
#include "drywrite.h"
#include "mute.h"
#include "unmute.h"


#include "firebasedirect.h"
#include "wifidirect.h"
#include "bluetoothdirect.h"
#include "firebasedirectdis.h"
#include "wifidirectdis.h"
#include "bluetoothdirectdis.h"
#include "encap2.h"
#include "Banner1.h"
#include "up.h"
#include "down.h"
#include "save.h"


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


#include <ArduinoJson.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include "time.h"
#include <DNSServer.h>
#include <Wifi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>
#include <Arduino.h>
#include "AsyncUDP.h"

//ENCONNECT
///////////////////////////
// Built-in
#include <WebServer.h>
#include <ESPmDNS.h>
#include "CSS.h"

WebServer server(80);



////////////////////////////



String SerialNumber;
boolean firebaseReady = false;

float AllCells[16];
String UDPprocessor = "";
////////////////////

#include <Wire.h>
#include <Adafruit_FT6206.h>
Adafruit_FT6206 ts = Adafruit_FT6206();
#include <Preferences.h>



Preferences preferences;
WiFiManager wifiManager;

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite PageFrame = TFT_eSprite(&tft);
TFT_eSprite ContentFrame = TFT_eSprite(&tft);
TFT_eSprite BannerFrame = TFT_eSprite(&tft);
TFT_eSprite NotificationFrame = TFT_eSprite(&tft);
TFT_eSprite GraphFrame = TFT_eSprite(&tft);


TFT_eSprite Icon1 = TFT_eSprite(&tft);
TFT_eSprite Icon2 = TFT_eSprite(&tft);
TFT_eSprite Icon3 = TFT_eSprite(&tft);
TFT_eSprite Icon4 = TFT_eSprite(&tft);
TFT_eSprite Icon5 = TFT_eSprite(&tft);
TFT_eSprite Icon6 = TFT_eSprite(&tft);
TFT_eSprite Icon7 = TFT_eSprite(&tft);
TFT_eSprite Icon8 = TFT_eSprite(&tft);
TFT_eSprite Icon9 = TFT_eSprite(&tft);
TFT_eSprite Icon10 = TFT_eSprite(&tft);
TFT_eSprite Icon11 = TFT_eSprite(&tft);
TFT_eSprite Icon12 = TFT_eSprite(&tft);
TFT_eSprite Icon13 = TFT_eSprite(&tft);
TFT_eSprite Icon14 = TFT_eSprite(&tft);
TFT_eSprite barcode = TFT_eSprite(&tft);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


/////////////////backlight

const uint8_t ledPin = 23;
// setting PWM properties
const int freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 8;








//WIFI SETTINGS

String SSIDName = "ENCAP Controller";
String Port = "2001";
String MAC = "No Connection";
String IP = "192.168.4.1";
bool wifistatus = false;


//Configuration settings
bool UpdateNextRestart = false;
bool WDNextRestart = false;
bool FBNextRestart = false;
bool BTNextRestart = false;
bool UDPNextRestart = true;
int ModuleCap = 10;
int SystemSOC = 100;
float SystemCAP = 216;

float SUMHI = 57;
float SUMLOW = 57;

float SOCLOW = 0;
float SOCHI = 100;

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


// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5


////TIMER VARIABLES//////////////////
const char* ntpServer = "pool.ntp.org";
long  gmtOffset_sec = 14400;
int   daylightOffset_sec = 3600;

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
String DateString = "No Data";


//UI Parameters
uint8_t PageNumber = 1;
uint8_t PageLevel = 0;
int SleepCounter = 0;
bool EEPROMbusy = false;
bool balancerState = false;

unsigned long previousMillisUI = 0;
unsigned long intervalUI = 250;
unsigned long currentMillisUI = 0;

bool numpadEnable = false;
String numpadValue = "";
bool notificationEnable = false;
uint8_t notificationCounter = 0;
bool CommandStatus = false;


//BMS Parameters.....

float CVD = 0;
int restartCounter = 0;
float HCD = 0;
float HCC = 0;
float HVC = 0;
float LVC = 0;
float RC = 0;
float RV = 0;
float BV = 0;
float BD = 0;
float HTC = 0;
float HTD = 0;
float SOC_CAL_VAL = 0;
uint16_t SLP = 3600;

bool HCDEnable = false;
bool HCCEnable = false;
bool HVCEnable = false;
bool LVCEnable = false;
bool RCEnable = false;
bool RVEnable = false;
bool BVEnable = false;
bool BDEnable = false;
bool SOCEnable = false;

float DryContactValueEnable;
float DryContactValueDisable;
bool DCEnable = false;
bool DCDisable = false;

bool HCC_HCD = false;
bool HVC_LVC = false;
bool RC_RV = false;
bool BV_BD = false;
bool SOC_CAL = false;
bool SOC_ALARM = false;
bool SUM_CAL = false;
bool HTC_HTD = false;
bool SLP_CAL = false;
bool CVD_CAL = false;



//EQU PARAMETER

float HVCEQU = 0;
float LVCEQU = 0;
bool HVC_LVC_EQU = false;


bool CAL_EQUVOLT = false;
float EQUVOLT_VAL = 0;

bool CAL_EQUCURRENT = false;
float EQUCURRENT_VAL = 0;

bool CAL_EQUSLEEP = false;
uint16_t  EQUSLEEP_VAL = 65535;

bool CAL_EQUSUM = false;
float EQUSUM_HI = 0;
float EQUSUM_LOW = 0;



bool ReadAllParameters = false;

//MOSFET SETTING
bool CHARGE_SET = false;
bool DISCHARGE_SET = false;
bool BALANCE_SET = false;
bool updateIcon = false;


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

//#define FORMAT_SPIFFS_IF_FAILED true
//StaticJsonDocument<6000> doc;
//JsonObject Voltage = doc.createNestedObject("Voltage");
/////////////////////////////
//GITHUB UPDATE

String FirmwareVer = {
  "5.4.5"
};


bool UpdateAvailable = false;
bool IPReady = true;
bool CoordinatesReady = true;


#define URL_fw_Version  "https://raw.githubusercontent.com/Emcukcan/FirmwareVersioner/master/bin_version.txt"
#define URL_fw_Bin  "https://raw.githubusercontent.com/Emcukcan/FirmwareVersioner/master/fw.bin"


int Dryindex = 0;

void setup() {
  tft.init();
  //  pinMode(TFT_BL, OUTPUT);
  //  digitalWrite(TFT_BL, 128);l
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  Serial.begin(9600);
  while (!Serial) continue;
  Serial.setTimeout(250);

  //  //  module
  RXD2 = 32;
  TXD2 = 33;


  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //buzzer rtc equalizer setup//////////////////////////////////
  begin_i2cdevice();
  spi_class_begin();
  set_gpio();

  //setup canbus
  CAN.setSPI(hspi);


  for (int i = 0; i < 5; i++) {

    if (CAN_OK == CAN.begin(CAN_500KBPS, MCP_8MHz)) {
      Serial.println("CAN BUS Shield init ok!");
      SELFTEST[2] = true;
      // bool SELFTEST[3] = {false, false, false}; //(RTC,SD,CANBUS)
      break;
    }
    else {
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");
      SELFTEST[2] = false;
      delay(100);
    }
  }



  delay(10);
  uarti2c_begin(9600);
  delay(100);
  device_select(2);
  delay(10);
  RTC.begin(&I2C_0);

  /////////////////////////////////////
  //SD CARD


  control_sd = sd_card_init(SD_CS_PIN , hspi );
  Serial.print("SD Card status:");
  Serial.println(control_sd);
  if (control_sd) {
    SELFTEST[1] = true;
    createDir(SD,  "/");
  }
  else {
    SELFTEST[1] = false;
  }




  //

  //Backlight
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 255);

  EEPROMbusy = true;
  preferences.begin("my-app", false);
  ChargeEnergyFixed = preferences.getFloat("ChargeEnergyFixed", 0);
  DischargeEnergyFixed = preferences.getFloat("DischargeEnergyFixed", 0);
  ChargeEnergy = preferences.getFloat("ChargeEnergy", 0);
  DischargeEnergy = preferences.getFloat("DischargeEnergy", 0);
  SerialNumber = preferences.getString("SerialNumber", "EN02048V00330023A00001");
  ModuleCap = preferences.getInt("ModuleCap", 10);
  timeoffset = preferences.getInt("timeoffset", 3600);


  restartCounter = preferences.getInt("RC", 0);
  UpdateNextRestart = preferences.getBool("UNR", false);
  WDNextRestart = preferences.getBool("WDNR", false);
  FBNextRestart = preferences.getBool("FBNR", false);
  BTNextRestart = preferences.getBool("BTNR", true);
  UDPNextRestart = preferences.getBool("UDPNR", false);


  DRYA = preferences.getString("DRYA", "0/0/0/0/0/");
  DRYB = preferences.getString("DRYB", "0/0/0/0/0/");
  DRYC = preferences.getString("DRYC", "0/0/0/0/0/");
  DRYD = preferences.getString("DRYD", "0/0/0/0/0/");

  DRYA_REMAINING = DRYA;
  DRYB_REMAINING = DRYB;
  DRYC_REMAINING = DRYC;
  DRYD_REMAINING = DRYD;


  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYA_REMAINING.indexOf('/');
    DRYA_ARRAY[i] = DRYA_REMAINING.substring(0, Dryindex);
    DRYA_REMAINING = DRYA_REMAINING.substring(Dryindex + 1);
    //    Serial.println("Number:" + DRYA_ARRAY[i]);
    //    Serial.println("Remaining:" + DRYA_REMAINING);
    //    Serial.println("------------");
  }

  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYB_REMAINING.indexOf('/');
    DRYB_ARRAY[i] = DRYB_REMAINING.substring(0, Dryindex);
    DRYB_REMAINING = DRYB_REMAINING.substring(Dryindex + 1);
    //   Serial.println("Number:" + DRYB_ARRAY[i]);
    //  Serial.println("Remaining:" + DRYB_REMAINING);
    //   Serial.println("------------");
  }

  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYC_REMAINING.indexOf('/');
    DRYC_ARRAY[i] = DRYC_REMAINING.substring(0, Dryindex);
    DRYC_REMAINING = DRYC_REMAINING.substring(Dryindex + 1);
    //  Serial.println("Number:" + DRYC_ARRAY[i]);
    //  Serial.println("Remaining:" + DRYC_REMAINING);
    //  Serial.println("------------");
  }

  for (int i = 0; i < 4; i++) {
    // Serial.println("------------");
    Dryindex = DRYD_REMAINING.indexOf('/');
    DRYD_ARRAY[i] = DRYD_REMAINING.substring(0, Dryindex);
    DRYD_REMAINING = DRYD_REMAINING.substring(Dryindex + 1);
    //  Serial.println("Number:" + DRYD_ARRAY[i]);
    //  Serial.println("Remaining:" + DRYD_REMAINING);
    // Serial.println("------------");
  }


  mute = preferences.getBool("mute", false);


  preferences.end();


  preferences.begin("my-app", false);
  restartCounter++;
  preferences.putInt("RC", restartCounter);
  preferences.end();
  EEPROMbusy = false;

  // SerialNumber = preferences.getString("SerialNumber", "ENC10482000001");

  buzzer_on();
  delay(200);
  buzzer_off();
  //Serial.println("setup is completed");



  //CREATING TASK FOR BMS_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    BMS_COMM_CODE,   /* Task function. */
    "BMS_COMM",     /* name of task. */
    7000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    100,           /* priority of the task */
    &BMS_COMM,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);


  //CREATING TASK FOR BMS_____________________________________________________________________________________________________
  if (FBNextRestart) {
    Serial.println("FB started");
    xTaskCreatePinnedToCore(
      FIREBASE_CODE,   /* Task function. */
      "FIREBASE",     /* name of task. */
      7000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      12,           /* priority of the task */
      &FIREBASE,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }


  //CREATING TASK FOR BMS_____________________________________________________________________________________________________

  if (UDPNextRestart) {
    Serial.println("UDP started");
    xTaskCreatePinnedToCore(
      UDPTEST_CODE,   /* Task function. */
      "UDPTEST",     /* name of task. */
      4000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      13,           /* priority of the task */
      &UDPTEST,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }


  //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    TOUCH_SCREEN_CODE,   /* Task function. */
    "TOUCH_SCREEN",     /* name of task. */
    3000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    96,           /* priority of the task */
    &TOUCH_SCREEN,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(50);



  if (BTNextRestart) {
    Serial.println("Bluetooth started");
    //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
    xTaskCreatePinnedToCore(
      BT_CODE,   /* Task function. */
      "BT",     /* name of task. */
      5000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      2,           /* priority of the task */
      &BT,      /* Task handle to keep track of created task */
      1);          /* pin task to core 0 */
    delay(50);
  }




  drawLandingPage();
  if (WDNextRestart) {
    Serial.println("Wifi-direct started");
    //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
    xTaskCreatePinnedToCore(
      ENCONNECT_CODE,   /* Task function. */
      "ENCONNECT",     /* name of task. */
      9000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      2,           /* priority of the task */
      &ENCONNECT,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(50);
  }


  Serial.println("SD CARD CANBUS Started");
  //CREATING TASK FOR TOUCH SCREEN_____________________________________________________________________________________________________
  xTaskCreatePinnedToCore(
    SDCANBUS_CODE,   /* Task function. */
    "SDCANBUS",     /* name of task. */
    4000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    7,           /* priority of the task */
    &SDCANBUS,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(50);
}


void SDCANBUS_CODE( void * pvParameters ) {

  for (;;) {

    // Serial.println ("SD Card and canbus worked...");

    //SEND CANBUS FRAME

    if (BMS.SOC * 0.1 < 80 && BMS.SOC * 0.1 > 30) {
      can_send_charge(55 , 200, 200);

    }
    else if (BMS.SOC * 0.1 > 80 && BMS.SOC * 0.1 < 90)
    {
      can_send_charge(55 , 100, 200);
    }
    else if (BMS.SOC * 0.1 > 90)
    {
      can_send_charge(55 , 20, 200);
    }
    else if (BMS.SOC * 0.1 < 30 && BMS.SOC * 0.1 > 20)
    {
      can_send_charge(55 , 200, 100);
    }

    else if (BMS.SOC * 0.1 < 20)
    {
      can_send_charge(55 , 200, 20);
    }

    can_send_soc(BMS.SOC * 0.1 , BMS.SOC * 0.1);
    // Serial.println(read_can(),HEX);
    can_send_total_volt(BMS.sum_voltage * 0.1, (BMS.current - 30000) * 0.1, BMS.max_cell_temp - 40);
    can_send_request(1, 1 , 0, 0, 0);
    can_send_alarm(alarmsr) ;
    Serial.println(read_can(), HEX);
    /////////////////////

    //SD_CARD
    if (!SDReset) {
      //Serial.println("SD Card started:" + String(millis()));
      String Address = "/" + String(rtc.days) + "_" + String(rtc.months) + "_" + String(rtc.years) + "-" + SerialNumber.substring(SerialNumber.length() - 5) + ".xls";
      if (!SD.exists(Address)) {
        write_title( Address.c_str());
      }

      String SDCard_row[40] = {"TimeStamp", "TerminalV", "TerminalC", "CellT", "MaxV", "MinV", "MaxDiff", "SOC" "ChargeE", "DischargeE",
                               "Cell#1", "Cell#2", "Cell#3", "Cell#4", "Cell#5", "Cell#6", "Cell#7", "Cell#8", "Cell#9", "Cell#10", "Cell#11", "Cell#12", "Cell#13", "Cell#14", "Cell#15", "Cell#16", "DRYA", "DRYB", "DRYC", "DRYD"
                              };

      SDCard_row[0] = String(rtc.hours) + ":" + String(rtc.minutes) + ":" + String(rtc.seconds);
      SDCard_row[1] = String(BMS.sum_voltage * 0.1, 3);
      SDCard_row[2] = String((BMS.current - 30000) * 0.1, 3);
      SDCard_row[3] = String(BMS.max_cell_temp - 40);
      SDCard_row[4] = String(BMS_equ.max_cell_volt_equ * 0.001, 3);
      SDCard_row[5] = String(BMS_equ.min_cell_volt_equ * 0.001, 3);
      SDCard_row[6] = String(BMS_equ.max_cell_volt_equ * 0.001 - BMS_equ.min_cell_volt_equ * 0.001, 3);
      SDCard_row[7] = String(BMS.SOC * 0.1, 3);
      SDCard_row[8] = String(ChargeEnergy, 3);
      SDCard_row[9] = String(DischargeEnergy, 3);
      SDCard_row[10] = String(BMS_equ.cell_voltage_equ[0] * 0.001, 3);
      SDCard_row[11] = String(BMS_equ.cell_voltage_equ[1] * 0.001, 3);
      SDCard_row[12] = String(BMS_equ.cell_voltage_equ[2] * 0.001, 3);
      SDCard_row[13] = String(BMS_equ.cell_voltage_equ[3] * 0.001, 3);
      SDCard_row[14] = String(BMS_equ.cell_voltage_equ[4] * 0.001, 3);
      SDCard_row[15] = String(BMS_equ.cell_voltage_equ[5] * 0.001, 3);
      SDCard_row[16] = String(BMS_equ.cell_voltage_equ[6] * 0.001, 3);
      SDCard_row[17] = String(BMS_equ.cell_voltage_equ[7] * 0.001, 3);
      SDCard_row[18] = String(BMS_equ.cell_voltage_equ[8] * 0.001, 3);
      SDCard_row[19] = String(BMS_equ.cell_voltage_equ[9] * 0.001, 3);
      SDCard_row[20] = String(BMS_equ.cell_voltage_equ[10] * 0.001, 3);
      SDCard_row[21] = String(BMS_equ.cell_voltage_equ[11] * 0.001, 3);
      SDCard_row[22] = String(BMS_equ.cell_voltage_equ[12] * 0.001, 3);
      SDCard_row[23] = String(BMS_equ.cell_voltage_equ[13] * 0.001, 3);
      SDCard_row[24] = String(BMS_equ.cell_voltage_equ[14] * 0.001, 3);
      SDCard_row[25] = String(BMS_equ.cell_voltage_equ[15] * 0.001, 3);

      if (DRY_VALUES[0] == 0) {
        SDCard_row[26] = "OFF";
      }
      else {
        SDCard_row[26] = "ON";
      }
      if (DRY_VALUES[1] == 0) {
        SDCard_row[27] = "OFF";
      }
      else {
        SDCard_row[27] = "ON";
      }

      if (DRY_VALUES[2] == 0) {
        SDCard_row[28] = "OFF";
      }
      else {
        SDCard_row[28] = "ON";
      }

      if (DRY_VALUES[3] == 0) {
        SDCard_row[29] = "OFF";
      }
      else {
        SDCard_row[29] = "ON";
      }

      
      if (BMS_equ.max_cell_volt_equ > BMS_equ.min_cell_volt_equ && SDCard_row[2] != "-3000" && SDCard_row[3] != "-40" && BMS_equ.cell_voltage_equ[15] != 0 && BMS_equ.cell_voltage_equ[0] != 0) {
        write_data( Address.c_str(), SDCard_row);
        Serial.print("SD card is written:");
        Serial.println(TimeString);
      }
    }
    else {
      rootSD = SD.open("/");
      delay(100);
      rm(rootSD, rootpath);
      SDReset = false;
      Serial.println("SD Card files deleted!!!!!!!!!!!");
    }

    delay(2000);
  }

}


void ENCONNECT_CODE( void * pvParameters ) {


  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Enconnect Server waiting..."));
    delay(500);
  }

  // The logical name http://fileserver.local will also access the device if you have 'Bonjour' running or your system supports multicast dns
  if (!MDNS.begin("fileserver")) {          // Set your preferred server name, if you use "myserver" the address would be http://myserver.local/
    Serial.println(F("Error setting up MDNS responder!"));
    ESP.restart();
  }




  server.on("/",         HomePage);
  server.on("/dir",       Dir);
  server.on("/download", File_Download);
  ///////////////////////////// End of Request commands
  server.begin();
  Serial.println("HTTP server started");




  /////////////////


  for (;;) {
    // Serial.println(F("Enconnect Server working"));
    server.handleClient(); // Listen for client connections

    delay(500);
  }

}





void BT_CODE( void * pvParameters ) {

  WIFIBT = true;
  String message = "";
  char incomingChar;
  int param_start = 0;
  int param_end = 0;
  String BTProcessor;
  int param_start2 = 0;
  int param_end2 = 0;
  String BTProcessor2;

  String BT_STRING;
  String BT_REMAINING;
  int BT_Dryindex;
  int BT_RTCindex;
  String BT_DRY_ARRAY[4];
  String DryString;
  int BT_RTC_ARRAY[6];





  Serial.println(F("WIFI is Off, BT is On"));
  wifistatus = false;
  String BT_STATION = "ENCAP-BT-" + SerialNumber.substring(SerialNumber.length() - 5);
  Serial.println(BT_STATION);

  BluetoothSerial SerialBT;
  SerialBT.begin(BT_STATION.c_str()); //Bluetooth device name
  Serial.println(F("The device started, now you can pair it with bluetooth!"));

  for (;;) {

    if (SerialBT.available()) {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n') {
        message += String(incomingChar);
      }
      else {
        message = "";
      }


      //SET RTC /////////////////////////////////
      param_start = message.indexOf("SETRTC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_RTCindex = 0;


        for (int i = 0; i < 6; i++) {
          // Serial.println("------------");
          BT_RTCindex = BT_REMAINING.indexOf('/');
          BT_RTC_ARRAY[i] = BT_REMAINING.substring(0, BT_RTCindex).toInt();
          BT_REMAINING = BT_REMAINING.substring(BT_RTCindex + 1);
        }

        SerialBT.println(String(BT_RTC_ARRAY[0]) + "/" + String(BT_RTC_ARRAY[1]) + "/" + String(BT_RTC_ARRAY[2]) + "/" + String(BT_RTC_ARRAY[3]) + "/" + String(BT_RTC_ARRAY[4]) + "/" + String(BT_RTC_ARRAY[5]));
        yil = BT_RTC_ARRAY[0];
        ayUInt = BT_RTC_ARRAY[1];
        gunUInt = BT_RTC_ARRAY[2];
        saatUInt = BT_RTC_ARRAY[3];
        dakikaUInt = BT_RTC_ARRAY[4];
        saniyeUInt = BT_RTC_ARRAY[5];
        RTCCongifured = true;

        message = "";


      }///////////////////////////////////////////////


      //SET EQUALIZER DIFFVOLTAGE////////////////////////////////
      param_start = message.indexOf("SETEQUVOLT");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 10, param_end);

        SerialBT.println(BTProcessor);
        EQUVOLT_VAL = BTProcessor.toFloat();
        CAL_EQUVOLT = true;
        message = "";

      }///////////////////////////////////////////////


      //SET EQUALIZER CURRENT////////////////////////////////
      param_start = message.indexOf("SETEQUCURRENT");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 13, param_end);

        SerialBT.println(BTProcessor);
        EQUCURRENT_VAL = BTProcessor.toFloat();
        CAL_EQUCURRENT = true;
        message = "";

      }///////////////////////////////////////////////


      //SET EQUALIZER SLEEP ////////////////////////////////
      param_start = message.indexOf("SETEQUSLEEP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 11, param_end);
        EQUSLEEP_VAL = BTProcessor.toInt();
        SerialBT.println(EQUSLEEP_VAL);
        CAL_EQUSLEEP = true;
        Serial.println("Sleep command is given TO EQU");
        //Serial.println(CAL_EQUSLEEP);
        message = "";

      }///////////////////////////////////////////////


      //SET SUMHI_SUMLOW EQU////////////////////////////////
      param_start2 = message.indexOf("SETSUMEQUHI");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 11, param_end2);
        EQUSUM_HI = BTProcessor2.toFloat();


      }///////////////////////////////////////////////


      param_start = message.indexOf("SETSUMEQULOW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 12, param_end);
        EQUSUM_LOW = BTProcessor.toFloat();
        message = "";
        SerialBT.print(EQUSUM_HI);
        SerialBT.print("/");
        SerialBT.println(EQUSUM_LOW);
        CAL_EQUSUM = true;

        Serial.println("Equsum command is given");
      }///////////////////////////////////////////////




      //SET HVC_LVC EQU////////////////////////////////
      param_start2 = message.indexOf("SETHVCEQU");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 9, param_end2);
        HVCEQU = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////


      param_start = message.indexOf("SETLVCEQU");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 9, param_end);
        LVCEQU = BTProcessor.toFloat();
        message = "";
        SerialBT.print(HVCEQU);
        SerialBT.print("/");
        SerialBT.println(LVCEQU);
        HVC_LVC_EQU = true;
      }///////////////////////////////////////////////



      //READ EQU CELL ALARM/////////////////////////////////
      param_start = message.indexOf("READEQUCELL");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(BMS_equ.balance_volt_equ) + "/" + String(BMS_equ.balance_volt_diff_equ) + "/" + String(BMS_equ.cell_volthigh2_equ) + "/" + String(BMS_equ.cell_voltlow2_equ) + "#");

        message = "";
      }///////////////////////////////////////////////

      //READ EQU SUM ALARM/////////////////////////////////
      param_start = message.indexOf("READEQUSUM");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(BMS_equ.sumv_high2_equ) + "/" + String(BMS_equ.sumv_low2_equ) + "#");

        message = "";
      }///////////////////////////////////////////////


      //READ EQU SLEEP/////////////////////////////////
      param_start = message.indexOf("READEQUSLEEP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(BMS_equ.secondsleepbm_equ) + "#");

        message = "";
      }///////////////////////////////////////////////




      //bool set_cellvolt_BMS_equ(float cellhigh_v,float celllow_volt,uint8_t *BMS_Set_equ)


      //RESET SD CARD/////////////////////////////////
      param_start = message.indexOf("RESETSD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("SD CARD RESETTED");
        SDReset = true;
        message = "";
      }///////////////////////////////////////////////


      //READ GPIO/////////////////////////////////
      param_start = message.indexOf("READGPIO");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(GPIO_READ[0]) + "/" + String(GPIO_READ[1]) + "/" + String(GPIO_READ[2]));
        message = "";
      }///////////////////////////////////////////////


      //READ DRYA/////////////////////////////////
      param_start = message.indexOf("GETDRYA");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYA_ARRAY[0]) + "/" + String(DRYA_ARRAY[1]) + "/" + String(DRYA_ARRAY[2]) + "/" + String(DRYA_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////


      //READ DRYB/////////////////////////////////
      param_start = message.indexOf("GETDRYB");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYB_ARRAY[0]) + "/" + String(DRYB_ARRAY[1]) + "/" + String(DRYB_ARRAY[2]) + "/" + String(DRYB_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////

      //READ DRYC/////////////////////////////////
      param_start = message.indexOf("GETDRYC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYC_ARRAY[0]) + "/" + String(DRYC_ARRAY[1]) + "/" + String(DRYC_ARRAY[2]) + "/" + String(DRYC_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////


      //READ DRYD/////////////////////////////////
      param_start = message.indexOf("GETDRYD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(String(DRYD_ARRAY[0]) + "/" + String(DRYD_ARRAY[1]) + "/" + String(DRYD_ARRAY[2]) + "/" + String(DRYD_ARRAY[3]));
        message = "";
      }///////////////////////////////////////////////




      //SET DRYA /////////////////////////////////
      param_start = message.indexOf("SETDRYA");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYA", DryString);
        DRYA_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYA_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYA_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYA_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////




      //SET DRYB /////////////////////////////////
      param_start = message.indexOf("SETDRYB");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        String DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYB", DryString);
        DRYB_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYB_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYB_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYB_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////



      //SET DRYC /////////////////////////////////
      param_start = message.indexOf("SETDRYC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        String DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYC", DryString);
        DRYC_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYC_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYC_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYC_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////





      //SET DRYD /////////////////////////////////
      param_start = message.indexOf("SETDRYD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);

        BT_STRING = BTProcessor;
        BT_REMAINING = BTProcessor;
        BT_Dryindex = 0;
        BT_DRY_ARRAY[4];

        for (int i = 0; i < 4; i++) {
          // Serial.println("------------");
          BT_Dryindex = BT_REMAINING.indexOf('/');
          BT_DRY_ARRAY[i] = BT_REMAINING.substring(0, BT_Dryindex);
          BT_REMAINING = BT_REMAINING.substring(BT_Dryindex + 1);
        }
        SerialBT.println(BT_DRY_ARRAY[0] + "/" + BT_DRY_ARRAY[1] + "/" + BT_DRY_ARRAY[2] + "/" + BT_DRY_ARRAY[3]);

        String DryString = String(BT_DRY_ARRAY[0]) + "/" + String(BT_DRY_ARRAY[1]) + "/" + String(BT_DRY_ARRAY[2]) + "/" + String(BT_DRY_ARRAY[3]) + "/" + String(DRY_VALUES[0]);
        Serial.println(DryString);
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("DRYD", DryString);
        DRYD_ARRAY[0] = BT_DRY_ARRAY[0];
        DRYD_ARRAY[1] = BT_DRY_ARRAY[1];
        DRYD_ARRAY[2] = BT_DRY_ARRAY[2];
        DRYD_ARRAY[3] = BT_DRY_ARRAY[3];
        preferences.end();
        EEPROMbusy = false;
        message = "";
      }///////////////////////////////////////////////


      //GET FIXED ENERGY/////////////////////////////////
      param_start = message.indexOf("GETENERGYFIX");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.print(ChargeEnergyFixed);
        SerialBT.print("/");
        SerialBT.print(DischargeEnergyFixed);
        message = "";
      }///////////////////////////////////////////////


      //show barcode/////////////////////////////////
      param_start = message.indexOf("SHOWBARCODE");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("BARCODE IS SHOWN");
        ShowBarcode = true;
        PageNumber = 16;
        message = "";
      }///////////////////////////////////////////////

      //hide barcode/////////////////////////////////
      param_start = message.indexOf("HIDEBARCODE");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("BARCODE IS HIDDEN");
        ShowBarcode = false;
        message = "";
      }///////////////////////////////////////////////

      //PINGING BMS/////////////////////////////////
      param_start = message.indexOf("PING");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println("PONG_BMS");
        message = "";
      }///////////////////////////////////////////////

      //RESET ENERGY////////////////////////////////
      param_start = message.indexOf("RESETENERGY");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putFloat("ChargeEnergy", 0);
        preferences.putFloat("DischargeEnergy", 0);
        SerialBT.println("Energies are reseted");
        preferences.end();
        ChargeEnergy = 0;
        DischargeEnergy = 0;
        EEPROMbusy = false;
      }///////////////////////////////////////////////


      //SET SERIAL BMS////////////////////////////////
      //EN-02-048V0-0-3-3-0-0-23-A-00001
      param_start = message.indexOf("SETSR");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 5, param_end);
        SerialNumber = "EN" + BTProcessor;
        SerialBT.println(SerialNumber);
        message = "";
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putString("SerialNumber", SerialNumber);
        preferences.end();
        EEPROMbusy = false;
      }///////////////////////////////////////////////


      //SET time OFFSET////////////////////////////////

      param_start = message.indexOf("SETOFFSET");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 9, param_end);
        SerialBT.println(BTProcessor);
        message = "";
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putInt("timeoffset", BTProcessor.toInt());
        preferences.end();
        EEPROMbusy = false;
      }///////////////////////////////////////////////


      //SET CAP BMS////////////////////////////////
      param_start = message.indexOf("SETCAP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        SystemCAP = BTProcessor.toInt();
        SerialBT.println(SystemCAP);
        RC_RV = true;
        RV = 0;
        message = "";

      }///////////////////////////////////////////////



      //SET SLEEP BMS////////////////////////////////
      param_start = message.indexOf("SETSLP");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        SLP = BTProcessor.toInt();
        SerialBT.println(SLP);
        SLP_CAL = true;
        //Serial.println("Sleep command is given");
        //Serial.println(SLP_CAL);
        message = "";

      }///////////////////////////////////////////////




      //SET SOC BMS////////////////////////////////
      param_start = message.indexOf("SETSOC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        SystemSOC = BTProcessor.toInt();
        SerialBT.println(SystemSOC);
        SOC_CAL = true;
        message = "";

      }///////////////////////////////////////////////


      //SET CVD BMS////////////////////////////////
      param_start = message.indexOf("SETCVD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        CVD = BTProcessor.toFloat();
        SerialBT.println(CVD);
        CVD_CAL = true;
        message = "";

      }///////////////////////////////////////////////





      //SET BV_BD BMS////////////////////////////////
      param_start2 = message.indexOf("SETBV");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 5, param_end2);
        BV = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETBD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 5, param_end);
        BD = BTProcessor.toFloat();
        message = "";
        SerialBT.print(BV);
        SerialBT.print("/");
        SerialBT.println(BD);
        BV_BD = true;
      }///////////////////////////////////////////////



      //SET SOC_ALARM BMS////////////////////////////////
      param_start2 = message.indexOf("SETSHI");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        SOCHI = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETSLOW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 7, param_end);
        SOCLOW = BTProcessor.toFloat();

        Serial.println("SOCALARM");
        Serial.println(SOCLOW);
        Serial.println(SOCHI);
        message = "";
        SerialBT.print(SOCHI);
        SerialBT.print("/");
        SerialBT.println(SOCLOW);
        SOC_ALARM = true;
      }///////////////////////////////////////////////

      //SET SUM_CALL BMS////////////////////////////////
      param_start2 = message.indexOf("SETSUMHI");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 8, param_end2);
        SUMHI = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETSUMLOW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 9, param_end);
        SUMLOW = BTProcessor.toFloat();
        message = "";
        SerialBT.print(SUMHI);
        SerialBT.print("/");
        SerialBT.println(SUMLOW);
        SUM_CAL = true;
      }///////////////////////////////////////////////


      //SET HCC&HCD BMS////////////////////////////////
      param_start2 = message.indexOf("SETHCC");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        HCC = BTProcessor2.toInt();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////

      param_start = message.indexOf("SETHCD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        HCD = BTProcessor.toInt();
        message = "";
        SerialBT.print(HCC);
        SerialBT.print("/");
        SerialBT.println(HCD);
        HCC_HCD = true;
      }///////////////////////////////////////////////


      //SET HVC_LVC BMS////////////////////////////////
      param_start2 = message.indexOf("SETHVC");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        HVC = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////


      param_start = message.indexOf("SETLVC");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        LVC = BTProcessor.toFloat();
        message = "";
        SerialBT.print(HVC);
        SerialBT.print("/");
        SerialBT.println(LVC);
        HVC_LVC = true;
      }///////////////////////////////////////////////





      ///////////////////////////////////////////
      //SET HTC_HTD BMS////////////////////////////////
      param_start2 = message.indexOf("SETHTC");
      param_end2 = message.indexOf("/");
      if (param_start2 != -1 && param_end2 != -1) {
        BTProcessor2 = message.substring(param_start2 + 6, param_end2);
        HTC = BTProcessor2.toFloat();
        //SerialBT.println(HCC);

      }///////////////////////////////////////////////


      param_start = message.indexOf("SETHTD");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        BTProcessor = message.substring(param_start + 6, param_end);
        HTD = BTProcessor.toFloat();
        message = "";
        SerialBT.print(HTC);
        SerialBT.print("/");
        SerialBT.println(HTD);
        HTC_HTD = true;
      }///////////////////////////////////////////////



      //GET SERIAL BMS////////////////////////////////
      //EN-02-048V0-0-3-3-0-0-23-A-00001
      param_start = message.indexOf("GETSR");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(SerialNumber);
        message = "";
      }///////////////////////////////////////////////

      //GET FIRMWARE BMS////////////////////////////////
      param_start = message.indexOf("GETFW");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        SerialBT.println(FirmwareVer);
        message = "";
      }///////////////////////////////////////////////


      //GET MEAS BMS////////////////////////////////
      param_start = message.indexOf("GET_MEAS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        String MEAS = "TV" + String(BMS.sum_voltage * 0.1) +  "/TC" + String((BMS.current - 30000) * 0.1) + "/SOC" +  String(BMS.SOC * 0.1) + "/TEMP" +  String(BMS.max_cell_temp - 40) +
                      "/MAX" + String(BMS_equ.max_cell_volt_equ * 0.001) + "/MIN" + String(BMS_equ.min_cell_volt_equ * 0.001) +  "/STT" + String(BMS.state) +
                      "/CH" + String(BMS.charge) + "/DSH" + String(BMS.discharge) + "/LF" + String(BMS.bms_life) +
                      "/RC" + String(BMS.rem_cap) + "/BL" + String(balancerState) + "/#";

        SerialBT.println(MEAS);
        message = "";
      }///////////////////////////////////////////////


      //GET CELLS BMS////////////////////////////////
      param_start = message.indexOf("GET_CELLS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        String CellString = "";
        for (int i = 0; i < 16; i++) {
          CellString = CellString + String(BMS_equ.cell_voltage_equ[i] * 0.001) + "/";
        }
        CellString = "CELLS" + CellString + "#";
        SerialBT.println(CellString);
        message = "";
      }///////////////////////////////////////////////


      //GET ALARMS BMS////////////////////////////////
      param_start = message.indexOf("GET_ALARMS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {

        String AlarmsString = "";
        for (int i = 0; i < 28; i++) {
          AlarmsString = AlarmsString + String(BMS.error[i]) + "/";
        }
        AlarmsString = "ALARMS" + AlarmsString + "#";
        SerialBT.println(AlarmsString);
        message = "";
      }///////////////////////////////////////////////




      //GET LIMITS BMS////////////////////////////////
      param_start = message.indexOf("GET_LIMITS");
      param_end = message.indexOf("#");
      if (param_start != -1 && param_end != -1) {
        //ReadAllParameters = true;

        String LIMITS = "DSCHI" + String(BMS.dischar_curr2) + "/" + "CHCHI" + String(BMS.charge_curr2) + "/" + "SUMVHI" + String(BMS.sumv_high2 * 0.1) + "/" +
                        "SUMVLO" + String(BMS.sumv_low2 * 0.1) + "/" + "CVHI" + String(BMS.cell_volthigh2 * 0.001) + "/" + "CVLO" + String(BMS.cell_voltlow2 * 0.001) + "/" +
                        "BV" + String(BMS.balance_volt * 0.001) + "/" + "BDV" + String(BMS.balance_volt_diff) + "/" + "CTHI" + String(BMS.charge_temp_high2) + "/" +
                        "CTLO" + String(BMS.charge_temp_low2) + "/" + "DTHI" + String(BMS.discharge_temp_high2) + "/" + "DTLO" + String(BMS.discharge_temp_low2) + "/" +
                        "VDF" + String(BMS.volt_diff2) + "/";

        LIMITS = "LIMITS" + LIMITS + "#";
        SerialBT.println(LIMITS);
        message = "";
      }///////////////////////////////////////////////

    }

    delay(100);
  }
}


void BMS_COMM_CODE( void * pvParameters ) {
  unsigned long previousMillisBMS = 0;
  unsigned long intervalBMS = 100;
  unsigned long currentMillisBMS = 0;
  String calibratedHour = "";
  String calibratedMinute = "";

  float periodBMS;
  Serial.println("BMS Started");
  //ReadAllParameters = true;
  for (;;) {



    currentMillisBMS = millis();
    if ((currentMillisBMS - previousMillisBMS) >= intervalBMS && (!HCC_HCD) && (!SLP_CAL) && (!HVC_LVC) && (!HVC_LVC_EQU) && (!HTC_HTD) && (!RC_RV)
        && (!BV_BD) && (!SOC_CAL) && (!SUM_CAL) && (!SOC_ALARM) && (!CHARGE_SET) && (!DISCHARGE_SET) && (!BALANCE_SET) && (!CAL_EQUVOLT) &&(!CAL_EQUSUM) && (!CAL_EQUSLEEP) && (!CAL_EQUCURRENT) && (!CVD_CAL)) {
      float period = (currentMillisBMS - previousMillisBMS) * 0.001;
      //RTC


      esp_task_wdt_init(60, false);
      if (RTCCongifured == true) {
        RTCCongifured = false;
        rtc_bms_begin(yil, ayUInt, gunUInt, saatUInt, dakikaUInt, saniyeUInt);
        Serial.println(String(rtc.days) + "/" + String(rtc.months) + "/" + String(rtc.years));
        Serial.println(String(rtc.hours) + "/" + String(rtc.minutes));
      }

      get_time();
      if (String(rtc.hours).length() < 2) {
        calibratedHour = "0" + String(rtc.hours);
      }
      else {
        calibratedHour = String(rtc.hours);
      }

      if (String(rtc.minutes).length() < 2) {
        calibratedMinute = "0" + String(rtc.minutes);
      }
      else {
        calibratedMinute = String(rtc.minutes);
      }
      TimeString = calibratedHour + ":" + calibratedMinute;
      DateString = String(rtc.days) + "/" + String(rtc.months) + "/" + String(rtc.years);










      ////////////////////////////CHECKING DRY CONTACTS
      //DRY_VALUES


      switch (DRYA_ARRAY[0].toInt()) {  //DRY CONTACT A

        case 0:  // Terminal Voltage Rule
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYA_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYA_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYA_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYA_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYA_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYA_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYA_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYA_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYA_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[0] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[0] = 0;
          break;

        case 5:
          DRY_VALUES[0] = 1;
          break;

        default:
          break;
      }

      ////////////////////


      switch (DRYB_ARRAY[0].toInt()) {  //DRY CONTACT B

        case 0:  // Terminal Voltage Rule
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYB_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYB_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYB_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYB_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYB_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYB_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYB_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[0] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYB_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYB_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYB_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[1] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[1] = 0;
          break;

        case 5:
          DRY_VALUES[1] = 1;
          break;

        default:
          break;
      }

      ////////////////////

      switch (DRYC_ARRAY[0].toInt()) {  //DRY CONTACT C

        case 0:  // Terminal Voltage Rule
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYC_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYC_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[1] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYC_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYC_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYC_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYC_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYC_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYC_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYC_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[2] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYC_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[2] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[2] = 0;
          break;

        case 5:
          DRY_VALUES[2] = 1;
          break;

        default:
          break;
      }

      ////////////////////

      ////////////////////


      switch (DRYD_ARRAY[0].toInt()) {  //DRY CONTACT D

        case 0:  // Terminal Voltage Rule
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (BMS.sum_voltage * 0.1 <= DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A  voltageis enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.sum_voltage * 0.1 > DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A  voltageis disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.sum_voltage * 0.1 > DRYD_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A voltage is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.sum_voltage * 0.1 <= DRYD_ARRAY[3].toFloat()) {
              //Serial.println("Dry contact A voltage is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        case 1:  // Current
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (abs((BMS.current - 30000 ) * 0.1) <= DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) > DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (abs((BMS.current - 30000) * 0.1) > DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact max volt A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (abs((BMS.current - 30000) * 0.1) <= DRYD_ARRAY[3].toFloat()) {
              // Serial.println("Dry contact max volt A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;


        case 2:  // temperature
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (BMS.max_cell_temp - 40 <= DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.max_cell_temp - 40 > DRYD_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.max_cell_temp - 40 > DRYD_ARRAY[2].toFloat()) {
              //   Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.max_cell_temp - 40 <= DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        case 3:  // soc
          if (DRYD_ARRAY[1].toInt() == 0) { //<=

            if (BMS.SOC * 0.1 <= DRYD_ARRAY[2].toFloat()) {
              //  Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.SOC * 0.1 > DRYD_ARRAY[3].toFloat()) {
              //  Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          else {//>
            if (BMS.SOC * 0.1 > DRYD_ARRAY[2].toFloat()) {
              // Serial.println("Dry contact A is enabled ");
              DRY_VALUES[3] = 1;
            }
            else if (BMS.SOC * 0.1 <= DRYD_ARRAY[3].toFloat()) {
              //   Serial.println("Dry contact A is disabled ");
              DRY_VALUES[3] = 0;
            }
          }
          break;

        case 4:
          DRY_VALUES[3] = 0;
          break;

        case 5:
          DRY_VALUES[3] = 1;
          break;

        default:
          break;
      }

      ////////////////////














      for (int i = 0; i < 4; i++) {

        if (DRY_VALUES[i]) {
          gpio_out_on(8 - i); // for OLD BOARD
          gpio_out_on(4 - i); // for new board
          //Serial.println("gpio: " + String (i) + " enabled");
        }
        else {
          gpio_out_off(8 - i); //OLD BOARD
          gpio_out_off(4 - i); // for new board
          //Serial.println("gpio: " + String (i) + " disabled");
        }
      }

      //DRY INPUTS//////////////////
      //      Serial.println("READ PIN1");
      //      Serial.println(gpio_read(1));
      //      Serial.println("READ PIN2");
      //      Serial.println( gpio_read(2));
      //      Serial.println("READ PIN3");
      //      Serial.println(gpio_read(3));


      GPIO_READ[0] = gpio_read(1);
      GPIO_READ[1] = gpio_read(2);
      GPIO_READ[2] = gpio_read(3);
      ///////////////////////////////////////////////////




      //Serial.println("BMS PARAMETER STARTED");
      //EQUALIZER
      EQUON = (BMS.current - 30000) <= 0;

      // Serial.println("BMS started:" + String(millis()));
      getBatteryParameters();
      // Serial.println("BMS ended:" + String(millis()));

      EQUOFF = (BMS.current - 30000) > 0;

      //GET BALANCER STATE
      for (uint8_t i = 0; i < 16; i++) {
        balancerState = BMS.cell_balance[i] || balancerState;
        // Serial.println(BMS.cell_balance[i]);
      }
      //Serial.println("BMS PARAMETER ENDED");

      //GET ALARMS

      //Serial.println("ALARMS STARTED");
      alarmNo = 0;
      for (int i = 0; i < 28; i++) {
        if (BMS.error[i] == true) {
          alarmNo++;
        }
      }

      //Serial.println("ALARMS ENDED");

      //EQUAL[IZER2



      //Serial.println(" EQU STARTED");
      if (EQUON > EQUON2 && EqualizerMasterInput) {
        //Serial.println(F("checking"));
        for (int i = 0; i < 5; i++) {
          //Serial.println(F("trying"));
          if (balance_turn_on()) {
            EQUON2 = true;
            EQUOFF2 = false;
            //Serial.println(F("Equalizer is on"));
            EqulizerStatus = true;
            for (int i = 0; i < 20; i++) {
              send_balance_cur(8, BMS_Equ);
            }
            break;

          }
        }

      }

      // Serial.println(" EQU ENDED");

      //Serial.println(" EQU STARTED");

      if (EQUOFF > EQUOFF2) {
        for (int i = 0; i < 20; i++) {
          if (balance_turn_off()) {
            Serial.println(F("Equalizer is off"));
            EqulizerStatus = false;
            EQUON2 = false;
            EQUOFF2 = true;
            break;
          }
        }
      }



      //Energy Calculations..........

      if (!EEPROMbusy && period != 0 ) {

        if ((BMS.current - 30000) * 0.1 > 1  && BMS.sum_voltage * 0.1 > 8 ) {
          ChargeEnergy = ChargeEnergy + ((BMS.sum_voltage * 0.1 * (BMS.current - 30000) * 0.1) / (3600 / period) * 0.001);
          ChargeEnergyFixed = ChargeEnergyFixed + ((BMS.sum_voltage * 0.1 * (BMS.current - 30000) * 0.1) / (3600 / period) * 0.001);
          //          Serial.println("Charging...");
          //          Serial.println("Energy per sec");
          //          Serial.print((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001, 4);
          //          Serial.println("kW");
        }



        if ((BMS.current - 30000) * 0.1 < -1 && BMS.sum_voltage * 0.1 > 8) {
          DischargeEnergy = DischargeEnergy + ((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001);
          DischargeEnergyFixed = DischargeEnergyFixed + ((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001);
          //          Serial.println("Discharging...");
          //          Serial.println("Energy per sec");
          //          Serial.print((BMS.sum_voltage * 0.1 * abs((BMS.current - 30000) * 0.1)) / (3600 / period) * 0.001);
          //          Serial.println("kW");

        }

        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putFloat("ChargeEnergy", ChargeEnergy);
        preferences.putFloat("DischargeEnergy", DischargeEnergy);
        preferences.putFloat("ChargeEnergyFixed", ChargeEnergyFixed);
        preferences.putFloat("DischargeEnergyFixed", DischargeEnergyFixed);
        preferences.end();
        EEPROMbusy = false;
      }
      previousMillisBMS = currentMillisBMS;
      esp_task_wdt_init(60, true);
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
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CHARGE_SET = false;
        //ReadAllParameters = true;
      }


      if (CVD_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_volttemp_diff_alarm(CVD, CVD, 0, 0);

          notificationStatus = CommandStatus;
          if (CommandStatus) {
            Serial.println("Command Status:" + String(CommandStatus));
            CommandStatus = false;
            CVD_CAL = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CVD_CAL = false;
        //ReadAllParameters = true;
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
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        DISCHARGE_SET = false;
        //ReadAllParameters = true;
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
            // Serial.println(F("Error"));
          }
        }
        //        Serial.println("param_save:");
        //        Serial.println(param_save);
        notificationEnable = true;
        HCC_HCD = false;
        //ReadAllParameters = true;
      }

      //SOC
      else if (SOC_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_soc_BMS(SystemSOC, BMS_Set);

          Serial.println("SOC is calibrated");
          Serial.println(SystemSOC);
          notificationStatus = CommandStatus;
          if (CommandStatus) {
            CommandStatus = false;
            SOC_CAL = false;
            break;
          }
          else {
            //Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SOC_CAL = false;
        //ReadAllParameters = true;
      }

      //
      //SUM_CAL
      else if (SUM_CAL) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_sumvolt_alarm(SUMHI - 1 , SUMHI, SUMLOW + 1, SUMLOW);

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            SUM_CAL = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SUM_CAL = false;
        //ReadAllParameters = true;
      }


      else if (SOC_ALARM) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_soc_alarm(SOCHI , SOCHI, SOCLOW, SOCLOW);
          Serial.println(SOCHI);
          Serial.println(SOCLOW);

          notificationStatus = CommandStatus;
          Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            SOC_ALARM = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SOC_ALARM = false;
        //ReadAllParameters = true;
      }


      //HVC
      else if (HVC_LVC) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_cellvolt_BMS(HVC, LVC, BMS_Set) ;
          Serial.println(HVC);
          Serial.println(LVC);
          notificationStatus = CommandStatus;
          //Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            HVC_LVC = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        HVC_LVC = false;
        //ReadAllParameters = true;
      }




      else if (HVC_LVC_EQU) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_cellvolt_BMS_equ(HVCEQU, LVCEQU, BMS_Set_equ) ;
          Serial.println(HVCEQU);
          Serial.println(LVCEQU);
          notificationStatus = CommandStatus;
          //Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            HVC_LVC_EQU = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        HVC_LVC_EQU = false;
        //ReadAllParameters = true;
      }




      //HTC_HTD
      else if (HTC_HTD) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_temp_alarm(HTC , HTC, -20, -20, HTD , HTD, -20, -20);
          Serial.println(HTC);
          Serial.println(HTD);
          notificationStatus = CommandStatus;
          //Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            HTC_HTD = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        HTC_HTD = false;
        //ReadAllParameters = true;
      }


      //SLP
      else if (SLP_CAL) {

        Serial.print("SLP_CAL:");
        Serial.println(SLP_CAL);
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          CommandStatus = set_manufect_BMS(0 , board1, SLP, BMS_Set, 2);
          Serial.println("------SLeep---");
          Serial.println(SLP);
          Serial.println("------SLeep---");

          notificationStatus = CommandStatus;
          //Serial.println("Command Status:" + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            SLP_CAL = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        SLP_CAL = false;
        //ReadAllParameters = true;
      }


      // EQU SLP
      else if (CAL_EQUSLEEP) {

        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          CommandStatus = set_manufect_BMS_equ(0 , board1, EQUSLEEP_VAL, BMS_Set_equ);


          Serial.println("------SLeep-- -");
          Serial.println(EQUSLEEP_VAL);
          Serial.println("------SLeep-- -");
          notificationStatus = CommandStatus;
          Serial.println("Command Status: " + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            CAL_EQUSLEEP = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CAL_EQUSLEEP = false;
        //ReadAllParameters = true;
      }


            //EQU SUM HI LOW
            else if (CAL_EQUSUM) {
              CommandStatus = false;
              for (int i = 0; i < 5; i++) {
      
                Serial.println("EQU SUM HI LOW IS WORKING");
      
                CommandStatus = set_sumvolt_alarm_equ(EQUSUM_HI, EQUSUM_HI, EQUSUM_LOW, EQUSUM_LOW );
                Serial.println(CAL_EQUSUM);
                notificationStatus = CommandStatus;
                Serial.println("Command Status: " + String(CommandStatus));
                if (CommandStatus) {
                  CommandStatus = false;
                  CAL_EQUSUM = false;
                  break;
                }
                else {
                  // Serial.println(F("Error"));
                }
              }
              notificationEnable = true;
              CAL_EQUSUM = false;
              //ReadAllParameters = true;
            }







      //EQU VOLT VALUE
      else if (CAL_EQUVOLT) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          CommandStatus = set_balancevolt_BMS_equ( 3.2,  EQUVOLT_VAL, BMS_Set_equ);
          Serial.println(EQUVOLT_VAL);
          notificationStatus = CommandStatus;
          Serial.println("Command Status: " + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            CAL_EQUVOLT = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CAL_EQUVOLT = false;
        //ReadAllParameters = true;
      }

      //EQU CURRENT VALUE
      else if (CAL_EQUCURRENT) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          CommandStatus = send_balance_cur(EQUCURRENT_VAL * 10, BMS_Equ);
          Serial.println(EQUCURRENT_VAL);
          notificationStatus = CommandStatus;
          Serial.println("Command Status: " + String(CommandStatus));
          if (CommandStatus) {
            CommandStatus = false;
            CAL_EQUCURRENT = false;
            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        CAL_EQUCURRENT = false;
        //ReadAllParameters = true;
      }

      //

      else if (RC_RV) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {

          if (RV != 0) {
            CommandStatus = set_rated_BMS(RC, RV, BMS_Set);
          }
          else {

            Serial.println("Capacity is setted");
            Serial.println(SystemCAP);
            CommandStatus = set_rated_BMS(SystemCAP, 3.20, BMS_Set);
          }

          notificationStatus = CommandStatus;
          // Serial.println(F("Command Status:" + String(CommandStatus)));
          if (CommandStatus) {
            CommandStatus = false;
            RC_RV = false;
            break;
          }
          else {
            //  Serial.println(F("Error"));
          }

        }
        notificationEnable = true;
        RC_RV = false;
        //ReadAllParameters = true;
      }

      //


      //BV-BD
      else if (BV_BD) {
        CommandStatus = false;
        for (int i = 0; i < 5; i++) {
          CommandStatus = set_balancevolt_BMS(BV, BD * 0.001, BMS_Set);

          notificationStatus = CommandStatus;
          // Serial.println(F("Command Status:" + String(CommandStatus)));
          if (CommandStatus) {
            CommandStatus = false;
            BV_BD = false;

            break;
          }
          else {
            // Serial.println(F("Error"));
          }
        }
        notificationEnable = true;
        BV_BD = false;
        //ReadAllParameters = true;
      }

      //
    }
    delay(50);
  }
}


void UDPTEST_CODE( void * pvParameters ) {


  AsyncUDP udpcomm;
  WiFiServer serverWifi(81);
  WiFiManager wifiManager;

  /////////////////FORGET
  //  wifiManager.resetSettings();
  //  Serial.println("forget password");
  //  delay(1000);

  esp_task_wdt_init(150, false);
  String isim = "ENCAP-WIFI-" + SerialNumber.substring(SerialNumber.length() - 5);
  //Wifi interface____________________________________________________________________________________________________________
  wifiManager.setCustomHeadElement("<style> h2 {padding: 30px;text-align: center;background: #0099ff;color: white;font-size: 30px; }footer {position: fixed;left: 0; bottom: 0;width: 100%;background-color: #0099ff;color: white;text-align: center;</style><h2>ENCONNECT+ Interface</h2><footer><p>Amber&Waseem Software Development and Design</p><p></p></footer>");
  wifiManager.setTimeout(120);
  if (!wifiManager.autoConnect((const char*)isim.c_str())) {
    Serial.println(F("failed to connect and hit timeout"));
    delay(250);
    Serial.println("re-connecting.....");
    Serial.println("resetting");

    //WiFi.disconnect(true);
    // WiFi.mode(WIFI_OFF);
    delay(100);
    // WIFIBT = true;
    //  Serial.println(F("BT Terminal is enabled"));
    wifistatus = false;
  }

  Serial.println(F("Connection is established"));
  //Serial.println(F("BT Terminal is disabled"));
  wifiManager.autoConnect((const char*)isim.c_str());
  //  IPAddress subnet(255, 255, 255, 0);
  //
  //  IPAddress localGateway(192, 168, 60, 1); //hardcoded
  Serial.println((WiFi.status()));
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), 2001);
  delay(250);

  unsigned long previousMillis = 0;
  unsigned long interval = 5000;
  unsigned long currentMillis = 0;
  SSIDName = WiFi.SSID();
  IP = WiFi.localIP().toString().c_str();
  Port = 2001;
  MAC = WiFi.macAddress();
  wifistatus = true;






  for (;;) {

    currentMillis = millis();
    // if WiFi is down, try reconnecting
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
      Serial.println(F("Reconnecting to WiFi..."));
      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis = currentMillis;
    }
    else {


      //ASYNC UDP
      if (udpcomm.listen(2001)) {
        udpcomm.onPacket([](AsyncUDPPacket packet) {
          String UDPRequest = String( (char*) packet.data());

          Serial.println(UDPRequest);

          //PINGING BMS/////////////////////////////////
          int PING_INDEX_START = UDPRequest.indexOf("PING");
          int PING_INDEX_END = UDPRequest.indexOf("#");
          if (PING_INDEX_START != -1 && PING_INDEX_END != -1) {
            packet.printf("PONG_BMS", packet.length());
          }///////////////////////////////////////////////

          //SET SERIAL BMS////////////////////////////////
          //EN-02-048V0-0-3-3-0-0-23-A-00001
          int SERIAL_INDEX_START = UDPRequest.indexOf("SETSR");
          int SERIAL_INDEX_END = UDPRequest.indexOf("#");
          if (SERIAL_INDEX_START != -1 && SERIAL_INDEX_END != -1) {
            UDPprocessor = UDPRequest.substring(SERIAL_INDEX_START + 5, SERIAL_INDEX_END);
            SerialNumber = UDPprocessor;
            packet.printf(SerialNumber.c_str(), packet.length());
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putString("SerialNumber", SerialNumber);
            preferences.end();
            EEPROMbusy = false;
          }///////////////////////////////////////////////



          //RESET ENERGY BMS////////////////////////////////
          int RESETENERGY_START = UDPRequest.indexOf("RESETENERGY");
          int RESETENERGY_END = UDPRequest.indexOf("#");
          if (RESETENERGY_START != -1 && RESETENERGY_END != -1) {

            packet.printf("Energy is resetted");
            EEPROMbusy = true;
            preferences.begin("my-app", false);
            preferences.putFloat("ChargeEnergy", 0);
            preferences.putFloat("DischargeEnergy", 0);
            preferences.end();
            EEPROMbusy = false;
          }///////////////////////////////////////////////



          //GET SERIAL BMS////////////////////////////////
          //EN-02-048V0-0-3-3-0-0-23-A-00001
          int GSERIAL_INDEX_START = UDPRequest.indexOf("GETSR");
          int GSERIAL_INDEX_END = UDPRequest.indexOf("#");
          if (GSERIAL_INDEX_START != -1 && GSERIAL_INDEX_END != -1) {
            packet.printf(SerialNumber.c_str(), packet.length());
          }///////////////////////////////////////////////



          //GET RESTART COUNTER BMS////////////////////////////////
          int GETRC_START = UDPRequest.indexOf("GETRC");
          int GETRC_END = UDPRequest.indexOf("#");
          if (GETRC_START != -1 && GETRC_END != -1) {
            packet.printf(String(restartCounter).c_str(), packet.length()) ;
          }///////////////////////////////////////////////





          //GET FIRMWARE BMS////////////////////////////////
          int GFIRM_INDEX_START = UDPRequest.indexOf("GETFW");
          int GFIRM_INDEX_END = UDPRequest.indexOf("#");
          if (GFIRM_INDEX_START != -1 && GFIRM_INDEX_END != -1) {
            packet.printf(FirmwareVer.c_str(), packet.length());
          }///////////////////////////////////////////////

          //GET MEAS BMS////////////////////////////////
          int GMEAS_INDEX_START = UDPRequest.indexOf("GET_MEAS");
          int GMEAS_INDEX_END = UDPRequest.indexOf("#");
          if (GMEAS_INDEX_START != -1 && GMEAS_INDEX_END != -1) {

            String MEAS = "TV" + String(BMS.sum_voltage * 0.1) +  "/TC" + String((BMS.current - 30000) * 0.1) + "/SOC" +  String(BMS.SOC * 0.1) + "/TEMP" +  String(BMS.max_cell_temp - 40) +
                          "/MAX" + String(BMS_equ.max_cell_volt_equ * 0.001) + "/MIN" + String(BMS_equ.min_cell_volt_equ * 0.001) +  "/STT" + String(BMS.state) +
                          "/CH" + String(BMS.charge) + "/DSH" + String(BMS.discharge) + "/LF" + String(BMS.bms_life) +
                          "/RC" + String(BMS.rem_cap) + "/BL" + String(balancerState) + "/CHE" + String(ChargeEnergy, 3) + "/DHE" + String(DischargeEnergy) + "/#";

            packet.printf(MEAS.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET CELLS BMS////////////////////////////////
          int GCELLS_INDEX_START = UDPRequest.indexOf("GET_CELLS");
          int GCELLS_INDEX_END = UDPRequest.indexOf("#");
          if (GCELLS_INDEX_START != -1 && GCELLS_INDEX_END != -1) {

            String CellString = "";
            for (int i = 0; i < 16; i++) {
              CellString = CellString + String(BMS_equ.cell_voltage_equ[i] * 0.001) + "/";
            }
            CellString = "CELLS" + CellString + "#";
            packet.printf(CellString.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET ALARMS BMS////////////////////////////////
          int GALARMS_INDEX_START = UDPRequest.indexOf("GET_ALARMS");
          int GALARMS_INDEX_END = UDPRequest.indexOf("#");
          if (GALARMS_INDEX_START != -1 && GALARMS_INDEX_END != -1) {

            String AlarmsString = "";
            for (int i = 0; i < 28; i++) {
              AlarmsString = AlarmsString + String(BMS.error[i]) + "/";
            }
            AlarmsString = "ALARMS" + AlarmsString + "#";
            packet.printf(AlarmsString.c_str(), packet.length());
          }///////////////////////////////////////////////


          //GET LIMITS BMS////////////////////////////////
          int GLIMITS_INDEX_START = UDPRequest.indexOf("GET_LIMITS");
          int GLIMITS_INDEX_END = UDPRequest.indexOf("#");
          if (GLIMITS_INDEX_START != -1 && GLIMITS_INDEX_END != -1) {
            //ReadAllParameters = true;

            String LIMITS = "DSCHI" + String(BMS.dischar_curr2) + "/" + "CHCHI" + String(BMS.charge_curr2) + "/" + "SUMVHI" + String(BMS.sumv_high2 * 0.1) + "/" +
                            "SUMVLO" + String(BMS.sumv_low2 * 0.1) + "/" + "CVHI" + String(BMS.cell_volthigh2 * 0.001) + "/" + "CVLO" + String(BMS.cell_voltlow2 * 0.001) + "/" +
                            "BV" + String(BMS.balance_volt * 0.001) + "/" + "BDV" + String(BMS.balance_volt_diff) + "/" + "CTHI" + String(BMS.charge_temp_high2) + "/" +
                            "CTLO" + String(BMS.charge_temp_low2) + "/" + "DTHI" + String(BMS.discharge_temp_high2) + "/" + "DTLO" + String(BMS.discharge_temp_low2) + "/" +
                            "VDF" + String(BMS.volt_diff2) + "/";

            LIMITS = "LIMITS" + LIMITS + "#";
            packet.printf(LIMITS.c_str(), packet.length());
          }///////////////////////////////////////////////






        });
      }
      delay(100);
    }
  }
}




void FIREBASE_CODE( void * pvParameters ) {

  Serial.println("Firebase task started");
  uint8_t FirebaseCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    // Serial.println("Firebase task is waiting for internet connection...");
    delay(500);
  }
  esp_task_wdt_init(60, false);



  //CHECK FOR UPDATE
  for (int i = 0; i < 3; i++) {
    //Serial.println("UNR Before Update:" + String(UpdateNextRestart));
    if (FirmwareVersionCheck()) {
      if (UpdateNextRestart) {
        delay(500);
        UpdateNextRestart = false;
        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putBool("UNR", false);
        preferences.end();
        EEPROMbusy = false;
        Serial.println(F("Firmware would update"));
        //Serial.println("UNR After Update:" + String(UpdateNextRestart));
        delay(100);
        firmwareUpdate();
      }
      else {
        Serial.println(F("Update Available"));
        UpdateAvailable = true;
        updateIcon = true;
      }
      break;
    }
  }




  String jsonBuffer;
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  // config.token_status_callback = tokenStatusCallback;
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

  // if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0) && WiFi.status() == WL_CONNECTED && IPReady && CoordinatesReady)


  for (;;) {

    esp_task_wdt_init(60, false);
    if (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)
    {
      if (RTCFetched == false) {
        //CHECK FOR RTC


        gmtOffset_sec = timeoffset;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

        if (printLocalTime()) {


          saat = timeHourS.toInt();
          dakika = timeMinS.toInt();
          saniye = timeSecS.toInt();
          gun = timeDayS.toInt();
          ay = timeMonthS.toInt();
          yil = timeYearS.toInt();


          saatUInt = (uint8_t)saat;
          dakikaUInt = (uint8_t)dakika;
          saniyeUInt = (uint8_t)saniye;
          gunUInt = (uint8_t)gun;
          ayUInt = (uint8_t)ay;
          RTCCongifured = true;
          RTCFetched = true;



          //          Serial.print("Time Configuration is done:");
          //          Serial.println(timeHourS + ":" + timeMinS + ":" + timeSecS);
          //          Serial.print(saatUInt);
          //          Serial.print(dakikaUInt);
          //          Serial.print(saniyeUInt);
        }

        else {
          Serial.println("Time Configuration is failed");
        }
      }


      sendDataPrevMillis = millis();
      Serial.println(F("Firebase task is running..."));
      if (WiFi.status() == WL_CONNECTED) {
        wifistatus = true;
        // Serial.println(F("1) Internet Connection:...Checked!"));
        if (Firebase.ready()) {
          //  Serial.println(F("2) Firebase Connection:...Checked!"));
          wifistatus = true;



          String FirebaseTimeStamp = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/TimeStamp";
          String FirebaseCoordinates = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/Coordinates";
          String FirebaseFirmwareVersion = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/Firmware";
          String FirebaseTerminalVoltage = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/TerminalVoltage";
          String FirebaseTerminalCurrent = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/TerminalCurrent";
          String FirebaseCellTemp = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/CellTemp";
          String FirebaseStateOfCharge = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/StateOfCharge";
          String FirebaseMaxCell = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/MaxCell";
          String FirebaseMinCell = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/MinCell";
          String FirebaseChargeEnergy = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/ChargeEnergy";
          String FirebaseDischargeEnergy = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/DischargeEnergy";
          String FirebaseRestartCounter = "/Monitoring/Version1/" + SerialNumber + "/" + timeYearS + "/" + timeMonthS + "/" + timeDayS + "/" + String(rtc.hours) + "/RestartCounter";


          switch (FirebaseCounter) {
            case 0:
              FirebaseCounter++;
              Serial.printf("Time Stamp... %s\n", Firebase.setString(fbdo, F(FirebaseTimeStamp.c_str()), TimeString) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 1:
              FirebaseCounter++;
              //Serial.printf("DateStamp... %s\n", Firebase.setString(fbdo, F(FirebaseDateStamp.c_str()), DateString) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 2:
              FirebaseCounter++;
              //Serial.printf("Coordinates... %s\n", Firebase.setString(fbdo, F(FirebaseCoordinates.c_str()), Coordinates) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 3:
              FirebaseCounter++;
              Serial.printf("FirmwareVersion... %s\n", Firebase.setString(fbdo, F(FirebaseFirmwareVersion.c_str()), FirmwareVer) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 4:
              FirebaseCounter++;
              Serial.printf("TerminalVoltage... %s\n", Firebase.setString(fbdo, F(FirebaseTerminalVoltage.c_str()), String(BMS.sum_voltage * 0.1)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 5:
              FirebaseCounter++;
              Serial.printf("TerminalCurrent... %s\n", Firebase.setString(fbdo, F(FirebaseTerminalCurrent.c_str()), String((BMS.current - 30000) * 0.1)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 6:
              FirebaseCounter++;
              Serial.printf("CellTemp... %s\n", Firebase.setString(fbdo, F(FirebaseCellTemp.c_str()), String(BMS.max_cell_temp - 40)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 7:
              FirebaseCounter++;
              Serial.printf("StateOfCharge... %s\n", Firebase.setString(fbdo, F(FirebaseStateOfCharge.c_str()), String(BMS.SOC * 0.1)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 8:
              FirebaseCounter++;
              Serial.printf("Set int... %s\n", Firebase.setString(fbdo, F(FirebaseMaxCell.c_str()), String(BMS_equ.max_cell_volt_equ * 0.001)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 9:
              FirebaseCounter++;
              Serial.printf("MaxCell... %s\n", Firebase.setString(fbdo, F(FirebaseMaxCell.c_str()), String(BMS_equ.max_cell_volt_equ * 0.001)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 10:
              FirebaseCounter++;
              Serial.printf("MinCell... %s\n", Firebase.setString(fbdo, F(FirebaseMinCell.c_str()), String(BMS_equ.min_cell_volt_equ * 0.001)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 11:
              FirebaseCounter++;
              Serial.printf("ChargeEnergy... %s\n", Firebase.setString(fbdo, F(FirebaseChargeEnergy.c_str()), String(ChargeEnergy)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 12:
              FirebaseCounter++;
              Serial.printf("DischargeEnergy... %s\n", Firebase.setString(fbdo, F(FirebaseDischargeEnergy.c_str()), String(DischargeEnergy)) ? "ok" : fbdo.errorReason().c_str());
              break;
            case 13:
              FirebaseCounter++;
              Serial.printf("RestartCounter... %s\n", Firebase.setString(fbdo, F(FirebaseRestartCounter.c_str()), String(restartCounter)) ? "ok" : fbdo.errorReason().c_str());
              break;

            case 14:
              FirebaseCounter++;
              printLocalTime();
              break;

            default:
              FirebaseCounter = 0;
              break;
          }
        }


        else {
          Serial.println("No Firebase Connection");
          firebaseReady = false;
        }
      }
      else {
        wifistatus = false;
        Serial.println("No Internet Connection");
      }

    }
    delay(100);
    esp_task_wdt_init(60, true);
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

  while (!ts.begin(18, 19, 40)) {
    Serial.println(F("Touch Screen trying to start...."));
    delay(500);
  }

  Serial.println(F("Touch Screen started.   !!!!!!!!!!!!!!!!!!!!!"));



  for (uint8_t i = 0; i < 100; i++) {
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




      //mute &unmute
      if (y >= 70 && y <= 92 && x >= 80 && x <= 110 && !action_done) {
        mute = !mute;
        Serial.println("mute toggled:");
        Serial.println(mute);

        EEPROMbusy = true;
        preferences.begin("my-app", false);
        preferences.putBool("mute", mute);
        preferences.end();
        EEPROMbusy = false;


        action_done = true;
      }


      ///SWIPE RIGHT
      if (TouchArray[touch_iteration] - TouchArray[0] > 50 && !action_done && TouchArray[touch_iteration] != 0 && touch_iteration > 10) {
        Serial.println("Swipe Right");
        if (PageNumber == 0) {
          PageNumber = 16;
        }

        action_done = true;

      }
      ///SWIPE LEFT
      if (TouchArray[0] - TouchArray[touch_iteration] > 50 && !action_done  && TouchArray[touch_iteration] != 0 && touch_iteration > 10) {
        Serial.println("Swipe Left");
        if (PageNumber == 16) {
          PageNumber = 0;
        }

        action_done = true;
        //        buzzer_on();
        //        Serial.println("Buzzing");
        //        delay(100);
        //        buzzer_off();
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
        //        buzzer_on();
        //        Serial.println("Buzzing");
        //        delay(100);
        //        buzzer_off();
        action_done = true;
        numpadEnable = false;
        notificationEnable = false;
      }


      ///Update Firmware
      if (y >= 66 && y <= 82 && x >= 385 && x <= 400 && !action_done && updateReady ) {
        Serial.println("Firmware update activated");
        action_done = true;
      }

      if (PageNumber == 16) {

        //Dry Contacts
        if (!action_done && y >= 95 && y <= 210 && x >= 0 && x <= 120 ) {
          // Serial.println("Dry Contacts Page is written");
          PageNumber = 17;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
          buzzerActive = true;
        }

      }

      if (PageNumber == 17 || PageNumber == 13) {
        //DRY NAME

        //contactname is toggled

        if (!action_done && y >= 115 && y <= 205 && x >= 30 && x <= 190 && !numpadEnable ) {
          ContactNameActive = !ContactNameActive;
          if (ContactNameActive) {
            ContactTypeActive = false;
            OperationActive = false;
            DCEnable = false;
            DCDisable = false;
          }

          Serial.print("ContactNameActive:");
          Serial.println(ContactNameActive);
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
        }

        if (!action_done && y >= 115 && y <= 205 && x >= 200 && x <= 360 && !numpadEnable) {
          ContactTypeActive = !ContactTypeActive;

          //contacttype is toggled

          if (ContactTypeActive) {
            ContactNameActive = false;
            OperationActive = false;
          }

          Serial.print("ContactTypeActive:");
          Serial.println(ContactTypeActive);
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
        }


        if (!action_done && y >= 115 && y <= 155 && x >= 380 && x <= 460 && !numpadEnable) {
          OperationActive = !OperationActive;
          Serial.print("OperationActive:");
          Serial.println(OperationActive);
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;

          if (OperationActive) {
            ContactNameActive = false;
            ContactTypeActive = false;
            DCEnable = false;
            DCDisable = false;
          }
        }


        if (!action_done && y >= 215 && y <= 305 && x >= 30 && x <= 190 && !numpadEnable ) {
          numpadEnable = true;
          DCEnable = true;
          numpadValue = "";
          action_done = true;
          Serial.print("DryContactValueEnable");


          if (DCEnable) {
            ContactNameActive = false;
            ContactTypeActive = false;
            ContactDisableActive = false;
            DCDisable = false;
          }
        }

        if (!action_done && y >= 215 && y <= 305 && x >= 200 && x <= 360 && !numpadEnable ) {
          numpadEnable = true;
          DCDisable = true;
          numpadValue = "";
          action_done = true;
          Serial.print("DryContactValueDisable");


          if (DCDisable) {
            ContactNameActive = false;
            ContactTypeActive = false;
            ContactDisableActive = false;
            DCEnable = false;
          }
        }




        if (!action_done && y >= 245 && y <= 300 && x >= 400 && x <= 448 && !numpadEnable ) {

          notificationEnable = true;
          numpadValue = "";
          action_done = true;
          Serial.print("SAVEDRY");
          ContactNameActive = false;
          ContactTypeActive = false;
          ContactDisableActive = false;
          DCEnable = false;
          DCDisable = false;


          Serial.println("---DRY CONTACTS SAVED---");
          Serial.println(ContactNameList[ContactNameIndex]);
          Serial.println(ContactFunctionList[ContactTypeIndex]);
          Serial.println(OperationList[OperationTypeIndex]);
          Serial.println(DryContactValueEnable);
          Serial.println(DryContactValueDisable);
          Serial.println("-------------");


          //SET DRYA ////

          String DryString = String(ContactTypeIndex) + "/" + String(OperationTypeIndex) + "/" + String(DryContactValueEnable) + "/" + String(DryContactValueDisable) + "/" + String(DRY_VALUES[ContactNameIndex]);
          Serial.println(DryString);
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          switch (ContactNameIndex) {
            case 0:
              preferences.putString("DRYA", DryString);
              DRYA_ARRAY[0] = String(ContactTypeIndex);
              DRYA_ARRAY[1] = String(OperationTypeIndex);
              DRYA_ARRAY[2] = String(DryContactValueEnable);
              DRYA_ARRAY[3] = String(DryContactValueDisable);


              break;

            case 1:
              preferences.putString("DRYB", DryString);
              DRYB_ARRAY[0] = String(ContactTypeIndex);
              DRYB_ARRAY[1] = String(OperationTypeIndex);
              DRYB_ARRAY[2] = String(DryContactValueEnable);
              DRYB_ARRAY[3] = String(DryContactValueDisable);

              break;

            case 2:
              preferences.putString("DRYC", DryString);
              DRYC_ARRAY[0] = String(ContactTypeIndex);
              DRYC_ARRAY[1] = String(OperationTypeIndex);
              DRYC_ARRAY[2] = String(DryContactValueEnable);
              DRYC_ARRAY[3] = String(DryContactValueDisable);

              break;

            case 3:
              preferences.putString("DRYD", DryString);
              DRYD_ARRAY[0] = String(ContactTypeIndex);
              DRYD_ARRAY[1] = String(OperationTypeIndex);
              DRYD_ARRAY[2] = String(DryContactValueEnable);
              DRYD_ARRAY[3] = String(DryContactValueDisable);

              break;

            default:
              preferences.putString("DRYD", DryString);

          }
          preferences.end();
          EEPROMbusy = false;










        }



        if (!action_done && y >= 165 && y <= 213 && x >= 400 && x <= 448 && !numpadEnable) {
          Serial.println("up");
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;


          if (ContactNameActive) {
            ContactNameIndex++;
            ContactNameIndex = ContactNameIndex % 4;
            Serial.println("ContactNameIndex:");
            Serial.println(ContactNameIndex);
          }


          if (ContactTypeActive) {
            ContactTypeIndex++;
            ContactTypeIndex = ContactTypeIndex % 6;
            Serial.println("ContactTypeIndex:");
            Serial.println(ContactTypeIndex);
          }

          if (OperationActive) {
            OperationTypeIndex++;
            OperationTypeIndex = OperationTypeIndex % 2;
            Serial.println("OperationTypeIndex:");
            Serial.println(OperationTypeIndex);
          }
        }
      }







      //MainMenuPushButtons/////////////////////////////////////////////////////////////////
      if (PageNumber == 0) {
        //Dashboard
        if (!action_done && y >= 95 && y <= 210 && x >= 0 && x <= 120 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 1;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
          buzzerActive = true;
        }

        //VoltageGraph
        if (!action_done && y >= 95 && y <= 210 && x >= 120 && x <= 240 ) {
          PageNumber = 2;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
          buzzerActive = true;
        }

        //Alarms //Current Alarms
        if (!action_done && y >= 95 && y <= 210 && x >= 240 && x <= 360 ) {
          PageNumber = 6;

          action_done = true;
          buzzerActive = true;
        }

        //Calibration
        if (!action_done && y >= 95 && y <= 210 && x >= 360 && x <= 480 ) {
          PageNumber = 12;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
          Serial.println("Calibration Page");
          buzzerActive = true;
        }


        //Energies
        if (!action_done && y >= 210 && y <= 319 && x >= 240 && x <= 360 ) {
          PageNumber = 10;
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
        }




        //Cell Monitor
        if (!action_done && y >= 210 && y <= 319 && x >= 0 && x <= 120 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 5;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
          buzzerActive = true;
        }
      }



      //AlarmList
      if (!action_done && y >= 0 && y <= 100 && x >= 400 && x <= 480 ) {
        action_done = true;
        //        buzzer_on();
        //        Serial.println("Buzzing");
        //        delay(100);
        //        buzzer_off();
        PageNumber = 11;
        buzzerActive = true;
      }


      //Mosfets   //Dry Contact
      if (!action_done && y >= 210 && y <= 319 && x >= 120 && x <= 240 && PageNumber == 0 ) {
        action_done = true;
        //        buzzer_on();
        //        Serial.println("Buzzing");
        //        delay(100);
        //        buzzer_off();
        PageNumber = 13;
        buzzerActive = true;

      }

      //Info
      if (!action_done && y >= 210 && y <= 319 && x >= 360 && x <= 480 && PageNumber == 0 ) {
        action_done = true;
        //        buzzer_on();
        //        Serial.println("Buzzing");
        //        delay(100);
        //        buzzer_off();
        PageNumber = 14;
        ReadAllParameters = true;
        buzzerActive = true;
        //        buzzer_on();
        //        Serial.println("Buzzing");
        //        delay(100);
        //        buzzer_off();
      }





      //GraphPushButtons/////////////////////////////////////////////////////////////////
      if (PageNumber == 2 || PageNumber == 3 || PageNumber == 4) {

        //VoltageGraph
        if (!action_done && y >= 275 && y <= 320 && x >= 0 && x <= 164 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 2;
          buzzerActive = true;
          action_done = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
        }

        //CurrentGraph
        if (!action_done && y >= 275 && y <= 320 && x >= 164 && x <= 328 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 3;
          action_done = true;
          buzzerActive = true;
        }

        //TempGraph
        if (!action_done && y >= 275 && y <= 320 && x >= 328 && x <= 480 ) {
          // Serial.println("Dashboard Page is written");
          PageNumber = 4;
          action_done = true;
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
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
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          buzzerActive = true;
        }
        if (!action_done && y >= 195 && y <= 255 && x >= 210 && x <= 360 && numpadEnable == false) { //open numpad for HCC
          Serial.println("HCC");
          action_done = true;
          numpadEnable = true;
          HCCEnable = true;
          numpadValue = "";
          buzzerActive = true;
        }

//        if (!action_done && y >= 105 && y <= 255 && x >= 389 && x <= 460 && numpadEnable == false) { //open numpad for HCCH
//          Serial.println("SETTED HCD & HCC");
//          action_done = true;
//          HCC_HCD = true;
//          //          buzzer_on();
//          //          Serial.println("Buzzing");
//          //          delay(100);
//          //          buzzer_off();
//          buzzerActive = true;
//
//        }

      }


      //SOC Calibration
      if (PageNumber == 12) {

        if (!action_done && y >= 115 && y <= 185 && x >= 30 && x <= 160 && numpadEnable == false) {
          Serial.println("Online Monitoring");
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("FBNR", true);
          preferences.putBool("BTNR", false);
          preferences.putBool("WDNR", false);
          preferences.putBool("UDPNR", true);
          preferences.end();
          delay(250);
          Serial.println("It will restart to configure");
          ESP.restart();
          action_done = true;
        }


        if (!action_done && y >= 115 && y <= 185 && x >= 320 && x <= 450 && numpadEnable == false) {
          Serial.println("WIFI Direct Monitoring");
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("FBNR", false);
          preferences.putBool("BTNR", false);
          preferences.putBool("WDNR", true);
          preferences.putBool("UDPNR", true);
          preferences.end();
          EEPROMbusy = false;
          delay(250);
          Serial.println("It will restart to configure");
          ESP.restart();
          action_done = true;
        }

        if (!action_done && y >= 115 && y <= 185 && x >= 175 && x <= 305 && numpadEnable == false) {
          Serial.println("Bluetooth Monitoring");
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("FBNR", false);
          preferences.putBool("BTNR", true);
          preferences.putBool("WDNR", false);
          preferences.putBool("UDPNR", false);
          preferences.end();
          EEPROMbusy = false;
          delay(250);
          Serial.println("It will restart to configure");
          ESP.restart();
          action_done = true;
        }





        if (!action_done && y >= 205 && y <= 275 && x >= 175 && x <= 305 && numpadEnable == false) { //open numpad for SOC
          Serial.println("ResetController");
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          action_done = true;
          ESP.restart();
        }

        if (!action_done && y >= 205 && y <= 275 && x >= 30 && x <= 160 && numpadEnable == false) { //open numpad for SOC
          Serial.println("updateFirmware");
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          EEPROMbusy = true;
          preferences.begin("my-app", false);
          preferences.putBool("UNR", true);
          preferences.end();
          EEPROMbusy = false;
          delay(250);
          Serial.println("It will restart to update firmware");
          ESP.restart();
          action_done = true;
        }

        if (!action_done && y >= 205 && y <= 275 && x >= 320 && x <= 450 && numpadEnable == false) { //open numpad for SOC
          Serial.println("resetNettwork");
          buzzerActive = true;
          //          buzzer_on();
          //          Serial.println("Buzzing");
          //          delay(100);
          //          buzzer_off();
          wifiManager.resetSettings();
          Serial.println("forget password");
          delay(1000);
          action_done = true;
        }
      }
















      //Operation NUMPAD
      if (numpadEnable == true ) {


        if (!action_done && y >= 146 && y <= 186 && x >= 110 && x <= 170 && numpadValue.length() < 5) { //7 BUTTON
          Serial.println("7");
          buzzerActive = true;
          action_done = true;

          numpadValue = numpadValue + "7";
        }

        if (!action_done && y >= 146 && y <= 186 && x >= 172 && x <= 232 && numpadValue.length() < 5) { //8 BUTTON
          Serial.println("8");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "8";
        }

        if (!action_done && y >= 146 && y <= 186 && x >= 234 && x <= 294 && numpadValue.length() < 5) { //9 BUTTON
          Serial.println("9");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "9";
        }

        if (!action_done && y >= 146 && y <= 186 && x >= 296 && x <= 371 ) { //DEL
          Serial.println("DEL");
          buzzerActive = true;
          action_done = true;
          int numpadLength = numpadValue.length();
          numpadValue.remove(numpadLength - 1, 1);
        }
        ////////////////////////////////////////////////////

        if (!action_done && y >= 189 && y <= 229 && x >= 110 && x <= 170 && numpadValue.length() < 5 ) { //4 BUTTON
          Serial.println("4");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "4";
        }

        if (!action_done && y >= 189 && y <= 229 && x >= 172 && x <= 232 && numpadValue.length() < 5) { //5 BUTTON
          Serial.println("5");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "5";
        }

        if (!action_done && y >= 189 && y <= 229 && x >= 234 && x <= 294 && numpadValue.length() < 5) { //6 BUTTON
          Serial.println("6");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "6";
        }

        if (!action_done && y >= 189 && y <= 229 && x >= 296 && x <= 371 ) { //CLR
          Serial.println("DEL");
          buzzerActive = true;
          action_done = true;
          numpadValue = "";
        }
        /////////////////////////////////////////////////////
        if (!action_done && y >= 232 && y <= 272 && x >= 110 && x <= 170 && numpadValue.length() < 5) { //1 BUTTON
          Serial.println("1");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "1";
        }

        if (!action_done && y >= 232 && y <= 272 && x >= 172 && x <= 232 && numpadValue.length() < 5 ) { //2 BUTTON
          Serial.println("2");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "2";
        }

        if (!action_done && y >= 232 && y <= 272 && x >= 234 && x <= 294 && numpadValue.length() < 5) { //3 BUTTON
          Serial.println("3");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "3";
        }

        if (!action_done && y >= 232 && y <= 272 && x >= 296 && x <= 371 ) { //BACK BUTTON
          Serial.println("back");
          buzzerActive = true;
          numpadEnable = false;
          action_done = true;
        }
        /////////////////////////////////////////////////////////////////////////

        if (!action_done && y >= 275 && y <= 315 && x >= 110 && x <= 230 && numpadValue.length() < 5) { //0 BUTTON
          Serial.println("0");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + "0";
        }

        if (!action_done && y >= 275 && y <= 315 && x >= 234 && x <= 294 && numpadValue.length() < 5 ) { //. BUTTON
          Serial.println(".");
          buzzerActive = true;
          action_done = true;
          numpadValue = numpadValue + ".";
        }

        if (!action_done && y >= 275 && y <= 315 && x >= 296 && x <= 371 ) { //OK BUTTON
          Serial.println("OK");
          buzzerActive = true;
          numpadEnable = false;
          action_done = true;



          if (DCEnable) {
            if (numpadValue.toFloat() <= 1000 && numpadValue.toFloat() >= 0) {
              DryContactValueEnable = numpadValue.toFloat();
              Serial.println("DC Value is setted:" + String(DryContactValueEnable));
              DCEnable = false;
            }
            else {
              Serial.println("Maximum value 100 is exceeded!");
            }
          }


          if (DCDisable) {
            if (numpadValue.toFloat() <= 1000 && numpadValue.toFloat() >= 0) {
              DryContactValueDisable = numpadValue.toFloat();
              Serial.println("DC Value is setted:" + String(DryContactValueDisable));
              DCEnable = false;
            }
            else {
              Serial.println("Maximum value 100 is exceeded!");
            }
          }



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
        buzzerActive = true;
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



void loop() {

  //  Serial.println("BMS task Remaining MEMORY USAGE: " + String(7000 - uxTaskGetStackHighWaterMark(BMS_COMM)) + " bytes");
  //  Serial.println("SDCANBUS task Remaining MEMORY USAGE: " + String(4000 - uxTaskGetStackHighWaterMark(SDCANBUS)) + " bytes");
  //  Serial.println("UDP task Remaining MEMORY USAGE: " + String(3000 - uxTaskGetStackHighWaterMark(UDPTEST)) + " bytes");
  //  Serial.println("FIREBASE task Remaining MEMORY USAGE: " + String(3000 - uxTaskGetStackHighWaterMark(FIREBASE)) + " bytes");
  // Serial.println("Enconnect task Remaining MEMORY USAGE: " + String(10000 - uxTaskGetStackHighWaterMark(ENCONNECT)) + " bytes");
//
//  Serial.print("EQUSUM STATE:");
//  Serial.println(CAL_EQUSUM);


  currentMillisUI = millis();
  if (currentMillisUI - previousMillisUI >= intervalUI) {


    Serial.println( "Free RAM1:" + ConvBinUnits(ESP.getFreeHeap(), 1));




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
        drawBanner(TimeString, DateString, "Main Menu", wifistatus, alarmNo, UpdateAvailable);
        break;

      case 1:
        drawBanner(TimeString, DateString, "Dashboard", wifistatus, alarmNo, UpdateAvailable);

        if (BMS_equ.max_cell_volt_equ >= BMS_equ.min_cell_volt_equ && BMS.max_cell_temp - 40 < 150) {

          drawDashboard(BMS.SOC * 0.1, BMS_equ.max_cell_volt_equ * 0.001, BMS_equ.min_cell_volt_equ * 0.001,
                        BMS_equ.max_cell_volt_equ * 0.001 - BMS_equ.min_cell_volt_equ * 0.001, BMS.sum_voltage * 0.1,
                        (BMS.current - 30000) * 0.1, BMS.max_cell_temp - 40);
        }





        break;

      case 2:
        drawOptimizerGraph (GraphFrame, VoltageArray, "Graph", BMS.sum_voltage * 0.1);
        drawBanner(TimeString, DateString, "Graphs", wifistatus, alarmNo, UpdateAvailable);
        drawGraphTab(true, false, false);

        break;
      case 3:
        drawOptimizerGraph (GraphFrame, CurrentArray, "Graphs", (BMS.current - 30000) * 0.1);
        drawBanner(TimeString, DateString, "Graphs", wifistatus, alarmNo, UpdateAvailable);
        drawGraphTab(false, true, false);

        break;

      case 4:
        drawOptimizerGraph (GraphFrame, TempArray, "Graphs", BMS.max_cell_temp - 40);
        drawBanner(TimeString, DateString, "Graphs", wifistatus, alarmNo, UpdateAvailable);
        //drawBarGraph(GraphFrame, dailyCells, TOLGA_RED, "Cell Values") ;
        //drawBarGraph(BarGraph, dailyGenerator, TOLGA_GREEN, "Daily Generator Energy (kWh)");

        drawGraphTab(false, false, true);

        break;

      case 5:

        drawBanner(TimeString, DateString, "Cell Volts", wifistatus, alarmNo, UpdateAvailable);
        drawCell();

        break;

      case 6:
        drawBanner(TimeString, DateString, "Dry Contacts", wifistatus, alarmNo, UpdateAvailable); //Current alarms
        // drawAlarmFrame(numpadEnable, true, false, false, false, 6, notificationEnable, notificationStatus);
        drawDryContactsReading();
        break;





      case 10:
        drawBanner(TimeString, DateString, "Energy", wifistatus, alarmNo, UpdateAvailable); //Balance alarms
        drawEnergies(ChargeEnergy, DischargeEnergy, notificationEnable);
        break;

      case 11:
        drawBanner(TimeString, DateString, "Alarm List", wifistatus, alarmNo, UpdateAvailable); //Balance alarms
        drawAlarmsList();
        break;

      case 12:
        drawBanner(TimeString, DateString, "Configuration", wifistatus, alarmNo, UpdateAvailable); //Balance alarms
        drawSOCCalibrate(numpadEnable, notificationEnable);
        break;

      case 13:
        drawBanner(TimeString, DateString, "Dry Contacts", wifistatus, alarmNo, UpdateAvailable); //Balance alarms
        //drawMosfets(notificationEnable, notificationStatus);
        drawDryContacts(numpadEnable, notificationEnable);
        break;


      case 14:
        drawBanner(TimeString, DateString, "Info", wifistatus, alarmNo, UpdateAvailable); //Balance alarms
        drawInfo();
        break;

      case 15:
        ledcWrite(ledChannel, 255);
        drawBanner(TimeString, DateString, "Update", wifistatus, alarmNo, UpdateAvailable);
        drawUpload();
        break;

      case 16:
        drawBanner(TimeString, DateString, "Barcode", wifistatus, alarmNo, UpdateAvailable);
        drawBarcode();
        break;




      default:
        PageNumber = 1;
        break;
    }

    previousMillisUI = currentMillisUI;
  }
  delay(100);
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


}



void getBatteryParameters() {

  if (!ReadAllParameters) {

    // Serial.println("ID 90 is requested...");
    BMS_recieve(0x90);
    delay(1);
    // Serial.println("ID 90 is done...");

    // Serial.println("ID 92 is requested...");
    BMS_recieve(0x92);
    delay(1);
    //Serial.println("ID 92 is done...");

    // Serial.println("ID 93 is requested...");
    BMS_recieve(0x93);
    delay(1);
    // Serial.println("ID 93 is done...");

    // Serial.println("ID 98 is requested...");
    BMS_recieve(0x98);
    delay(1);
    // Serial.println("ID 98 is done...");

    // Serial.println("ID 53 is requested...");
    BMS_recieve(0x53);
    delay(1);
    //Serial.println("ID 53 is done...");

    //Serial.println("ID 95 is requested...");
    BMS_recieve_equ(0x95);
    delay(100);
    //Serial.println("ID 95 is done...");

    //erial.println("ID 91 is requested...");
    BMS_recieve_equ(0x91);
    delay(100);
    //Serial.println("ID 91 is done...");


    // Serial.println("ID balance is requested...");
    for (uint8_t i = 0; i < 2; i++) {
      read_balance_sts_equ();
      delay(50);
    }
    // Serial.println("ID balance is done...");
    delay(100);

    if (!mute) {
      delay(200);
      buzzer_on();
      delay(200);
      buzzer_off();
    }
  }

  else {
    loading = true;
    //rated capacity
    BMS_recieve(0x50);
    delay(50);
    //Serial.print("rated capacity:");
    // Serial.println(BMS.rated_cap);
    loadingPercentage = 10;
    //charge discharge high low
    BMS_recieve(0x5B);
    delay(50);
    loadingPercentage = 20;
    //sum highlow


    BMS_recieve(0x5A);
    delay(50);
    loadingPercentage = 30;
    //CHARGE DISCHARGE TEMP
    BMS_recieve(0x5C);
    delay(50);
    loadingPercentage = 40;
    //SOC
    BMS_recieve(0x5D);
    delay(50);
    loadingPercentage = 60;
    //CUMULATIVE
    BMS_recieve(0x52);
    delay(50);
    loadingPercentage = 70;
    //vOLT DIFF
    BMS_recieve(0x5E);
    delay(50);
    loadingPercentage = 80;
    //    BALANCE START
    BMS_recieve(0x5F);
    delay(50);
    loadingPercentage = 90;
    //    cell volt high low
    BMS_recieve(0x59);
    delay(50);
    loadingPercentage = 95;





    BMS_recieve_equ(0x5F);

    loadingPercentage = 97;

    BMS_recieve_equ(0x5A);

    loadingPercentage = 98;



    BMS_recieve_equ(0x59);
    loadingPercentage = 99;



    BMS_recieve_equ(0x53);
    delay(50);

    loadingPercentage = 100;
    delay(1000);
    loading = false;
    ReadAllParameters = false;
    Serial.println("All parameters are read");
  }
}




//DISPLAY FUNCTIONS///////////////////////////////////////




void drawDryContactsReading() {


  //OperationList[OperationTypeIndex]
  //ContactFunctionList[ContactTypeIndex]
  //  DRYD_ARRAY

  PageFrame.deleteSprite();
  PageFrame.createSprite(480, 225);

  PageFrame.setTextDatum(MC_DATUM);

  //PageFrame.fillRoundRect(20, 20, 160, 90, 5, TOLGA_DARKSEA);
  PageFrame.drawRoundRect(20, 20, 440, 162, 5, TOLGA_BLUE);
  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.fillRoundRect(22, 22, 436, 30, 5, TOLGA_DARKSEA);

  PageFrame.drawString("Contact Name", 75, 40, 2);
  PageFrame.drawString("Contact Type", 185, 40, 2);
  PageFrame.drawString("Operation", 275, 40, 2);
  PageFrame.drawString("Enable", 335, 40, 2);
  PageFrame.drawString("Disable", 385, 40, 2);
  PageFrame.drawString("Status", 435, 40, 2);


  PageFrame.setTextColor(TOLGA_YELLOW);

  PageFrame.fillRoundRect(22, 54, 436, 30, 5, TOLGA_DARKSEA);
  PageFrame.drawString("Dry Contact A", 75, 72, 2);
  PageFrame.fillRoundRect(22, 86, 436, 30, 5, TOLGA_DARKSEA);
  PageFrame.drawString("Dry Contact B", 75, 104, 2);
  PageFrame.fillRoundRect(22, 118, 436, 30, 5, TOLGA_DARKSEA);
  PageFrame.drawString("Dry Contact C", 75, 136, 2);
  PageFrame.fillRoundRect(22, 150, 436, 30, 5, TOLGA_DARKSEA);
  PageFrame.drawString("Dry Contact D", 75, 168, 2);

  PageFrame.setTextColor(TOLGA_BLUE);
  PageFrame.drawString(ContactFunctionList[DRYA_ARRAY[0].toInt()], 185, 72, 2);
  PageFrame.drawString(OperationList[DRYA_ARRAY[1].toInt()], 275, 72, 2);
  PageFrame.drawString(DRYA_ARRAY[2], 335, 72, 2);
  PageFrame.drawString(DRYA_ARRAY[3], 385, 72, 2);

  if (DRY_VALUES[0] == 0) {
    PageFrame.drawString("OFF", 435, 72, 2);
  }
  else {
    PageFrame.drawString("ON", 435, 72, 2);
  }


  PageFrame.setTextColor(TOLGA_BLUE);
  PageFrame.drawString(ContactFunctionList[DRYB_ARRAY[0].toInt()], 185, 104, 2);
  PageFrame.drawString(OperationList[DRYB_ARRAY[1].toInt()], 275, 104, 2);
  PageFrame.drawString(DRYB_ARRAY[2], 335, 104, 2);
  PageFrame.drawString(DRYB_ARRAY[3], 385, 104, 2);

  if (DRY_VALUES[1] == 0) {
    PageFrame.drawString("OFF", 435, 104, 2);
  }
  else {
    PageFrame.drawString("ON", 435, 104, 2);
  }





  PageFrame.setTextColor(TOLGA_BLUE);
  PageFrame.drawString(ContactFunctionList[DRYC_ARRAY[0].toInt()], 185, 136, 2);
  PageFrame.drawString(OperationList[DRYC_ARRAY[1].toInt()], 275, 136, 2);
  PageFrame.drawString(DRYC_ARRAY[2], 335, 136, 2);
  PageFrame.drawString(DRYC_ARRAY[3], 385, 136, 2);



  if (DRY_VALUES[2] == 0) {
    PageFrame.drawString("OFF", 435, 136, 2);
  }
  else {
    PageFrame.drawString("ON", 435, 136, 2);
  }

  PageFrame.setTextColor(TOLGA_BLUE);
  PageFrame.drawString(ContactFunctionList[DRYD_ARRAY[0].toInt()], 185, 168, 2);
  PageFrame.drawString(OperationList[DRYD_ARRAY[1].toInt()], 275, 168, 2);
  PageFrame.drawString(DRYD_ARRAY[2], 335, 168, 2);
  PageFrame.drawString(DRYD_ARRAY[3], 385, 168, 2);


  if (DRY_VALUES[3] == 0) {
    PageFrame.drawString("OFF", 435, 168, 2);
  }
  else {
    PageFrame.drawString("ON", 435, 168, 2);
  }


  PageFrame.drawRect(130, 20, 1, 162, TOLGA_BLUE);
  PageFrame.drawRect(240, 20, 1, 162, TOLGA_BLUE);
  PageFrame.drawRect(310, 20, 1, 162, TOLGA_BLUE);
  PageFrame.drawRect(360, 20, 1, 162, TOLGA_BLUE);
  PageFrame.drawRect(410, 20, 1, 162, TOLGA_BLUE);


  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.drawRoundRect(20, 188, 440, 30, 5, TOLGA_BLUE);
  PageFrame.fillRoundRect(21, 189, 438, 28, 5, TOLGA_DARKSEA);
  PageFrame.drawString("Digital Inputs", 75, 200, 2);
  PageFrame.setTextColor(TOLGA_BLUE);

  Icon1.createSprite(24, 24);
  Icon1.setSwapBytes(true);

  if (GPIO_READ[0])
  {
    Icon1.pushImage(0, 0, 24, 24, onequ);
  }
  else {
    Icon1.pushImage(0, 0, 24, 24, offequ);
  }
  Icon1.pushToSprite(&PageFrame, 200, 190, TFT_BLACK);
  PageFrame.drawString("Input#1", 170, 201, 2);



  Icon2.createSprite(24, 24);
  Icon2.setSwapBytes(true);

  if (GPIO_READ[1])
  {
    Icon2.pushImage(0, 0, 24, 24, onequ);
  }
  else {
    Icon2.pushImage(0, 0, 24, 24, offequ);
  }
  Icon2.pushToSprite(&PageFrame, 300, 190, TFT_BLACK);
  PageFrame.drawString("Input#2", 270, 201, 2);


  Icon3.createSprite(24, 24);
  Icon3.setSwapBytes(true);

  if (GPIO_READ[2])
  {
    Icon3.pushImage(0, 0, 24, 24, onequ);
  }
  else {
    Icon3.pushImage(0, 0, 24, 24, offequ);
  }
  Icon3.pushToSprite(&PageFrame, 400, 190, TFT_BLACK);
  PageFrame.drawString("Input#3", 370, 201, 2);











  PageFrame.pushSprite(0, 95);

  PageFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();
  Icon7.deleteSprite();
  Icon8.deleteSprite();
  Icon9.deleteSprite();
  Icon10.deleteSprite();
  Icon11.deleteSprite();
  Icon12.deleteSprite();
  Icon13.deleteSprite();


}



void drawDryContacts(bool numpad, bool Notification) {


  if (Notification) {
    Serial.println("Notification started");
    PageFrame.deleteSprite();
    PageFrame.createSprite(480, 225);
    PageFrame.fillSprite(TFT_BLACK);
    drawNotification(PageFrame , true);
    PageFrame.pushSprite(0, 95);

  }
  else {

    if (numpad) {
      PageFrame.deleteSprite();
      PageFrame.createSprite(480, 225);
      PageFrame.fillSprite(TFT_BLACK);
      drawNumpad(PageFrame);
      PageFrame.pushSprite(0, 95);
    }

    else {
      PageFrame.deleteSprite();
      PageFrame.createSprite(480, 225);
      PageFrame.setTextColor(TFT_WHITE);

      Icon1.createSprite(48, 48);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 48, 48, up);
      Icon1.pushToSprite(&PageFrame, 400, 70, TFT_BLACK);




      Icon3.createSprite(48, 48);
      Icon3.setSwapBytes(true);
      Icon3.pushImage(0, 0, 48, 48, save);
      Icon3.pushToSprite(&PageFrame, 400, 150, TFT_BLACK);

      if (ContactNameActive) {
        PageFrame.fillRoundRect(30, 20, 160, 90, 5, TOLGA_DARKSEA);
        PageFrame.drawRoundRect(30, 20, 160, 90, 5, TOLGA_BLUE);
      }
      else {
        PageFrame.drawRoundRect(30, 20, 160, 90, 5, TOLGA_DARKSEA);
      }


      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.drawString("Name", 110, 45 );
      PageFrame.setTextColor(TOLGA_YELLOW);
      PageFrame.drawString(ContactNameList[ContactNameIndex], 110, 85, 4);
      PageFrame.setTextColor(TFT_WHITE);


      if (ContactTypeActive) {
        PageFrame.fillRoundRect(200, 20, 160, 90, 5, TOLGA_DARKSEA);
        PageFrame.drawRoundRect(200, 20, 160, 90, 5, TOLGA_BLUE);
      }
      else {
        PageFrame.drawRoundRect(200, 20, 160, 90, 5, TOLGA_DARKSEA);
      }

      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.drawString("Type", 280, 45);
      PageFrame.setTextColor(TOLGA_YELLOW);
      PageFrame.drawString(ContactFunctionList[ContactTypeIndex], 280, 85, 4);
      PageFrame.drawRoundRect(30, 120, 160, 90, 5, TOLGA_DARKSEA);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.drawString("Enable", 110, 145 );
      PageFrame.setTextColor(TOLGA_YELLOW);
      PageFrame.drawString(String(DryContactValueEnable), 110, 185, 4);

      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawRoundRect(200, 120, 160, 90, 5, TOLGA_DARKSEA); //8
      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.drawString("Disable", 280, 145);
      PageFrame.setTextColor(TOLGA_YELLOW);

      PageFrame.drawString(String(DryContactValueDisable), 280, 185, 4);

      PageFrame.setTextColor(TFT_WHITE);
      if (OperationActive) {
        PageFrame.fillRoundRect(380, 20, 80, 40, 5, TOLGA_DARKSEA);
        PageFrame.drawRoundRect(380, 20, 80, 40, 5, TOLGA_BLUE);
      }
      else
      {
        PageFrame.drawRoundRect(380, 20, 80, 40, 5, TOLGA_DARKSEA);
      }

      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.setTextColor(TOLGA_YELLOW);
      PageFrame.drawString(OperationList[OperationTypeIndex], 420, 40, 4);




      PageFrame.pushSprite(0, 95);
      PageFrame.deleteSprite();
      Icon1.deleteSprite();
      Icon2.deleteSprite();
      Icon3.deleteSprite();
      Icon4.deleteSprite();
      Icon5.deleteSprite();
      Icon6.deleteSprite();
      Icon7.deleteSprite();
      Icon8.deleteSprite();
      Icon9.deleteSprite();
      Icon10.deleteSprite();
      Icon11.deleteSprite();
      Icon12.deleteSprite();
      Icon13.deleteSprite();
    }
  }
}





void drawMainMenu2() {
  PageFrame.createSprite(480, 225);
  PageFrame.setTextColor(TFT_WHITE);

  Icon6.createSprite(48, 48);
  Icon6.setSwapBytes(true);
  Icon6.pushImage(0, 0, 48, 48, drycontacts);
  Icon6.pushToSprite(&PageFrame, 40, 20, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Dry C.", 70, 105);
  PageFrame.setTextDatum(TL_DATUM);


  PageFrame.pushSprite(0, 95);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();
  Icon7.deleteSprite();
  Icon8.deleteSprite();
  Icon9.deleteSprite();
  Icon10.deleteSprite();
  Icon11.deleteSprite();
  Icon12.deleteSprite();
  Icon13.deleteSprite();
}


void drawBanner(String Time, String Date, String Page, bool wifistatus, int notification_no, bool UpdateAvailable) {

  BannerFrame.createSprite(480, 95);
  BannerFrame.setTextColor(TFT_WHITE);
  BannerFrame.setTextDatum(TL_DATUM);
  Icon1.createSprite(480, 95);
  Icon1.setSwapBytes(true);
  Icon1.pushImage(0, 0, 480, 95, Banner1);
  Icon1.pushToSprite(&BannerFrame, 0, 0, TFT_BLACK);

  // Banner.setFreeFont(&Orbitron_Light_24);
  Icon2.createSprite(16, 16);
  Icon2.setSwapBytes(true);
  Icon2.pushImage(0, 0, 16, 16, clock1);
  Icon2.pushToSprite(&BannerFrame, 10, 79, TFT_BLACK);
  BannerFrame.drawString(Time, 35, 78, 2);


  Icon6.createSprite(16, 16);
  Icon6.setSwapBytes(true);

  if (mute) {
    Icon6.pushImage(0, 0, 16, 16, mute_icon);
  }
  else {
    Icon6.pushImage(0, 0, 16, 16, unmute_icon);
  }

  Icon6.pushToSprite(&BannerFrame, 90, 79, TFT_BLACK);




  BannerFrame.setTextDatum(BC_DATUM);
  BannerFrame.setFreeFont(&Orbitron_Light_32);
  BannerFrame.drawString(Page, 240, 90);
  BannerFrame.setTextDatum(TL_DATUM);

  //FirmwareVersion+RESTART+SERIAL
  //  BannerFrame.setTextFont(GLCD);
  //  BannerFrame.drawString("Ver:" + FirmwareVer, 395, 5);
  //  BannerFrame.drawString("R:" + String(restartCounter), 395, 18);
  //  BannerFrame.drawString("SR:" + SerialNumber, 395, 32);
  //  BannerFrame.setFreeFont(&Orbitron_Light_32);



  //Banner.setFreeFont(&Orbitron_Light_24);
  //BannerFrame.drawString(String(notification_no), 460, 78, 2);
  BannerFrame.drawString(String(alarmNo), 460, 78, 2);




  Icon3.createSprite(16, 16);
  Icon3.setSwapBytes(true);
  Icon3.pushImage(0, 0, 16, 16, notification);
  Icon3.pushToSprite(&BannerFrame, 440, 79, TFT_BLACK);

  Icon4.createSprite(16, 16);
  Icon4.setSwapBytes(true);
  if (wifistatus)
    Icon4.pushImage(0, 0, 16, 16, wifi);
  else

    if (WIFIBT) {
      Icon4.pushImage(0, 0, 16, 16, bluetooth);
    }
    else {
      Icon4.pushImage(0, 0, 16, 16, nowifi);
    }


  Icon4.pushToSprite(&BannerFrame, 410, 79, TFT_BLACK);


  if (UpdateAvailable) {
    Icon5.createSprite(16, 16);
    Icon5.setSwapBytes(true);
    Icon5.pushImage(0, 0, 16, 16, updateReady);
    Icon5.pushToSprite(&BannerFrame, 385, 79, TFT_BLACK);
  }

  BannerFrame.pushSprite(0, 0);
  BannerFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();


}

void drawMainMenu() {
  PageFrame.createSprite(480, 225);
  PageFrame.setTextColor(TFT_WHITE);

  Icon6.createSprite(48, 48);
  Icon6.setSwapBytes(true);
  Icon6.pushImage(0, 0, 48, 48, dashboard);
  Icon6.pushToSprite(&PageFrame, 40, 20, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Dashb.", 70, 105);
  PageFrame.setTextDatum(TL_DATUM);

  Icon7.createSprite(48, 48);
  Icon7.setSwapBytes(true);
  Icon7.pushImage(0, 0, 48, 48, graph);
  Icon7.pushToSprite(&PageFrame, 155, 15, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Graphs", 185, 105);
  PageFrame.setTextDatum(TL_DATUM);

  Icon8.createSprite(48, 48);
  Icon8.setSwapBytes(true);
  Icon8.pushImage(0, 0, 48, 48, dryread);
  Icon8.pushToSprite(&PageFrame, 275, 15, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Dry R.", 305, 105);
  PageFrame.setTextDatum(TL_DATUM);

  Icon9.createSprite(48, 48);
  Icon9.setSwapBytes(true);
  Icon9.pushImage(0, 0, 48, 48, calibration);
  Icon9.pushToSprite(&PageFrame, 390, 15, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Conf.", 420, 105);
  PageFrame.setTextDatum(TL_DATUM);

  Icon10.createSprite(48, 48);
  Icon10.setSwapBytes(true);
  Icon10.pushImage(0, 0, 48, 48, cell);
  Icon10.pushToSprite(&PageFrame, 40, 120, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Cells", 70, 205);
  PageFrame.setTextDatum(TL_DATUM);

  Icon11.createSprite(48, 48);
  Icon11.setSwapBytes(true);
  Icon11.pushImage(0, 0, 48, 48, drywrite);
  Icon11.pushToSprite(&PageFrame, 155, 120, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Dry W.", 185, 205);
  PageFrame.setTextDatum(TL_DATUM);

  Icon12.createSprite(48, 48);
  Icon12.setSwapBytes(true);
  Icon12.pushImage(0, 0, 48, 48, energy);
  Icon12.pushToSprite(&PageFrame, 275, 120, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Energy", 305, 205);
  PageFrame.setTextDatum(TL_DATUM);

  Icon13.createSprite(48, 48);
  Icon13.setSwapBytes(true);
  Icon13.pushImage(0, 0, 48, 48, settings);
  Icon13.pushToSprite(&PageFrame, 390, 120, TFT_BLACK);
  PageFrame.setTextDatum(BC_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.drawString("Info", 420, 205);
  PageFrame.setTextDatum(TL_DATUM);


  PageFrame.pushSprite(0, 95);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();
  Icon7.deleteSprite();
  Icon8.deleteSprite();
  Icon9.deleteSprite();
  Icon10.deleteSprite();
  Icon11.deleteSprite();
  Icon12.deleteSprite();
  Icon13.deleteSprite();

}


void drawDashboard(int SOC, float MaxVolt, float MinVolt, float MaxDiff, float TerminalVolt, float TerminalCurr, float TerminalTemp) {



  if ((MaxVolt >= MinVolt && MaxVolt > 2.2 && MinVolt > 2.2) || TerminalTemp < 200 ) {
    PageFrame.createSprite(480, 225);
    PageFrame.setFreeFont(&Orbitron_Light_24);
    PageFrame.setTextDatum(TL_DATUM);
    ringMeter(SOC, 0, 100, 155, 15, 80, "SOC", TOLGA_BLUE, PageFrame); // Draw analogue meter

    PageFrame.setFreeFont(&Orbitron_Light_24);
    PageFrame.setTextDatum(TL_DATUM);

    //Maximum Voltage
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon6.createSprite(48, 48);
    Icon6.setSwapBytes(true);
    Icon6.pushImage(0, 0, 48, 48, maxIcon);
    Icon6.pushToSprite(&PageFrame, 15, 15, TFT_BLACK);
    PageFrame.drawString("Maximum ", 70, 15, 2);
    PageFrame.drawString("Voltage ", 72, 30, 2);
    PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    PageFrame.drawString(String(MaxVolt, 3) + "V", 72, 50, 2);

    //Minimum Voltage
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon7.createSprite(48, 48);
    Icon7.setSwapBytes(true);
    Icon7.pushImage(0, 0, 48, 48, minIcon);
    Icon7.pushToSprite(&PageFrame, 15, 90, TFT_BLACK);
    PageFrame.drawString("Minimum ", 70, 90, 2);
    PageFrame.drawString("Voltage ", 72, 105, 2);
    PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    PageFrame.drawString(String(MinVolt, 3) + "V", 72, 125, 2);

    //Diff Voltage
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon8.createSprite(48, 48);
    Icon8.setSwapBytes(true);
    Icon8.pushImage(0, 0, 48, 48, delta);
    Icon8.pushToSprite(&PageFrame, 15, 165, TFT_BLACK);
    PageFrame.drawString("Diffr. ", 70, 165, 2);
    PageFrame.drawString("Voltage ", 72, 180, 2);
    PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    PageFrame.drawString(String(MaxDiff, 3) + "V", 72, 200, 2);

    //Total Voltage
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon9.createSprite(48, 48);
    Icon9.setSwapBytes(true);
    Icon9.pushImage(0, 0, 48, 48, voltage);
    Icon9.pushToSprite(&PageFrame, 350, 20, TFT_BLACK);
    PageFrame.drawString("Terminal ", 405, 20, 2);
    PageFrame.drawString("Voltage ", 407, 35, 2);
    PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    PageFrame.drawString(String(TerminalVolt) + "V", 407, 55, 2);

    //Terminal Current
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon10.createSprite(48, 48);
    Icon10.setSwapBytes(true);
    Icon10.pushImage(0, 0, 48, 48, current);
    Icon10.pushToSprite(&PageFrame, 350, 90, TFT_BLACK);
    PageFrame.drawString("Terminal ", 405, 90, 2);
    PageFrame.drawString("Current ", 407, 105, 2);
    PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    PageFrame.drawString(String(TerminalCurr) + "A", 407, 125, 2);

    //Terminal Temp
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon11.createSprite(48, 48);
    Icon11.setSwapBytes(true);
    Icon11.pushImage(0, 0, 48, 48, temp);
    Icon11.pushToSprite(&PageFrame, 350, 165, TFT_BLACK);
    PageFrame.drawString("Cell ", 405, 165, 2);
    PageFrame.drawString("Temp. ", 407, 180, 2);
    PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
    PageFrame.drawString(String(TerminalTemp) + "C", 407, 200, 2);



    //Charge Terminal
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon12.createSprite(24, 24);
    Icon12.setSwapBytes(true);


    if (BMS.charge)
    {
      Icon12.pushImage(0, 0, 24, 24, onequ);
    }
    else {
      Icon12.pushImage(0, 0, 24, 24, offequ);
    }
    Icon12.pushToSprite(&PageFrame, 170, 195, TFT_BLACK);
    PageFrame.drawString("Char.", 170, 180, 2);

    //Discharge Terminal
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon13.createSprite(24, 24);
    Icon13.setSwapBytes(true);


    if (BMS.discharge) {
      Icon13.pushImage(0, 0, 24, 24, onequ);
    }
    else {
      Icon13.pushImage(0, 0, 24, 24, offequ);
    }
    Icon13.pushToSprite(&PageFrame, 220, 195, TFT_BLACK);
    PageFrame.drawString("Dsch.", 220, 180, 2);


    //Equ Terminal
    PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
    Icon13.createSprite(24, 24);
    Icon13.setSwapBytes(true);


    if (BMS_equ.balance_status_equ) {
      Icon13.pushImage(0, 0, 24, 24, onequ);
    }
    else {
      Icon13.pushImage(0, 0, 24, 24, offequ);
    }


    Icon13.pushToSprite(&PageFrame, 270, 195, TFT_BLACK);
    PageFrame.drawString("Equ.", 270, 180, 2);





    PageFrame.pushSprite(0, 95);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
    Icon3.deleteSprite();
    Icon4.deleteSprite();
    Icon5.deleteSprite();
    Icon6.deleteSprite();
    Icon7.deleteSprite();
    Icon8.deleteSprite();
    Icon9.deleteSprite();
    Icon10.deleteSprite();
    Icon11.deleteSprite();
    Icon12.deleteSprite();
    Icon13.deleteSprite();
  }
}


void drawLoading(TFT_eSprite & tft, int percentage) {

  PageFrame.createSprite(480, 225);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.fillSprite(TFT_BLACK);
  PageFrame.setFreeFont(&Orbitron_Light_32);

  PageFrame.fillRoundRect(80, 20, 320, 70, 5, TFT_BACKGROUND); //8
  PageFrame.setTextDatum(MC_DATUM);
  PageFrame.drawString("Loading please wait", 240, 50);
  PageFrame.fillRoundRect(80, 85, 320, 40, 5, TOLGA_RED); //8
  PageFrame.drawString(String(percentage) + "%", 240, 100);
  PageFrame.setTextDatum(TL_DATUM);

  PageFrame.pushToSprite(&tft, 0, 0, TFT_BLACK);
  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.deleteSprite();
}

void drawInfo() {

  if (loading) {
    Serial.println("loading screen");
    NotificationFrame.createSprite(480, 225);
    NotificationFrame.setTextDatum(TL_DATUM);
    NotificationFrame.setTextColor(TFT_WHITE);
    NotificationFrame.fillSprite(TFT_BLACK);
    drawLoading(NotificationFrame, loadingPercentage);
    NotificationFrame.pushSprite(0, 95);
  }
  else {
    loadingPercentage = 0;
    NotificationFrame.createSprite(480, 225);
    NotificationFrame.setTextDatum(TL_DATUM);
    NotificationFrame.setTextColor(TFT_WHITE);
    NotificationFrame.fillSprite(TFT_BLACK);
    NotificationFrame.setFreeFont(&Orbitron_Light_32);

    NotificationFrame.drawRoundRect(10, 10, 463, 200, 5, TFT_BACKGROUND);
    NotificationFrame.drawLine(10, 39, 469, 39, TFT_BACKGROUND);
    NotificationFrame.drawLine(10, 68, 469, 68, TFT_BACKGROUND);
    NotificationFrame.drawLine(10, 97, 469, 97, TFT_BACKGROUND);
    NotificationFrame.drawLine(10, 126, 469, 126, TFT_BACKGROUND);
    NotificationFrame.drawLine(10, 155, 469, 155, TFT_BACKGROUND);
    NotificationFrame.drawLine(10, 182, 469, 182, TFT_BACKGROUND);

    NotificationFrame.drawLine(174, 10, 174, 209, TFT_BACKGROUND);

    NotificationFrame.drawLine(325, 10, 325, 209, TFT_BACKGROUND);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("DischCurHi:", 20, 18, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString (String(BMS.dischar_curr2) + "A", 105, 18, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("ChargeCurHi:", 20, 47, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.charge_curr2) + "A", 105, 47, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);


    NotificationFrame.drawString("SumVoltHi:", 20, 76, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.sumv_high2 / 10) + "V", 95, 76, 2);
    NotificationFrame.setTextColor(TOLGA_YELLOW);
    NotificationFrame.drawString( "/", 125, 76, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS_equ.sumv_high2_equ / 10) + "V", 135, 76, 2);
    NotificationFrame.setTextColor(TOLGA_BLUE);

    NotificationFrame.drawString("SumVoltLo:", 20, 105, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.sumv_low2 / 10) + "V", 95, 105, 2);
    NotificationFrame.setTextColor(TOLGA_YELLOW);
    NotificationFrame.drawString( "/", 125, 105, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS_equ.sumv_low2_equ / 10) + "V", 135, 105, 2);


    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("CellVoltHi:", 20, 134, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.cell_volthigh2 * 0.001, 3) + "V", 105, 134, 2);
    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("CellVoltLo:", 20, 163, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.cell_voltlow2 * 0.001, 3) + "V", 105, 163, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("Date:", 20, 190, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(DateString), 80, 190, 2);


    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("BalVolt:", 190, 18, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS_equ.balance_volt_equ * 0.001, 3) + "V", 265, 18, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("BalDeltaV:", 190, 47, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS_equ.balance_volt_diff_equ) + "mV", 265, 47, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("ChgTempHi:", 190, 76, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.charge_temp_high2) + "C", 265, 76, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("ChgTempLo:", 190, 105, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.charge_temp_low2) + "C", 265, 105, 2);

    Serial.println();

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("DisTempHi:", 190, 134, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.discharge_temp_high2) + "C", 265, 134, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("DisTempLo:", 190, 163, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.discharge_temp_low2) + "C", 265, 163, 2);

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("RatedVolt:", 190, 190, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.rated_volt * 0.001, 3) + "V", 265, 190, 2);

    ///////////////////////////////////////////////////////

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("SOCHi", 335, 18, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.SOC_high2) + "%", 408, 18, 2);



    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("SOCLo:", 335, 47, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.SOC_low2) + "%", 408, 47, 2);







    /////////////////////////////////////////


    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("VoltDif:", 335, 76, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.volt_diff2) + "mV", 410, 76, 2);


    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("System Cap:", 335, 105, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.rated_cap * 0.001 * 48 * 0.001) + "kWh", 410, 105, 2);


    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("SleepTime", 335, 134, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( String(BMS.secondsleepbm), 410, 134, 2);


    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("IP:", 335, 163, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( IP , 370, 163, 2);



    if (SSIDName.length() > 12) {
      SSIDName = SSIDName.substring(0, 8);
    }

    NotificationFrame.setTextColor(TOLGA_BLUE);
    NotificationFrame.drawString("SSID:", 335, 190, 2);
    NotificationFrame.setTextColor(TOLGA_RED);
    NotificationFrame.drawString( SSIDName , 370, 190, 2);
    NotificationFrame.pushSprite(0, 95);
    NotificationFrame.deleteSprite();
  }
}

void drawMosfets(bool Notification, bool Status) {
  if (Notification) {
    PageFrame.createSprite(480, 225);
    PageFrame.fillSprite(TFT_BLACK);
    drawNotification(PageFrame , Status);
    PageFrame.pushSprite(0, 95);
  }
  else {
    PageFrame.createSprite(480, 225);
    PageFrame.setTextDatum(TL_DATUM);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.fillSprite(TFT_BLACK);
    PageFrame.setFreeFont(&Orbitron_Light_32);

    PageFrame.fillRoundRect(160, 20, 210, 190, 5, TFT_BACKGROUND);
    PageFrame.fillRoundRect(160, 20, 210, 30, 5, TOLGA_DARKSEA);
    PageFrame.setTextColor(TFT_WHITE);
    PageFrame.drawString("Read", 260, 20);

    //    PageFrame.fillRoundRect(250, 20, 210, 190, 5, TFT_BACKGROUND);
    //    PageFrame.fillRoundRect(250, 20, 210, 30, 5, TOLGA_DARKSEA);
    //    PageFrame.setTextColor(TFT_WHITE);
    //    PageFrame.drawString("Write", 320, 20);

    Icon1.createSprite(48, 48);
    Icon1.setSwapBytes(true);
    Icon1.pushImage(0, 0, 48, 48, switchon);

    Icon2.createSprite(48, 48);
    Icon2.setSwapBytes(true);
    Icon2.pushImage(0, 0, 48, 48, switchoff);





    PageFrame.drawString("Charge", 180, 60);
    PageFrame.drawString("Disch.", 180, 110);
    //PageFrame.drawString("Balance", 40, 160);

    if (BMS.charge) {
      Icon1.pushToSprite(&PageFrame, 306, 55, TFT_BLACK);
    }
    else {
      Icon2.pushToSprite(&PageFrame, 305, 55, TFT_BLACK);
    }

    if (BMS.discharge) {
      Icon1.pushToSprite(&PageFrame, 305, 105, TFT_BLACK);
    }
    else {
      Icon2.pushToSprite(&PageFrame, 305, 105, TFT_BLACK);
    }


    //    if (WriteCharge) {
    //      Icon1.pushToSprite(&PageFrame, 280, 55, TFT_BLACK);
    //    }
    //    else {
    //      Icon2.pushToSprite(&PageFrame, 280, 55, TFT_BLACK);
    //    }
    //
    //    if (WriteDischarge) {
    //      Icon1.pushToSprite(&PageFrame, 280, 105, TFT_BLACK);
    //    }
    //    else {
    //      Icon2.pushToSprite(&PageFrame, 280, 105, TFT_BLACK);
    //    }
    //
    //    if (WriteBalance) {
    //      Icon1.pushToSprite(&PageFrame, 280, 155, TFT_BLACK);
    //    }
    //    else {
    //      Icon2.pushToSprite(&PageFrame, 280, 155, TFT_BLACK);
    //    }

    //    PageFrame.fillRoundRect(360, 60, 80, 40, 5, TOLGA_DARKSEA);
    //    PageFrame.drawString("SET", 370, 65);
    //
    //    PageFrame.fillRoundRect(360, 110, 80, 40, 5, TOLGA_DARKSEA);
    //    PageFrame.drawString("SET", 370, 115);
    //
    //    PageFrame.fillRoundRect(360, 160, 80, 40, 5, TOLGA_DARKSEA);
    //    PageFrame.drawString("SET", 370, 165);







    PageFrame.pushSprite(0, 95);
    PageFrame.deleteSprite();
    Icon1.deleteSprite();
    Icon2.deleteSprite();
    Icon3.deleteSprite();
    Icon4.deleteSprite();
    Icon5.deleteSprite();
    Icon6.deleteSprite();
    Icon7.deleteSprite();
    Icon8.deleteSprite();
    Icon9.deleteSprite();
    Icon10.deleteSprite();
    Icon11.deleteSprite();
    Icon12.deleteSprite();
    Icon13.deleteSprite();
  }
}


void drawSOCCalibrate(bool numpad, bool Notification) {


  if (Notification) {
    PageFrame.createSprite(480, 225);
    PageFrame.fillSprite(TFT_BLACK);
    drawNotification(PageFrame , true);
    PageFrame.pushSprite(0, 95);
  }
  else {



    if (numpad) {
      PageFrame.deleteSprite();
      PageFrame.createSprite(480, 225);
      PageFrame.fillSprite(TFT_BLACK);
      drawNumpad(PageFrame);
      PageFrame.pushSprite(0, 95);
    }
    else {
      PageFrame.deleteSprite();
      PageFrame.createSprite(480, 225);
      PageFrame.fillSprite(TFT_BLACK);
      PageFrame.setTextDatum(TL_DATUM);
      PageFrame.setFreeFont(&Orbitron_Light_32);

      //      PageFrame.drawString("SOC", 42, 20);
      //      PageFrame.drawString("Percent.", 42, 50);
      //      PageFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
      //      PageFrame.setTextColor(TOLGA_BLUE);
      //      PageFrame.setTextDatum(TR_DATUM);
      //      PageFrame.drawString(String(SOC_CAL_VAL, 1) + "%", 350, 30);
      //
      //      PageFrame.setTextDatum(TL_DATUM);
      //      PageFrame.fillRoundRect(380, 18, 70, 60, 5, TFT_RED);
      //      PageFrame.setTextColor(TFT_WHITE);
      //      PageFrame.drawString("SET", 389, 30);


      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.fillRoundRect(30, 20, 130, 70, 5, TFT_RED);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawString("Online", 95, 34);
      PageFrame.drawString("Mon.", 95, 64);

      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.fillRoundRect(175, 20, 130, 70, 5, TFT_RED);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawString("Enable", 240, 34);
      PageFrame.drawString("Blue.", 240, 64);

      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.fillRoundRect(320, 20, 130, 70, 5, TFT_RED);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawString("Enable", 385, 34);
      PageFrame.drawString("WifiDir.", 385, 64);










      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.fillRoundRect(30, 110, 130, 70, 5, TFT_RED);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawString("Update", 95, 124);
      PageFrame.drawString("Firm.", 95, 154);

      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.fillRoundRect(175, 110, 130, 70, 5, TFT_RED);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawString("Reset", 240, 124);
      PageFrame.drawString("Unit", 240, 154);

      PageFrame.setTextDatum(MC_DATUM);
      PageFrame.fillRoundRect(320, 110, 130, 70, 5, TFT_RED);
      PageFrame.setTextColor(TFT_WHITE);
      PageFrame.drawString("Reset", 385, 124);
      PageFrame.drawString("Network", 385, 154);


      PageFrame.pushSprite(0, 95);
      PageFrame.deleteSprite();
    }
  }

}


void drawConfirmation(TFT_eSprite & tft) {
  tft.deleteSprite();
  tft.createSprite(480, 225);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.fillSprite(TFT_BLACK);
  tft.setFreeFont(&Orbitron_Light_32);


  tft.drawString("YES", 80, 115);

  tft.fillRoundRect(80, 20, 320, 180, 5, TFT_BACKGROUND); //8
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Are you sure?", 240, 50);
  tft.setTextDatum(TL_DATUM);

  tft.fillRoundRect(100, 100, 100, 60, 5, TOLGA_RED); //8
  tft.drawRoundRect(100, 100, 100, 60, 5, TOLGA_BLUE_2);
  tft.drawString("YES", 120, 115);


  tft.fillRoundRect(280, 100, 100, 60, 5, TOLGA_RED); //8
  tft.drawRoundRect(280, 100, 100, 60, 5, TOLGA_BLUE_2);
  tft.drawString("NO", 310, 115);

  tft.pushToSprite(&tft, 0, 0, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.deleteSprite();
}


void drawAlarmsList() {

  int xcoor = 40;
  int ycoor = 20;
  int errorno = 0;
  PageFrame.createSprite(480, 225);
  PageFrame.fillSprite(TFT_BLACK);

  PageFrame.setTextDatum(TL_DATUM);
  PageFrame.setFreeFont(&Orbitron_Light_32);

  for (int i = 0; i < 28; i++) {

    if (BMS.error[i]) {

      //Serial.print("AlarmCode:");
      Serial.println(BMS.error[i]);


      if (errorno < 5) {
        PageFrame.fillRoundRect(xcoor - 20, ycoor + errorno * 50, 220, 30, 5, TOLGA_RED);
        PageFrame.setTextColor(TOLGA_BLUE);
        PageFrame.drawString(String(errorno + 1) + ".", xcoor, ycoor + errorno * 50);
        PageFrame.setTextColor(TFT_WHITE);
        PageFrame.drawString(AlarmArray[i], xcoor + 30, ycoor + errorno * 50);
      }
      else {
        PageFrame.fillRoundRect(xcoor + 220, ycoor + (errorno - 5) * 50, 220, 30, 5, TOLGA_RED);
        PageFrame.setTextColor(TOLGA_BLUE);
        PageFrame.drawString(String(errorno + 1) + ".", xcoor + 240, ycoor + (errorno - 5) * 50);
        PageFrame.setTextColor(TFT_WHITE);
        PageFrame.drawString(AlarmArray[i], xcoor + 270, ycoor + (errorno - 5) * 50);

      }
      errorno++;
    }
  }
  PageFrame.pushSprite(0, 95);
  PageFrame.deleteSprite();
}


void drawEnergies(float ChargeEnergy, float DischargeEnergy, bool Notification) {

  Serial.println(Notification);






  float RTE;
  if (ChargeEnergy != 0) {
    RTE = (DischargeEnergy / ChargeEnergy) * 100;
  }
  else {
    RTE = 0;
  }

  PageFrame.createSprite(480, 225);
  PageFrame.fillSprite(TFT_BLACK);
  //    Icon1.createSprite(48, 48);
  //    Icon1.setSwapBytes(true);
  //    Icon1.pushImage(0, 0, 48, 48, energyCD);


  //Charge Energy
  PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
  Icon8.createSprite(48, 48);
  Icon8.setSwapBytes(true);
  Icon8.pushImage(0, 0, 48, 48, maxIcon);
  Icon8.pushToSprite(&PageFrame, 15, 15, TFT_BLACK);
  PageFrame.drawString("Charge ", 70, 15, 2);
  PageFrame.drawString("Energy ", 72, 30, 2);
  PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  PageFrame.drawString(String(ChargeEnergy, 3) + "kW", 72, 50, 2);

  //Dsicharge Energy
  PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
  Icon9.createSprite(48, 48);
  Icon9.setSwapBytes(true);
  Icon9.pushImage(0, 0, 48, 48, voltage);
  Icon9.pushToSprite(&PageFrame, 340, 20, TFT_BLACK);
  PageFrame.drawString("Discharge ", 395, 20, 2);
  PageFrame.drawString("Energy ", 397, 35, 2);
  PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  PageFrame.drawString(String(DischargeEnergy, 3) + "kW", 397, 55, 2);


  //CycleNumber
  PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
  Icon10.createSprite(48, 48);
  Icon10.setSwapBytes(true);
  Icon10.pushImage(0, 0, 48, 48, minIcon);
  Icon10.pushToSprite(&PageFrame, 15, 90, TFT_BLACK);
  PageFrame.drawString("Cycle ", 70, 90, 2);
  PageFrame.drawString("Number ", 72, 105, 2);
  PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  PageFrame.drawString(String(ChargeEnergy / 10, 1) + "", 72, 125, 2);


  //Round Trip Efficincy
  PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
  Icon11.createSprite(48, 48);
  Icon11.setSwapBytes(true);
  Icon11.pushImage(0, 0, 48, 48, current);
  Icon11.pushToSprite(&PageFrame, 340, 90, TFT_BLACK);
  PageFrame.drawString("Round T. ", 395, 90, 2);
  PageFrame.drawString("Efficiency", 397, 105, 2);
  PageFrame.setTextColor(TOLGA_YELLOW, TFT_BLACK);
  PageFrame.drawString(String(RTE, 1) + "%", 397, 125, 2);

  PageFrame.fillRoundRect(230, 35, 20, 20, 5, TFT_LIGHTGREY);
  PageFrame.fillRoundRect(200, 40, 80, 100, 10, TFT_DARKGREY);


  float a = BMS.SOC * 0.1;



  PageFrame.setTextColor(TFT_WHITE, TFT_BLACK);
  PageFrame.fillRoundRect(200, 140 - a, 80, a, 10, TOLGA_GREEN);
  PageFrame.drawString("Remain. Cap.", 200, 160, 2);
  PageFrame.setTextColor(TFT_YELLOW, TFT_BLACK);
  PageFrame.drawString(String(BMS.rem_cap * 0.001 * 48 * 0.001) + "kWh", 200, 180, 2);

  for (int i = 0; i < 10; i++) {
    PageFrame.drawLine(190, 40 + (i * 10), 195, 40 + (i * 10), TOLGA_BLUE);
  }
  PageFrame.pushSprite(0, 95);
  PageFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();
  Icon7.deleteSprite();
  Icon8.deleteSprite();
  Icon9.deleteSprite();
  Icon10.deleteSprite();
  Icon11.deleteSprite();
  Icon12.deleteSprite();
  Icon13.deleteSprite();
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
  NotificationFrame.deleteSprite();
  notificationCounter++;
  Serial.println("Notification counter:" + String(notificationCounter));

  if (notificationCounter >= 5) {
    notificationEnable = false;
    notificationCounter = 0;
  }

}


void drawAlarmFrame (bool numpad, bool Current, bool Cells, bool Rates, bool Balance, int pageNumber, bool Notification, bool NotificationStatus) {


  if (Notification) {
    PageFrame.deleteSprite();
    PageFrame.createSprite(480, 225);
    PageFrame.fillSprite(TFT_BLACK);
    drawNotification(PageFrame , NotificationStatus);
    PageFrame.pushSprite(0, 95);
  }

  else {


    if (numpad) {
      PageFrame.deleteSprite();
      PageFrame.createSprite(480, 225);
      PageFrame.fillSprite(TFT_BLACK);
      drawNumpad(PageFrame);
      PageFrame.pushSprite(0, 95);
    }

    else  {
      PageFrame.deleteSprite();
      PageFrame.createSprite(480, 175);
      PageFrame.fillSprite(TFT_BLACK);
      PageFrame.setFreeFont(&Orbitron_Light_32);
      PageFrame.setTextColor(TFT_WHITE);


      switch (pageNumber) {
        case 6:
          PageFrame.drawString("High Curr.", 42, 20);
          PageFrame.drawString("Discharge", 42, 50);
          PageFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(HCD, 1) + "A", 350, 30);

          PageFrame.setTextDatum(TL_DATUM);
          PageFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          PageFrame.setTextColor(TFT_WHITE);
          PageFrame.drawString("SET", 389, 70);


          PageFrame.drawString("High Curr.", 42, 100);
          PageFrame.drawString("Charge", 42, 130);
          PageFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(HCC, 1) + "A", 350, 110);
          PageFrame.setTextDatum(TL_DATUM);





          break;

        case 7:
          PageFrame.drawString("High Volt.", 42, 20 );
          PageFrame.drawString("Cell", 42, 50 );
          PageFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(HVC, 1) + "V", 350, 30);

          PageFrame.setTextDatum(TL_DATUM);
          PageFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          PageFrame.setTextColor(TFT_WHITE);
          PageFrame.drawString("SET", 389, 70);


          PageFrame.drawString("Low Volt.", 42, 100);
          PageFrame.drawString("Cell", 42, 130);
          PageFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(LVC, 1) + "V", 350, 110);


          break;

        case 8:

          PageFrame.drawString("Rated", 42, 20);
          PageFrame.drawString("Capacity", 42, 50);
          PageFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(RC, 1) + "W", 350, 30);

          PageFrame.setTextDatum(TL_DATUM);
          PageFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          PageFrame.setTextColor(TFT_WHITE);
          PageFrame.drawString("SET", 389, 70);

          PageFrame.drawString("Rated", 42, 100);
          PageFrame.drawString("Voltage", 42, 130);
          PageFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(RV, 1) + "V", 350, 110);



          break;

        case 9:

          PageFrame.drawString("Balance", 42, 20);
          PageFrame.drawString("Voltage", 42, 50);
          PageFrame.fillRoundRect(210, 18, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(BV , 1) + "V", 350, 30);

          PageFrame.setTextDatum(TL_DATUM);
          PageFrame.setTextColor(TFT_WHITE);
          PageFrame.drawString("Balance", 42, 100);
          PageFrame.drawString("DeltaV", 42, 130);
          PageFrame.fillRoundRect(210, 98, 150, 60, 5, TFT_BACKGROUND5);
          PageFrame.setTextColor(TOLGA_BLUE);
          PageFrame.setTextDatum(TR_DATUM);
          PageFrame.drawString(String(BD, 1) + "mV", 350, 110);

          PageFrame.setTextDatum(TL_DATUM);
          PageFrame.fillRoundRect(380, 18, 70, 140, 5, TFT_RED);
          PageFrame.setTextColor(TFT_WHITE);
          PageFrame.drawString("SET", 389, 70);


          break;

        default:

          break;
      }

      NotificationFrame.createSprite(480, 50);
      NotificationFrame.setTextDatum(BC_DATUM);
      NotificationFrame.setTextColor(TFT_WHITE);
      NotificationFrame.fillSprite(TFT_BLACK);
      NotificationFrame.setFreeFont(&Orbitron_Light_32);

      Icon1.createSprite(115, 44);
      Icon1.setSwapBytes(true);
      Icon1.pushImage(0, 0, 115, 44, tabMini);


      NotificationFrame.setTextColor(TFT_WHITE);

      if (Current) {
        Icon1.pushToSprite(&NotificationFrame, 2, 0, TFT_BLACK);
        NotificationFrame.setTextColor(TOLGA_BLUE_2);
        NotificationFrame.drawString("Curr.", 60, 44);
      }
      else {
        Icon1.pushToSprite(&NotificationFrame, 2, 5, TFT_BLACK);
        NotificationFrame.setTextColor(TFT_WHITE);
        NotificationFrame.drawString("Curr.", 60, 44);
      }

      /////////////////////////////////////////
      if (Cells) {
        Icon1.pushToSprite(&NotificationFrame, 122, 0, TFT_BLACK);
        NotificationFrame.setTextColor(TOLGA_BLUE_2);
        NotificationFrame.drawString("Cells", 180, 44);

      }
      else {
        Icon1.pushToSprite(&NotificationFrame, 122, 5, TFT_BLACK);
        NotificationFrame.setTextColor(TFT_WHITE);
        NotificationFrame.drawString("Cells", 180, 44);
      }

      ////////////////////////////////////////
      if (Rates) {
        Icon1.pushToSprite(&NotificationFrame, 242, 0, TFT_BLACK);
        NotificationFrame.setTextColor(TOLGA_BLUE_2);
        NotificationFrame.drawString("Rates", 300, 44);
      }
      else {
        Icon1.pushToSprite(&NotificationFrame, 242, 5, TFT_BLACK);
        NotificationFrame.setTextColor(TFT_WHITE);
        NotificationFrame.drawString("Rates", 300, 44);;
      }

      ////////////////////////////////////////
      if (Balance) {

        Icon1.pushToSprite(&NotificationFrame, 362, 0, TFT_BLACK);
        NotificationFrame.setTextColor(TOLGA_BLUE_2);
        NotificationFrame.drawString("Blnc.", 420, 44);
      }
      else {

        Icon1.pushToSprite(&NotificationFrame, 362, 5, TFT_BLACK);
        NotificationFrame.setTextColor(TFT_WHITE);
        NotificationFrame.drawString("Blnc.", 420, 44);
      }

      NotificationFrame.setTextDatum(TL_DATUM);

      NotificationFrame.pushSprite(0, 270);
      PageFrame.pushSprite(0, 95);

      PageFrame.deleteSprite();
      NotificationFrame.deleteSprite();
      Icon1.deleteSprite();
      Icon2.deleteSprite();
      Icon3.deleteSprite();
      Icon4.deleteSprite();
      Icon5.deleteSprite();
      Icon6.deleteSprite();
      Icon7.deleteSprite();
      Icon8.deleteSprite();
      Icon9.deleteSprite();
      Icon10.deleteSprite();
      Icon11.deleteSprite();
      Icon12.deleteSprite();
      Icon13.deleteSprite();
      PageFrame.setTextDatum(TL_DATUM);
    }
  }
}



void drawNumpad(TFT_eSprite & tft) {

  ContentFrame.createSprite(320, 225);
  ContentFrame.setTextDatum(TL_DATUM);
  ContentFrame.setTextColor(TFT_WHITE);
  ContentFrame.fillSprite(TFT_BLACK);
  ContentFrame.setFreeFont(&Orbitron_Light_32);

  ContentFrame.fillRoundRect(30, 7, 262, 40, 5, TFT_WHITE);
  ContentFrame.drawRoundRect(30, 7, 262, 40, 5, TOLGA_BLUE_2);
  ContentFrame.setTextColor(TFT_RED);
  ContentFrame.setTextDatum(TR_DATUM);
  ContentFrame.drawString(numpadValue, 270, 10);
  ContentFrame.setTextDatum(TL_DATUM);

  ContentFrame.setTextDatum(MC_DATUM);
  ContentFrame.setTextColor(TFT_WHITE);

  ContentFrame.fillRoundRect(30, 51, 60, 40, 5, TFT_BACKGROUND2); //7
  ContentFrame.drawRoundRect(30, 51, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("7", 55, 67);

  ContentFrame.fillRoundRect(92, 51, 60, 40, 5, TFT_BACKGROUND2); //8
  ContentFrame.drawRoundRect(92, 51, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("8", 117, 67);

  ContentFrame.fillRoundRect(154, 51, 60, 40, 5, TFT_BACKGROUND2); //9
  ContentFrame.drawRoundRect(154, 51, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("9", 177, 67);

  ContentFrame.fillRoundRect(216, 51, 75, 40, 5, TFT_BACKGROUND2); //del
  ContentFrame.drawRoundRect(216, 51, 75, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("DEL", 256, 67);

  //=========================================================================
  ContentFrame.fillRoundRect(30, 94, 60, 40, 5, TFT_BACKGROUND2); //4
  ContentFrame.drawRoundRect(30, 94, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("4", 55, 110);

  ContentFrame.fillRoundRect(92, 94, 60, 40, 5, TFT_BACKGROUND2); //5
  ContentFrame.drawRoundRect(92, 94, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("5", 117, 110);

  ContentFrame.fillRoundRect(154, 94, 60, 40, 5, TFT_BACKGROUND2); //6
  ContentFrame.drawRoundRect(154, 94, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("6", 179, 110);

  ContentFrame.fillRoundRect(216, 94, 75, 40, 5, TFT_BACKGROUND2); //
  ContentFrame.drawRoundRect(216, 94, 75, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("CLR", 254, 110);

  //=========================================================================
  ContentFrame.fillRoundRect(30, 137, 60, 40, 5, TFT_BACKGROUND2); //1
  ContentFrame.drawRoundRect(30, 137, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("1", 55, 153);

  ContentFrame.fillRoundRect(92, 137, 60, 40, 5, TFT_BACKGROUND2); //1
  ContentFrame.drawRoundRect(92, 137, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("2", 117, 153);

  ContentFrame.fillRoundRect(154, 137, 60, 40, 5, TFT_BACKGROUND2); //1
  ContentFrame.drawRoundRect(154, 137, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("3", 179, 153);

  ContentFrame.fillRoundRect(216, 137, 75, 40, 5, TFT_BACKGROUND2); //
  ContentFrame.drawRoundRect(216, 137, 75, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("BCK", 254, 153);

  //=========================================================================


  ContentFrame.fillRoundRect(30, 180, 122, 40, 5, TFT_BACKGROUND2); //1
  ContentFrame.drawRoundRect(30, 180, 122, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("0", 90, 196);
  ContentFrame.fillRoundRect(154, 180, 60, 40, 5, TFT_BACKGROUND2); //1
  ContentFrame.drawRoundRect(154, 180, 60, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString(".", 179, 196);
  ContentFrame.fillRoundRect(216, 180, 75, 40, 5, TFT_BACKGROUND2); //
  ContentFrame.drawRoundRect(216, 180, 75, 40, 5, TOLGA_BLUE_2);
  ContentFrame.drawString("OK", 254, 196);
  ContentFrame.pushToSprite(&tft, 80, 0, TFT_BLACK);
  ContentFrame.setTextDatum(TL_DATUM);
  ContentFrame.deleteSprite();
}


void drawGraphTab(bool Voltage, bool Current, bool Temp ) {


  NotificationFrame.createSprite(480, 45);
  NotificationFrame.setTextDatum(BC_DATUM);
  NotificationFrame.setTextColor(TFT_WHITE);
  NotificationFrame.fillSprite(TFT_BLACK);
  NotificationFrame.setFreeFont(&Orbitron_Light_32);

  Icon7.createSprite(159, 45);
  Icon7.setSwapBytes(true);
  Icon7.pushImage(0, 0, 159, 45, tab);


  Icon7.pushToSprite(&NotificationFrame, 0, 0, TFT_BLACK);
  Icon7.pushToSprite(&NotificationFrame, 164, 0, TFT_BLACK);
  Icon7.pushToSprite(&NotificationFrame, 328, 0, TFT_BLACK);

  if (Voltage) {
    NotificationFrame.setTextColor(TOLGA_BLUE_2);
    NotificationFrame.drawString("Voltage", 80, 40);
  }
  else {
    NotificationFrame.setTextColor(TFT_WHITE);
    NotificationFrame.drawString("Voltage", 80, 40);
  }

  if (Current) {
    NotificationFrame.setTextColor(TOLGA_BLUE_2);
    NotificationFrame.drawString("Current", 240, 40);
  }
  else {
    NotificationFrame.setTextColor(TFT_WHITE);
    NotificationFrame.drawString("Current", 240, 40);
  }

  if (Temp) {
    NotificationFrame.setTextColor(TOLGA_BLUE_2);
    NotificationFrame.drawString("Temp.", 405, 40);
  }
  else {
    NotificationFrame.setTextColor(TFT_WHITE);
    NotificationFrame.drawString("Temp", 405, 40);
  }


  NotificationFrame.setTextDatum(TL_DATUM);
  NotificationFrame.pushSprite(0, 275);

  NotificationFrame.deleteSprite();
  PageFrame.deleteSprite();
  NotificationFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();
  Icon7.deleteSprite();
  Icon8.deleteSprite();
  Icon9.deleteSprite();
  Icon10.deleteSprite();
  Icon11.deleteSprite();
  Icon12.deleteSprite();
  Icon13.deleteSprite();

}



void drawLandingPage() {

  //SETUP SCREEN------------------------------


  //Serialnumber
  // SerialNumber
  ContentFrame.createSprite(150, 50);
  ContentFrame.fillSprite(TFT_BLACK);

  ContentFrame.setTextColor(TOLGA_YELLOW);
  ContentFrame.drawString("Serial No", 10, 30, 1);

  ContentFrame.setTextColor(TFT_WHITE);
  ContentFrame.drawString(SerialNumber, 10, 40, 1);

  ContentFrame.setTextColor(TOLGA_YELLOW);
  ContentFrame.drawString("Firm. Version", 10, 0, 1);

  ContentFrame.setTextColor(TFT_WHITE);
  ContentFrame.drawString(FirmwareVer, 10, 10, 1);

  Icon2.createSprite(280, 105);
  Icon2.setSwapBytes(true);
  Icon2.pushImage(0, 0, 280, 105, encap2);


  Icon4.createSprite(58, 95);
  Icon4.setSwapBytes(true);
  if (WDNextRestart) {
    Icon4.drawString("Enabled", 0, 50, 2);
    Icon4.pushImage(0, 0, 48, 48, wifidirect);

  }
  else {
    Icon4.drawString("Disabled", 0, 50, 2);
    Icon4.pushImage(0, 0, 48, 48, wifidirectdis);

  }

  Icon5.createSprite(58, 95);
  Icon5.setSwapBytes(true);
  if (FBNextRestart) {
    Icon5.drawString("Enabled", 0, 50, 2);
    Icon5.pushImage(0, 0, 48, 48, firebasedirect);


  }
  else {
    Icon5.drawString("Disabled", 0, 50, 2);
    Icon5.pushImage(0, 0, 48, 48, firebasedirectdis);
  }

  Icon6.createSprite(58, 95);
  Icon6.setSwapBytes(true);
  if (BTNextRestart) {
    Icon6.drawString("Enabled", 0, 50, 2);
    Icon6.pushImage(0, 0, 48, 48, bluetoothdirect);
  }
  else {
    Icon6.drawString("Disabled", 0, 50, 2);
    Icon6.pushImage(0, 0, 48, 48, bluetoothdirectdis);
  }

  PageFrame.createSprite(200, 120);
  PageFrame.fillSprite(TFT_BLACK);
  PageFrame.setFreeFont(&Orbitron_Light_32);
  PageFrame.setTextColor(TFT_WHITE);


  for (int i = 0; i < 100; i++) {

    PageFrame.fillSprite(TFT_BLACK);
    PageFrame.setTextColor(TFT_WHITE);
    ContentFrame.drawString("SR:", 0, 80, 2);



    PageFrame.drawString(" Initialization:", 0, 10);
    PageFrame.setTextColor(TOLGA_YELLOW);
    PageFrame.drawString(String(i) + "%", 70, 45);

    if (updateIcon) {
      PageFrame.drawString("New firmware update", 30, 85, 2);
      PageFrame.drawString( "is ready", 65, 100, 2);
    }


    if (i == 99) {

    }
    else {
      //Icon1.pushRotated(i * 3);
      Icon2.pushSprite(110, 10, TFT_BLACK);
      ContentFrame.pushSprite(0, 250, TFT_BLACK);
      PageFrame.pushSprite(150, 110);
      Icon4.pushSprite(170, 240, TFT_BLACK);
      Icon5.pushSprite(240, 240, TFT_BLACK);
      Icon6.pushSprite(310, 240, TFT_BLACK);


      delay(10);

    }
  }
  PageFrame.deleteSprite();
  ContentFrame.deleteSprite();
  Icon1.deleteSprite();
  Icon2.deleteSprite();
  Icon3.deleteSprite();
  Icon4.deleteSprite();
  Icon5.deleteSprite();
  Icon6.deleteSprite();
  Icon7.deleteSprite();
  Icon8.deleteSprite();
  Icon9.deleteSprite();
  Icon10.deleteSprite();
  Icon11.deleteSprite();
  Icon12.deleteSprite();
  Icon13.deleteSprite();
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
    tft.drawString(String(buf) + "%", x, y + 10); // Value in middle
  }
  else {
    tft.setTextColor(TOLGA_YELLOW);
    tft.setTextPadding(3 * 14); // Allow for 3 digits each 14 pixels wide
    tft.drawString(String(buf) + "%", x, y + 10 ); // Value in middle
  }

  tft.setTextColor(TFT_WHITE);
  tft.drawString(units, x, y - 20); // Units display



  tft.setTextColor(TFT_WHITE);
  tft.drawString(String(BMS.rem_cap * 0.001 * 0.001 * 48) + "kWh", x + 3, y + 58 , 2); // Units
  return x + r;
}



void drawCell() {


  for (int i = 0; i < 16; i++) {
    AllCells[i] = BMS_equ.cell_voltage_equ[i] * 0.001;
  }

  PageFrame.createSprite(480, 225);

  for (int t = 0; t < 8; t++) {
    drawCellMainUp(50 + 57 * t, 40 , t);
  }


  PageFrame.pushSprite(0, 95);
  PageFrame.deleteSprite();
}


void drawCellMainUp(int startXcoor, int startYcoor, int CellNumber) {
  PageFrame.setTextDatum(MC_DATUM);

  float envelope = random(0, 10) * 0.1;
  float anode = AllCells[CellNumber];
  float cathode = AllCells[CellNumber + 1];

  PageFrame.fillRoundRect(startXcoor , startYcoor , 10, 120, 2, TFT_DARKGREY);
  // CellFrame.fillRoundRect(startXcoor , startYcoor , 10, 50, 2, TOLGA_GREEN);



  PageFrame.fillRoundRect(startXcoor , 40 , 10, ((anode - 2.7)) * 50, 2, TOLGA_GREEN);
  PageFrame.fillRoundRect(startXcoor , 40 + ((anode - 2.7)) * 50 , 10, 2, 2, TFT_WHITE);

  PageFrame.fillRoundRect(startXcoor , 160 - (cathode - 2.7) * 50 , 10, ((cathode - 2.7)) * 50, 2, TOLGA_YELLOW);
  PageFrame.fillRoundRect(startXcoor , 160 - (cathode - 2.7) * 50 , 10, 2, 2, TFT_WHITE);

  PageFrame.setTextFont(GLCD);
  PageFrame.setTextColor(TOLGA_GREEN);
  PageFrame.drawString("Anode", startXcoor - 20, startYcoor + 10);

  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.drawString(String(anode, 3), startXcoor - 20, startYcoor + 23);

  PageFrame.setTextColor(TFT_DARKGREY);
  PageFrame.drawString("Env.", startXcoor - 20, startYcoor + 55);

  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.drawString(String(envelope), startXcoor - 20, startYcoor + 68);

  PageFrame.setTextColor(TOLGA_YELLOW);
  PageFrame.drawString("Cath", startXcoor - 20, startYcoor + 95);

  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.drawString(String(cathode, 3), startXcoor - 20, startYcoor + 108);

  PageFrame.setTextColor(TOLGA_BLUE_2);
  PageFrame.drawString("Cell#" + String(CellNumber + 1), startXcoor , startYcoor + 140);

  PageFrame.setTextColor(TFT_WHITE);
  PageFrame.drawString(String(anode + cathode, 3) + "V", startXcoor , startYcoor + 153);

}


void drawUpload() {
  NotificationFrame.createSprite(480, 225);
  NotificationFrame.setTextDatum(TL_DATUM);
  NotificationFrame.setTextColor(TFT_WHITE);
  NotificationFrame.fillSprite(TFT_BLACK);
  NotificationFrame.setFreeFont(&Orbitron_Light_32);
  NotificationFrame.fillRoundRect(80, 20, 320, 120, 5, TFT_BACKGROUND); //8
  NotificationFrame.setTextDatum(MC_DATUM);
  NotificationFrame.drawString("Firmware is Updating,", 240, 50);
  NotificationFrame.drawString("Please Wait....", 240, 90);
  NotificationFrame.setTextDatum(TL_DATUM);
  NotificationFrame.pushSprite(0, 95);
}


void drawBarcode() {


  barcode.createSprite(480, 225);
  barcode.setTextDatum(MC_DATUM);
  barcode.fillSprite(TFT_WHITE);
  barcode.setTextColor(TFT_BLACK);
  barcode.drawString("QR Code for Monitoring & Configuring", 240, 30, 4);
  String link = "http://" + IP + "/";
  Display_QRcode(180, 60, 3, 6, 3, link.c_str());
  barcode.pushSprite(0, 95);


}






//SD CARD FUNCTIONS
void rm(File dir, String tempPath) {
  while (true) {
    File entry =  dir.openNextFile();
    String localPath;

    Serial.println("");
    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if ( SD.rmdir( folderBuf ) )
        {
          Serial.print("Deleted folder ");
          Serial.println(folderBuf);
          FolderDeleteCount++;
        }
        else
        {
          Serial.print("Unable to delete folder ");
          Serial.println(folderBuf);
          FailCount++;
        }
      }
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if ( SD.remove( charBuf ) )
        {
          Serial.print("Deleted ");
          Serial.println(localPath);
          DeletedCount++;
        }
        else
        {
          Serial.print("Failed to delete ");
          Serial.println(localPath);
          FailCount++;
        }
      }
    }
    else {
      // break out of recursion
      break;
    }
  }
}




//OPERATIONAL FUNCTIONS/////////////////////////////

//#########################################################################################
void Display_QRcode(int offset_x, int offset_y, int element_size, int QRsize, int ECC_Mode, const char* Message) {
  // QRcode capacity examples Size-12  65 x 65 LOW      883 535 367
  //                                           MEDIUM   691 419 287
  //                                           QUARTILE 489 296 203
  //                                           HIGH     374 227 155
  uint8_t qrcodeData[qrcode_getBufferSize(QRsize)];
  //ECC_LOW, ECC_MEDIUM, ECC_QUARTILE and ECC_HIGH. Higher levels of error correction sacrifice data capacity, but ensure damaged codes remain readable.
  if (ECC_Mode % 4 == 0) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_LOW, Message);
  if (ECC_Mode % 4 == 1) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_MEDIUM, Message);
  if (ECC_Mode % 4 == 2) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_QUARTILE, Message);
  if (ECC_Mode % 4 == 3) qrcode_initText(&qrcode, qrcodeData, QRsize, ECC_HIGH, Message);
  for (int y = 0; y < qrcode.size; y++) {
    for (int x = 0; x < qrcode.size; x++) {
      if (qrcode_getModule(&qrcode, x, y)) {
        //tft.setColor(EPD_BLACK);
        barcode.fillRect(x * element_size + offset_x, y * element_size + offset_y, element_size, element_size, TFT_BLACK);
      }
      else
      {
        // tft.setColor(EPD_WHITE);
        barcode.fillRect(x * element_size + offset_x, y * element_size + offset_y, element_size, element_size, TFT_WHITE);
        //  tft.fillRect(0, 0, 230, 32, TFT_RED);
      }
    }
  }
}















void firmwareUpdate(void) {

  EEPROMbusy = true;
  preferences.begin("my-app", false);
  restartCounter = 0;
  preferences.putInt("RC", restartCounter);
  preferences.end();
  EEPROMbusy = false;
  delay(100);

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

String ConvBinUnits(int bytes, int resolution) {
  if      (bytes < 1024)                 {
    return String(bytes) + " B";
  }
  else if (bytes < 1024 * 1024)          {
    return String((bytes / 1024.0), resolution) + " KB";
  }
  else if (bytes < (1024 * 1024 * 1024)) {
    return String((bytes / 1024.0 / 1024.0), resolution) + " MB";
  }
  else return "";
}

void HomePage() {
  SendHTML_Header();
  navbar();
  webpage += F("<body class='hold-transition sidebar-mini'>");
  webpage += F("<div class='wrapper'>");
  Home();
  append_page_footer();

  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download"))
      SD_file_download(server.arg(0));
    Serial.print("argumnent:");
    Serial.println(server.arg(0));
  }
  else SelectInput("File Download", "Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SD_file_download(String filename) {
  if (control_sd) {
    File download = SD.open("/" + filename);
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSDNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-vControl", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String heading2, String command, String arg_calling_name) {
  SendHTML_Header();
  navbar();
  webpage += F("<body class='hold-transition sidebar-mini'>");


  String Fname1, Fname2;


  String filename = "";
  int index = 0;
  Directory(); // Get a list of files on the FS

  webpage += " <div class='content-wrapper'>";
  webpage += "     <div class='content-header'>";
  webpage += "    </div>";
  webpage += "  <section class='content'>";
  webpage += "  <div class='container-fluid'>";

  webpage += "<table class='table table-striped table-hover'>";
  webpage += "<tr><th>Type</th><th>File Name</th><th>File Size</th><th class='sp'></th><th>Type</th><th>File Name</th><th>File Size</th></tr>";
  while (index < numfiles) {
    Fname1 = Filenames[index].filename;
    Fname2 = Filenames[index + 1].filename;
    webpage += "<tr>";
    webpage += "<td style = 'width:5%'>" + Filenames[index].ftype + "</td><td style = 'width:25%'>" + Fname1 + "</td><td style = 'width:10%'>" + Filenames[index].fsize + "</td>";
    webpage += "<td class='sp'></td>";
    if (index < numfiles - 1) {
      webpage += "<td style = 'width:5%'>" + Filenames[index + 1].ftype + "</td><td style = 'width:25%'>" + Fname2 + "</td><td style = 'width:10%'>" + Filenames[index + 1].fsize + "</td>";
    }
    webpage += "</tr>";
    index = index + 2;
  }
  webpage += "</table>";
  //  webpage += "  </div>";
  //  webpage += F("</div>");


  /////////////////

  //  webpage += F("<div class='content-wrapper'>");
  //  webpage += F("<div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("<h3 class='rcorners_m'>");
  webpage += heading1 + "</h3><br>";
  webpage += F("<h3>");
  webpage += heading2 + "</h3>";
  webpage += F("<FORM action='/");
  webpage += command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += F("<input type='text' name='");
  webpage += arg_calling_name;
  webpage += F("' value=''><br>");
  webpage += F("<type='submit' name='");
  webpage += arg_calling_name;
  webpage += F("' value=''><br><br>");
  webpage += F("</div>");
  webpage += F("</div>");


  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSDNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No SD Card present</h3>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>");
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//#############################################################################################
void Directory() {
  numfiles  = 0; // Reset number of FS files counter
  File root = SD.open("/");
  if (root) {
    Serial.println("ROOT AVAILABLE");

    printDirectory(root, 0);
  }
}

//#############################################################################################
void Dir() {
  SendHTML_Header();
  navbar();
  webpage += F("<body class='hold-transition sidebar-mini'>");
  webpage += F("<div class='wrapper'>");


  String Fname1, Fname2;
  int index = 0;
  Directory(); // Get a list of the current files on the FS

  webpage += F("<div class='content-wrapper'>");
  webpage += "<h3>Filing System Content</h3><br>";
  if (numfiles > 0) {

    webpage += F("<div class='px-4 py-4'>");
    webpage += "<table class='table table-striped table-hover'>";
    webpage += "<tr><th>Type</th><th>File Name</th><th>File Size</th><th class='sp'></th><th>Type</th><th>File Name</th><th>File Size</th></tr>";
    while (index < numfiles) {
      Fname1 = Filenames[index].filename;
      Fname2 = Filenames[index + 1].filename;
      webpage += "<tr>";
      webpage += "<td style = 'width:5%'>" + Filenames[index].ftype + "</td><td style = 'width:25%'>" + Fname1 + "</td><td style = 'width:10%'>" + Filenames[index].fsize + "</td>";
      webpage += "<td class='sp'></td>";
      if (index < numfiles - 1) {
        webpage += "<td style = 'width:5%'>" + Filenames[index + 1].ftype + "</td><td style = 'width:25%'>" + Fname2 + "</td><td style = 'width:10%'>" + Filenames[index + 1].fsize + "</td>";
      }
      webpage += "</tr>";
      index = index + 2;
    }
    webpage += "</table>";
    webpage += F("</div>");
    webpage += "<p style='background-color:yellow;'><b>" + MessageLine + "</b></p>";
    MessageLine = "";
  }
  else
  {
    webpage += "<h2>No Files Found</h2>";
  }
  webpage += F("</div>");

  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent

}

void printDirectory(File dir, int numTabs) {
  dir.seek(0);
  numfiles = 0;
  while (true) {

    File entry =  dir.openNextFile();

    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);

      if (String(entry.name()).indexOf("System") == -1 && String(entry.name()).indexOf("BMS Data") == -1) {
        Serial.println(String(entry.name()));
        Filenames[numfiles].filename = (String(entry.name()).startsWith("/") ? String(entry.name()).substring(1) : entry.name());
        Filenames[numfiles].ftype    = (entry.isDirectory() ? "Dir" : "File");
        Filenames[numfiles].fsize    = ConvBinUnits(entry.size(), 1);
        numfiles++;
      }
    }
    entry.close();
  }
}

void Home() {


  webpage += F("<div class='content-wrapper'>");
  webpage += F("<div class='content-header'></div>");
  webpage += F(     "<section class='content'>");
  webpage += F( "   <div class='container-fluid'>");

  webpage += F("     <div class='row'>");
  webpage += F("   <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F(" <div class='info-box mb-3'>");
  webpage += F( "     <span class='info-box-icon bg-info elevation-1'>");
  webpage += F( "      <i class='fas fa-battery-three-quarters'></i></span>");
  webpage += F("   <div class='info-box-content'>");
  webpage += F("     <h6 class='info-box-text'> Module Voltage</h6>");
  webpage += "     <h7 class='info-box-number' id='voltage'>" + String(BMS.sum_voltage * 0.1) + "V</h7>";
  webpage += F("  </div>");
  webpage += F("</div>");
  webpage += F("  </div>");

  webpage += F(" <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("        <span class='info-box-icon bg-teal elevation-1'>");
  webpage += F( "        <i class='fas fa-bolt'></i></span>");

  webpage += F("     <div class='info-box-content'>");
  webpage += F("        <h6 class='info-box-text'>Current</h6>");
  webpage += "<h7 class='info-box-number' id='current'>" + String((BMS.current - 30000) * 0.1) + "A</h7>";
  webpage += F("    </div>");
  webpage += F("     </div>");
  webpage += F("  </div>");


  webpage += F(" <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("  <div class='info-box mb-3'>");
  webpage += F("      <span class='info-box-icon bg-gray elevation-1'>");
  webpage += F("          <i class='fas fa-thermometer-half'></i></span>");
  webpage += F("      <div class='info-box-content'>");
  webpage += F("          <h6 class='info-box-text'>Module Temperature</h6>");
  webpage += "          <h7 class='info-box-number' id='temp'>" + String(BMS.max_cell_temp - 40) + "C</h7>";
  webpage += F("       </div>");
  webpage += F("      </div>");
  webpage += F("    </div>");


  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("    <div class='info-box mb-3'>");
  webpage += F("        <span class='info-box-icon bg-danger elevation-1'>");
  webpage += F("          <i class='fas fa-lightbulb'></i>");
  webpage += F("       </span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'>Discharge Energy</h6>");
  webpage += "           <h7 class='info-box-number' id='dischargeEnergy'>" + String(DischargeEnergy, 4) + "kWh</h7>";
  webpage += F("        </div>");
  webpage += F("     </div>");
  webpage += F("     </div>");
  webpage += F("</div>");
  //row ned



  webpage += F("   <div class='row'>");
  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-indigo elevation-1'>");
  webpage += F("           <i class='fa fa-tachometer' aria-hidden='true'></i></span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Module SOC</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS.SOC * 0.1) + "%</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");

  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-maroon elevation-1'>");
  webpage += F("           <i class='fa fa-plus-square-o' aria-hidden='true'></i></span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Maximum Cell</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS_equ.max_cell_volt_equ * 0.001) + "V</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");


  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-olive elevation-1'>");
  webpage += F("           <i class='fa fa-minus-square-o' aria-hidden='true'></i></span>");
  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Minimum Cell</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS_equ.min_cell_volt_equ * 0.001) + "V</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");

  webpage += F("  <div class='col-12 col-sm-6 col-md-3'>");
  webpage += F("     <div class='info-box mb-3'>");
  webpage += F("       <span class='info-box-icon bg-cyan elevation-1'>");
  webpage += F("           <i class='fa-solid fa-diagram-next'></i></span>");

  webpage += F("       <div class='info-box-content'>");
  webpage += F("           <h6 class='info-box-text'> Difference Cell</h6>");
  webpage += "           <h7 class='info-box-number' id='voltage'>" + String(BMS_equ.max_cell_volt_equ * 0.001 - BMS_equ.min_cell_volt_equ * 0.001, 3) + "V</h7>";
  webpage += F("       </div>");
  webpage += F("   </div>");
  webpage += F("  </div>");
  webpage += F("</div>");
  //row e

  webpage += F("</div>");
  webpage += F(" </section>");
  webpage += F(" </div>");
  webpage += F(" </div>");
}

void navbar() {
  webpage += F("<nav class='main-header navbar navbar-expand navbar-light'>");
  webpage += F("<ul class='navbar-nav'>");
  webpage += F("<li class='nav-item'>");
  webpage += F("<a class='nav-link' data-widget='pushmenu' href='#' role='button'><i class='fas fa-bars'></i></a>");
  webpage += F("</li>");
  webpage += F("</ul>");
  //  webpage += F("<ul class='navbar-nav ml-auto'>");
  //  webpage += F("<div class='container'>");
  //  webpage += F("<i class='fa-solid fa-barcode'></i>");
  //  webpage += "<h7 class='nav-link' id='site_name'>" + SerialNumber + "</h7>";
  //  webpage += F("</div>");
  //  webpage += F("<li class='nav-item'>");
  //  webpage += F("<a class='nav-link' data-widget='fullscreen' href='#' role='button'>");
  //  webpage += F("<i class='fas fa-expand-arrows-alt'></i>");
  //  webpage += F("</a>");
  //  webpage += F("</li>");
  //  webpage += F("</ul>");


  webpage += F(" <ul class='navbar-nav ml-auto'>");
  webpage += F("   <div class='container'>");
  webpage += F("       <i class='fa-solid fa-barcode'></i>");
  webpage += "        <h7 class='nav-link' id='site_name'>" + SerialNumber + "</h7>";
  webpage += F("     </div>");
  webpage += F("     </ul>");

  webpage += F("    <ul class='navbar-nav ml-auto'>");
  webpage += F("        <div class='container'>");
  webpage += F("         <i class='fa-regular fa-clock'></i>");
  webpage += "         <h7 class='nav-link' id='site_name'>" + TimeString + "</h7>";
  webpage += F("      </div>");
  webpage += F("     </ul>");


  webpage += F("    <ul class='navbar-nav ml-auto'>");
  webpage += F("        <div class='container'>");
  webpage += F("         <i class='fa-solid fa-calendar-days'></i>");
  webpage += "         <h7 class='nav-link' id='site_name'>" + DateString + "</h7>";
  webpage += F("      </div>");
  webpage += F("     </ul>");










  webpage += F("</nav>");
  webpage += F("<aside class='main-sidebar sidebar-dark-primary elevation-4'>");
  webpage += F("<div class='brand-link'>");
  webpage += F("<img src='https://kilowattlabs.com/wp-content/themes/KiloWatt-Labs/Technical_Data_Sheets/Low/en.png' alt='CentauriVIEW Logo' class='brand-image elevation' style='opacity: .8'>");
  webpage += F("<span class='brand-text '> ENCONNECT </span>");
  webpage += F("</div>");
  webpage += F("<div class='sidebar'>");
  webpage += F("<nav class='mt-2'>");
  webpage += F("<ul class='nav nav-pills nav-sidebar flex-column' data-widget='treeview' role='menu'");
  webpage += F("data-accordion='false' id='bars'>");
  webpage += F("<li class='nav-item'>");
  webpage += F("<a href=' / ' class='nav-link '>");
  webpage += F("<i class='nav-icon fas fa-tachometer-alt'></i>");
  webpage += F("<p>");
  webpage += F("Dashboard");
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");
  webpage += F("<li class='nav-item'>");
  webpage += F("<a href='/download' class='nav-link'>");
  webpage += F("<i class='nav-icon fas fa-table'></i>");
  webpage += F("<p>");
  webpage += F("Download");
  webpage += F("</p>");
  webpage += F("</a>");
  webpage += F("</li>");
  webpage += F(" </nav>");
  webpage += F(" </div>");
  webpage += F("</aside>");
}
