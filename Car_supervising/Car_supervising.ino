#include <Arduino.h>
#include "CanIsoTp.h"
#include "DoanCfg.h"
#include <stdlib.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <time.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define FIREBASE_HOST "https://car-supervising-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTHORIZATION_KEY "cm9cRGjg2D1IiHRnTWDWK7daMlDbLvLkwIRpPuV4"

#define WIFI_SSID "tenwifi"
#define WIFI_PASSWORD "passwifi"

// #define MAX_ERRORS pdu_data->data[PID_POSITION]
#define MAX_ERRORS 5                            // Số lượng tối đa các mã lỗi có thể lưu trữ0
#define FIREBASE_ERROR_CODES_PATH "/MODE3_DATA" // Đường dẫn trên Firebase để lưu trữ các mã lỗi

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 25200, 60000);

static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

char Time[12];
char Date[12];

char errorCodes[MAX_ERRORS][100]; // Mảng lưu trữ các chuỗi hexa_String
int errorCodeIndex = 0;           // Chỉ số hiện tại trong mảng errorCodes

unsigned long long timecount;
uint8_t buff[128]; // Buffer used in the pdu.data.
int count = 1;

pdu_t *cantp_pdu = (pdu_t *)malloc(sizeof(pdu_t));
pdu_t *cantp_pdu_rx = (pdu_t *)malloc(sizeof(pdu_t));

CanIsoTp cantp; // CAN IsoTp protocol to send and receive the pdus

FirebaseData firebaseData;

TaskHandle_t requestDataTask;
TaskHandle_t receiveDataTask;

void updateTimeAndDate()
{
  unsigned long unix_epoch = timeClient.getEpochTime();
  sprintf(Time, "%02d:%02d:%02d", hour(unix_epoch), minute(unix_epoch), second(unix_epoch));
  sprintf(Date, "%02d-%02d-%04d", day(unix_epoch), month(unix_epoch), year(unix_epoch));
}
void setup()
{
  Serial.begin(115200);
  cantp.begin(500E3);
  cantp_pdu->data = buff;
  cantp_pdu->rxId = 0x7E8;
  cantp_pdu->txId = 0x7DF;
  cantp_pdu_rx->data = buff;
  cantp_pdu_rx->rxId = 0x7E8;
  cantp_pdu_rx->txId = 0x7DF;
  Serial.println("Starting");
  timecount = millis();

  Serial.begin(115200);
  ss.begin(GPSBaud);

  // connetWWifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  // connet Firebase database
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTHORIZATION_KEY);

  xTaskCreate(requestDataTaskFunction, "RequestDataTask", 10000, NULL, 1, &requestDataTask);
  xTaskCreate(receiveDataTaskFunction, "ReceiveDataTask", 10000, NULL, 1, &receiveDataTask);
}

void loop()
{
  timeClient.update();
  updateTimeAndDate();

  //gps 
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      unsigned long distanceKmToLondon =
      (unsigned long)TinyGPSPlus::distanceBetween(
       gps.location.lat(),
       gps.location.lng(),
       LONDON_LAT, 
       LONDON_LON) / 1000;

       double courseToLondon =
       TinyGPSPlus::courseTo(
       gps.location.lat(),
       gps.location.lng(),
       LONDON_LAT, 
       LONDON_LON);

 String path = String(Date);
  Firebase.setFloat(firebaseData, path + "/car/lat",  gps.location.lat());
   Firebase.setFloat(firebaseData, path + "/car/lng",  gps.location.lng());
}

void requestDataTaskFunction(void *pvParameters)
{
  for (;;)
  {
    RequestData();
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}

void receiveDataTaskFunction(void *pvParameters)
{
  for (;;)
  {
    ReceiveData();
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}

void RequestData()
{
  unsigned char data1[] = {01, 20, 00, 00, 00, 00, 00};    // Example data for message 1
  unsigned char data2[] = {01, 32, 00, 00, 00, 00, 00};    // Example data for message 2
  unsigned char data3[] = {01, 0xD, 00, 00, 00, 00, 00};   // data speed
  unsigned char data4[] = {03, 00, 00, 00, 00, 00, 00};    // chuẩn đoán mã lỗi
  unsigned char data5[] = {9, 02, 00, 00, 00, 00, 00};     // mã vin
  unsigned char data6[] = {01, 0XC, 00, 00, 00, 00, 00};   // mã ENGINE_RPM
  unsigned char data7[] = {01, 05, 00, 00, 00, 00, 00};    // engine_Coolant_Temperature_Msg
  unsigned char data8[] = {01, 0x10, 00, 00, 00, 00, 00};  // MAF: Mass air flow sensor (MAF)
  unsigned char data9[] = {01, 0x11, 00, 00, 00, 00, 00};  // throttle_position
  unsigned char data10[] = {01, 0x2F, 00, 00, 00, 00, 00}; // Fuel Level
  unsigned char data11[] = {01, 04, 00, 00, 00, 00, 00};   // Calculated engine load
  unsigned char data12[] = {01, 0x62, 00, 00, 00, 00, 00}; // percent torque
  unsigned char data13[] = {0XA, 00, 00, 00, 00, 00, 00};  // Mã lỗi cố định

  if (millis() > timecount + REQUEST_TIME)
  {
    switch (count)
    {
    case 1:
      cantp_pdu->data = data1;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 2:
      cantp_pdu->data = data2;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 3:
      cantp_pdu->data = data3;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 4:
      cantp_pdu->data = data4;
      cantp_pdu->len = 1;
      cantp.send(cantp_pdu);
      break;
    case 5:
      cantp_pdu->data = data5;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 6:
      cantp_pdu->data = data6;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 7:
      cantp_pdu->data = data7;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 8:
      cantp_pdu->data = data8;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 9:
      cantp_pdu->data = data9;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 10:
      cantp_pdu->data = data10;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 11:
      cantp_pdu->data = data11;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 12:
      cantp_pdu->data = data12;
      cantp_pdu->len = 2;
      cantp.send(cantp_pdu);
      break;
    case 13:
      cantp_pdu->data = data13;
      cantp_pdu->len = 1;
      cantp.send(cantp_pdu);
      break;
    }
    count++;
    if (count > 13)
      count = 1;
    timecount = millis();
  }
}

void ReceiveData()
{
  int retVal = -1;
  retVal = cantp.receive(cantp_pdu_rx);
  if (retVal == 0) // có data
  {
    ParseData(cantp_pdu_rx); // giải mã
  }
}

char *dec_to_hexa_conversion(int decimal_Number)
{
  int i = 0, j, temp;
  static char hexa_Number[100];

  while (decimal_Number != 0)
  {
    temp = decimal_Number % 16;
    if (temp < 10)
      temp = temp + 48;
    else
      temp = temp + 55;
    hexa_Number[i++] = temp;
    decimal_Number = decimal_Number / 16;
  }
  hexa_Number[i] = '\0';

  for (j = 0; j < i / 2; j++)
  {
    char temp = hexa_Number[j];
    hexa_Number[j] = hexa_Number[i - j - 1];
    hexa_Number[i - j - 1] = temp;
  }

  return hexa_Number;
}
void sendErrorCodesToFirebase()
{
  // Duyệt qua mảng errorCodes và gửi từng mã lỗi lên Firebase
  String path = String(Date);
  for (int i = 0; i < errorCodeIndex; i++)
  {
    String errorCodePath = String(FIREBASE_ERROR_CODES_PATH) + "/" + String(i);

    // Ghi đè dữ liệu cũ bằng một giá trị không mong muốn
    Firebase.setString(firebaseData, path + errorCodePath.c_str(), "");

    // Gửi mã lỗi mới lên Firebase
    Firebase.setString(firebaseData, path + errorCodePath.c_str(), errorCodes[i]);
    
    // Thêm xử lý lỗi nếu cần
  }
}

void print_hexadecimal_value_pair(int a1, int a2)
{
  char a[5];
  int bit76 = a1 >> 6;
  int bit54 = (a1 >> 4) & 0x3;
  int bit0123 = (a1 & 0xF);

  if (bit76 == 0)
  {
    a[0] = 'P';
  }
  else if (bit76 == 1)
  {
    a[0] = 'B';
  }
  else if (bit76 == 2)
  {
    a[0] = 'C';
  }
  else if (bit76 == 3)
  {
    a[0] = 'U';
  }

  if (bit54 == 0)
  {
    a[1] = '0';
  }
  else if (bit54 == 1)
  {
    a[1] = '1';
  }
  else if (bit54 == 2)
  {
    a[1] = '2';
  }
  else if (bit54 == 3)
  {
    a[1] = '3';
  }

  char hexa_bit0123[2];
  sprintf(hexa_bit0123, "%s", dec_to_hexa_conversion(bit0123));
  strcpy(a + 2, hexa_bit0123);

  char hexa_String[100];
  if (a2 < 16)
  { // Nếu byte thứ hai nhỏ hơn 16, thêm ký tự '0' vào trước để đảm bảo hiển thị đúng dạng hai chữ số hex
    sprintf(hexa_String, "%s0%s", a, dec_to_hexa_conversion(a2));
  }
  else
  {
    sprintf(hexa_String, "%s%s", a, dec_to_hexa_conversion(a2));
  }

  printf("Hexadecimal value pair is: %s\n", hexa_String);
  Serial.println("");

  // Firebase.setString(firebaseData, "/Mode3Data", hexa_String);

  // Lưu trữ hexa_String vào mảng errorCodes
  strcpy(errorCodes[errorCodeIndex], hexa_String);
  errorCodeIndex++;

  // Kiểm tra xem đã đạt đến giới hạn số lượng mã lỗi tối đa chưa
  if (errorCodeIndex >= MAX_ERRORS)
  {
    // Đạt đến giới hạn, gửi mảng đã lưu trữ lên Firebase và reset chỉ số
    sendErrorCodesToFirebase();
    Serial.print("da gui");
    errorCodeIndex = 0;
  }
}

void ParseData(pdu_t *pdu_data)
{
  switch (pdu_data->data[SID_POSITION] - RETURN_OFFSET) // SID(41)  41
  {
  case OBD_MODE_1:
    PareseData_OBD_MODE_1(pdu_data);
    break;
  case OBD_MODE_2:
    // PareseData_OBD_MODE_2(pdu_data);
    break;
  case OBD_MODE_3:
    PareseData_OBD_MODE_3(pdu_data);
    break;
  case OBD_MODE_4:
      // PareseData_OBD_MODE_4(pdu_data);
      break;
      case OBD_MODE_5:
    // PareseData_OBD_MODE_5(pdu_data);
    break;
  case OBD_MODE_9:
    PareseData_OBD_MODE_9(pdu_data);
    break;
  case OBD_MODE_A:
    PareseData_OBD_MODE_A(pdu_data);
    break;
  default:
  break;
  }
}

void PareseData_OBD_MODE_1(pdu_t *pdu_data)
{

  String path = String(Date);
  switch (pdu_data->data[PID_POSITION])
  {

  case MODE_1_VEHICAL_SPEED:
  {
    float VehicalSpeed = pdu_data->data[DATA_A_POSITION];
    Serial.print("VEHICLE_SPEED: ");
    Serial.println(VehicalSpeed);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID13/vehicle_speed", VehicalSpeed);
  }
  break;
  case MODE_1_ENGINE_RPM:
  {
    float data_1 = pdu_data->data[DATA_A_POSITION];
    float data_2 = pdu_data->data[DATA_B_POSITION];
    float EngineRPM = (data_1 * 256 + data_2) / 4;
    Serial.print("EngineRPM: ");
    Serial.println(EngineRPM);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID12/engine_rpm", EngineRPM);
  }
  break;
  case MODE_1_ENGINE_COOL_TEMP:
  {
    float EngineCoolTemp = (pdu_data->data[DATA_A_POSITION] - 40);
    Serial.print("Engine_Coolant_Temperature: ");
    Serial.println(EngineCoolTemp);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID05/engine_temperature", EngineCoolTemp);
  }
  break;
  case MODE_1_AIR_FLOW:
  {
    float data_1 = pdu_data->data[DATA_A_POSITION];
    float data_2 = pdu_data->data[DATA_B_POSITION];
    float MAF = (data_1 * 256 + data_2) / 100;
    Serial.print("MAF_Mass air flow sensor: ");
    Serial.println(MAF);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID10/air_flow_rate", MAF);
  }
  break;
  case MODE_1_THROTTLE_POSITION:
  {
    float data = pdu_data->data[DATA_A_POSITION];
    // float ThrottlePosition = (pdu_data->data[DATA_A_POSITION] * 100) / 255;
    float ThrottlePosition = (data * 100) / 255;
    Serial.print("Throttle_position: ");
    Serial.println(ThrottlePosition);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID11/throttle_pos", ThrottlePosition);
  }
  break;
  case MODE_1_FUEL_LEVEL:
  {
    float data = pdu_data->data[DATA_A_POSITION];
    // float FuelLevel = (pdu_data->data[DATA_A_POSITION] * 100) / 255;
    float FuelLevel = (data * 100) / 255;
    Serial.print("Fuel_Level: ");
    Serial.println(FuelLevel);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID47/fuel_level", FuelLevel); // 2F
  }
  break;
  case MODE_1_ENGINE_LOAD:
  {
    float data = pdu_data->data[DATA_A_POSITION];
    // float FuelLevel = (pdu_data->data[DATA_A_POSITION] * 100) / 255;
    float ENGINE_LOAD = (data * 100) / 255;
    Serial.print("ENGINE_LOAD: ");
    Serial.println(ENGINE_LOAD);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID04/engine_load", ENGINE_LOAD);
  }
  break;
  case MODE_1_PERCENT_TORQUE:
  {
    float data = pdu_data->data[DATA_A_POSITION];
    // float FuelLevel = (pdu_data->data[DATA_A_POSITION] * 100) / 255;
    float PERCENT_TORQUE = data - 125;
    Serial.print("PERCENT_TORQUE: ");
    Serial.println(PERCENT_TORQUE);
    Firebase.setFloat(firebaseData, path + "/Mode1_PID98/torque_force", PERCENT_TORQUE); // 62
  }
  break;
  }
}

void PareseData_OBD_MODE_9(pdu_t *pdu_data)
{

  char Vin[18] = {0};
  String path = String(Date);
  switch (pdu_data->data[PID_POSITION])
  {
  case 2:
  {

    // char Vin[18] = { 0 };  // Đảm bảo đủ kích thước để chứa chuỗi và ký tự kết thúc chuỗi
    for (int i = 0; i < 17; i++)
    {
      Vin[i] = pdu_data->data[i + 3];
    }
    Vin[17] = '\0'; // Thêm ký tự kết thúc chuỗi
    Serial.println(Vin);
  }
    Firebase.setString(firebaseData, path + "/Mode9_PID02/vin", Vin);
  }
}
//
void PareseData_OBD_MODE_3(pdu_t *pdu_data)
{
  int SoDTC = pdu_data->data[PID_POSITION] * 2;
  for (int j = 2; j < SoDTC + 2; j += 2) // Increment j by 2
  {
    print_hexadecimal_value_pair(pdu_data->data[j], pdu_data->data[j + 1]); // Pass data from pdu_data
  }
}

void PareseData_OBD_MODE_A(pdu_t *pdu_data)
{
  int SoDTC = pdu_data->data[PID_POSITION] * 2;
  for (int j = 2; j < SoDTC + 2; j += 2) // Increment j by 2
  {
    print_hexadecimal_value_pair(pdu_data->data[j], pdu_data->data[j + 1]); // Pass data from pdu_data
  }
}


