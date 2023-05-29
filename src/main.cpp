/*============ Máy cắt vật liệu ============
Vi điều khiển: ESP32
Phiên bản: V1.0
Ngày   : 24/04/2023
Update1: 07/05/2023

Mô tả phiên bản: 

*/

#include <Arduino.h>
#include <AccelStepper.h>  // https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#adfb19e3cd2a028a1fe78131787604fd1
#include <MultiStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SH110X.h>    // OLED 1.3(inch) https://github.com/adafruit/Adafruit_SH110x  
#include <Adafruit_SSD1306.h>
//#include <ClickEncoder.h>
#include <Servo_ESP32.h>  // Servo cho ESP32
#include <EEPROM.h>

// Information
#define Model          "Model: BM0523CVL      "
#define SeriaNumber    "S/N  : SN01         "
#define FirmwareVision "FW   : V1.0 ESP32   "
#define Author         "      Ba Manh 4E    "
#define SDT            "     036.788.0317   "
#define Origin         "   Made in VietNam  "
// 
#define InformationPage         0
#define CutSpeedSettingPage     1
#define WireSpeedSettingPage    2
#define CuttingAngleSettingPage 3
#define DiameterSettingPage     4
#define HomeScreenPage          5
#define ChooseLengthPage        6
#define ChooseQuantityPage      7
#define ConfirmPage             8
#define CurrentlyCuttingPage    9
#define CompletePage            10
// Button pins
#define Next_Button       26        // GPIO26
#define Back_Button       25        // GPIO25
#define Up_Button         33        // GPIO33
#define Down_Button       32        // GPIO32
// The X Stepper pins
#define STEPPER1_DIR_PIN  18        // GPIO18  
#define STEPPER1_STEP_PIN 19        // GPIO19
// The Y stepper pins
#define STEPPER2_DIR_PIN  5         // GPIO5
#define STEPPER2_STEP_PIN 17        // GPIO17
// Roll IN/OUT pins
#define BTN1_PIN          16        // GPIO16
#define BTN2_PIN          4         // GPIO4
// 
#define led               2         // GPIO2
#define Relay             14        // GPIO14 Relay cắt vật liệu
#define servoPin          27        // GPIO27 Servo cắt vật liệu

#define OLED_RESET        -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS    0x3C      //< See datasheet for Address
const uint8_t SCREEN_WIDTH  = 128;  // OLED display width, in pixels
const uint8_t SCREEN_HEIGHT = 64;   // OLED display height, in pixels

//======= System Parameters ========//
const uint8_t angleMin            = 0;        // Góc ban đầu Servo
const uint8_t angleMax            = 180;      // Góc cắt của Servo
const uint16_t SetMaxSpeed        = 4000;     // Set tốc độ tối đa động cơ
const uint16_t DELAY_BETWEEN_CUTS = 100;      // Thời gian chờ cắt sợi tiếp theo
const float pi                    = 3.141592; // Số Pi
const float Full_Step             = 1.0;      //
const float Half_Step             = 0.5;      // 1/2
const float Quarter_Step          = 0.25;     // 1/4
const float Eighth_Step           = 0.125;    // 1/8
const float Sixteenth_Step        = 0.0625;   // 1/16
    
float Microstep = Quarter_Step;                          // Chế độ 
float StepPerRound = 200.0/Microstep;                    // Số xung để quay được vòng 360 độ
float LengthPerStep = (pi * 1.8 * Microstep) / 360.0;    // Chiều dài l trên một bước (R*pi*n)/180 = (D*pi*n)/360
uint16_t Steps = 0;

int16_t
state = 5,                     // Home Screen = state 5
incrementSpeed = 1,

WireLength           = 10,     // Wire Length                   | Chiều dài vật liệu cắt
Quantity             = 10,     // Wire Quantity                 | Số lượng  
Diameter             = 10,     // Shaft Diameter                | Đường kính trục con lăn
CuttingAngle         = 400,    // Cutting Angle                 | Góc cắt
WireSpeed            = 50,     // Wire Speed Steps Motor 0-100% | Tốc độ dây
CutSpeed             = 50,     // Cut Speed Steps Motor 0-100%  | Tốc độ cắt
previousWireLength   = 0,      
previousWireQuantity = 0,
previousDiameter     = 0,
previousCuttingAngle = 0,
previousWireSpeed    = 0,
previousCutSpeed     = 0,
WireSpeed_Raw        = 0,
CutSpeed_Raw         = 0,
CuttingAngle_Raw     = 0;

typedef struct 
{
  uint16_t EEPROM_Diameter;
  uint16_t EEPROM_CuttingAngle;
  uint16_t EEPROM_WireSpeed;
  uint16_t EEPROM_CutSpeed;
}ParameterSetting;
ParameterSetting EEPROM_Setting;

// Khai báo Servo
Servo_ESP32 servo1;
// Khai báo động cơ Steper
AccelStepper linMotSteppers(1, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper extruderStepper(1, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
// Khai báo màn OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int changeValue(int currentValue) {
  if (!digitalRead(Up_Button)) {
    currentValue += incrementSpeed;
    incrementSpeed++;
  }
  if (!digitalRead(Down_Button)) {
    if (currentValue - incrementSpeed >= 0) {
      currentValue -= incrementSpeed;
      incrementSpeed++;
    } else {
      currentValue = 0;
    }
  }
  if (digitalRead(Up_Button) && digitalRead(Down_Button)) {
    incrementSpeed = 1;
  }
  delay(80);
  return currentValue;
}

void ButtonHandle()
{
  if (!digitalRead(Next_Button)) 
  {
    if (state == 11) state = 0;
    else state += 1;
    delay(200);
  }
  if (!digitalRead(Back_Button) && state > 0 && state < 11) 
  {
    state -= 1;
    delay(200);
  }
  
  if (!digitalRead(BTN1_PIN)) 
  {
     extruderStepper.setCurrentPosition(0);
     extruderStepper.moveTo(100);  //set vị trí đích đến, số vòng 600 = 3 vòng
     extruderStepper.setSpeed(WireSpeed_Raw); //Cho motor chạy với tốc độ 200 bước/s
     while (extruderStepper.distanceToGo() != 0) // 400 bước = 2 vòng ..... currentPosition() trả về vị trí hiện tại của ĐC
      {
         extruderStepper.runSpeedToPosition();
      }
  }   
  if (!digitalRead(BTN2_PIN)) 
  {
     extruderStepper.setCurrentPosition(100);
     extruderStepper.moveTo(0);  //set vị trí đích đến, số vòng 600 = 3 vòng
     extruderStepper.setSpeed(WireSpeed_Raw); //Cho motor chạy với tốc độ 200 bước/s
     while (extruderStepper.distanceToGo() != 0) // 400 bước = 2 vòng ..... currentPosition() trả về vị trí hiện tại của ĐC
      {
         extruderStepper.runSpeedToPosition();
      }
  }
}
void EEPROM_SaveSetting()
{
  EEPROM_Setting.EEPROM_CutSpeed = CutSpeed;
  EEPROM_Setting.EEPROM_WireSpeed = WireSpeed;
  EEPROM_Setting.EEPROM_Diameter = Diameter;
  EEPROM_Setting.EEPROM_CuttingAngle = CuttingAngle;
  EEPROM.put(0, EEPROM_Setting);
  EEPROM.commit();
}
void EEPROM_LoadSetting()
{
  EEPROM.get(0, EEPROM_Setting);
  Diameter = EEPROM_Setting.EEPROM_Diameter;
  CuttingAngle = EEPROM_Setting.EEPROM_CuttingAngle;
  WireSpeed = EEPROM_Setting.EEPROM_WireSpeed;
  CutSpeed = EEPROM_Setting.EEPROM_CutSpeed;
}

void moveBlade(int Steps) {
  linMotSteppers.setCurrentPosition(0);
  linMotSteppers.moveTo(Steps);  
  linMotSteppers.setSpeed(CutSpeed_Raw); 
  digitalWrite(led, HIGH);
  while (linMotSteppers.distanceToGo() != 0)
  {
      linMotSteppers.runSpeedToPosition();
  }
  digitalWrite(led, LOW);
  linMotSteppers.setCurrentPosition(Steps);
  linMotSteppers.moveTo(0); 
  linMotSteppers.setSpeed(CutSpeed_Raw); 
  while (linMotSteppers.distanceToGo() != 0) 
  {
      linMotSteppers.runSpeedToPosition();
  }
}
void moveWire(float wirelength) {  
  Steps = wirelength/(LengthPerStep*(float)Diameter);  // 

  extruderStepper.setCurrentPosition(0);
  while (extruderStepper.currentPosition() != Steps)
  {
    extruderStepper.setSpeed(WireSpeed_Raw); 
    extruderStepper.runSpeed();
  }
}

void displayNavigation(String back, String next) {
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 56);
  display.print(back);
  display.setCursor(67, 56);
  display.print(next);
}
void displayStringSettingPage(String NamePage, int value, String unit) {
  display.setTextSize(1);
  display.setCursor(0, 22);
  display.print(NamePage);
  display.setTextSize(2);
  display.setCursor(0, 35);
  display.print("> ");
  display.setTextColor(BLACK, WHITE);
  display.print(value);
  display.setTextColor(WHITE, BLACK);
  display.print(unit);
}
void displayTitlePage(String NamePage)
{
  display.fillRect(0,0,128,18, WHITE);
  display.setTextSize(2);
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 1);
  display.print(NamePage);
  display.setTextColor(WHITE, BLACK);
  //display.drawLine(0, 18, 128, 18, WHITE);
}

void Information()         // Page 1
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.println(Model);
  display.setCursor(0, 10);
  display.println(SeriaNumber);
  display.setCursor(0, 20);
  display.println(FirmwareVision);

  display.fillRoundRect(0, 30, 128, 34, 3, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(0, 33);
  display.println(Author);
  display.setCursor(0, 43);
  display.println(SDT);
  display.setCursor(0, 53);
  display.println(Origin);
  display.display();
}
void CutSpeedSetting()     // Page 2
{
  CutSpeed = changeValue(CutSpeed);
  if (previousCutSpeed != CutSpeed) {
    display.clearDisplay();
    previousCutSpeed = CutSpeed;
  }
  if(CutSpeed>100) CutSpeed = 100;

  display.clearDisplay();
  displayTitlePage("  SETTING ");
  displayStringSettingPage("      Cut Speed:    ", CutSpeed, " %");
  displayNavigation("<Info    ", "  W.SPEED>");
  display.display();
}
void WireSpeedSetting()    // Page 3
{
  WireSpeed = changeValue(WireSpeed);
  if (previousWireSpeed != WireSpeed) {
    display.clearDisplay();
    previousWireSpeed = WireSpeed;
  }
  if(WireSpeed>100) WireSpeed = 100;

  display.clearDisplay();
  displayTitlePage("  SETTING ");
  displayStringSettingPage("      Wire Speed:    ", WireSpeed, " %");
  displayNavigation("<C.SPEED", "   ANGLE>");
  display.display();
}
void CuttingAngleSetting() // Page 4
{
  CuttingAngle = changeValue(CuttingAngle);
  if (previousCuttingAngle != CuttingAngle) {
    display.clearDisplay();
    previousCuttingAngle = CuttingAngle;
  }
  if(CuttingAngle>360) CuttingAngle = 360;
  
  display.clearDisplay();
  displayTitlePage("  SETTING ");
  displayStringSettingPage("    Cutting Angle:   ", CuttingAngle, " deg");
  displayNavigation("<W.SPEED", "DIAMETER>");
  display.display();
  CuttingAngle_Raw = map(CuttingAngle, 1, 360, 1, StepPerRound);
  //moveBlade(CuttingAngle_Raw);
}
void DiameterSetting()     // Page 5
{
  Diameter = changeValue(Diameter);
  if (previousDiameter != Diameter) {
    display.clearDisplay();
    previousDiameter = Diameter;
  }

  display.clearDisplay();
  displayTitlePage("  SETTING ");
  displayStringSettingPage("      Diameter:      ", Diameter, " mm");
  displayNavigation("<ANGLE", "    HOME>");
  display.display();
}
void HomeScreen()          // Page 6
{
  WireSpeed_Raw = map(WireSpeed, 1, 100, 1, SetMaxSpeed);
  CutSpeed_Raw  = map(CutSpeed, 1, 100, 1, SetMaxSpeed);
  CuttingAngle_Raw = map(CuttingAngle, 1, 360, 1, StepPerRound);

  display.clearDisplay();
  displayTitlePage("   HOME   ");
  display.setCursor(65, 50);
  display.print("NEXT>");
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print("<SETUP");
  display.display();
}
void ChooseLength()        // Page 7
{
  WireLength = changeValue(WireLength);

  if (previousWireLength != WireLength) {
    display.clearDisplay();
    previousWireLength = WireLength;
  }

  display.clearDisplay();
  displayTitlePage("  LENGTH  ");
  display.setCursor(0, 27);
  display.print("> ");
  display.print(WireLength);
  display.print(" mm");
  displayNavigation("<HOME", "    NEXT>");
  display.display();
}
void ChooseQuantity()      // Page 8
{
  Quantity = changeValue(Quantity);

  if (previousWireQuantity != Quantity) {
    display.clearDisplay();
    previousWireQuantity = Quantity;
  }

  display.clearDisplay();
  displayTitlePage(" QUANTITY ");
  display.setCursor(0, 27);
  display.print("> ");
  display.print(Quantity);
  display.print(" PCS");
  displayNavigation("<BACK", "    NEXT>");
  display.display();
}
void Confirm()             // Page 9
{
  display.clearDisplay();
  displayTitlePage("  CONFIRM ");
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Length: ");
  display.print(WireLength);
  display.print(" (mm)");
  display.setCursor(0, 36);
  display.print("Quantity: ");
  display.print(Quantity);
  display.print(" (PCS)");
  displayNavigation("<BACK", "    NEXT>");
  display.display();
}
void CurrentlyCutting()    // Page 10
{
  display.clearDisplay();
  displayTitlePage("PROCESSING");
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Length: ");
  display.print(WireLength);
  display.print(" mm");
  display.display();
  
  for (int i = 0; i < Quantity; i++) 
  {
    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print((String)(i + 1) + "/" + (String)Quantity + " PCS");
    display.display();

    moveWire(WireLength);
    moveBlade(CuttingAngle_Raw);
    delay(DELAY_BETWEEN_CUTS);
  }
  state = CompletePage;
}
void Complete()            // Page 11
{
  display.setTextSize(2);
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 20);
  display.print(" COMPLETE ");
  display.display();
  EEPROM_SaveSetting();
  delay(2000);
  state = HomeScreenPage;   // Return Home Page
}

void setup() {
  pinMode(Next_Button, INPUT_PULLUP);  // Không có trở kéo ngoài
  pinMode(Back_Button, INPUT);
  pinMode(Up_Button  , INPUT);
  pinMode(Down_Button, INPUT);
  pinMode(BTN1_PIN   , INPUT);
  pinMode(BTN2_PIN   , INPUT);
  pinMode(led        , OUTPUT);
  pinMode(Relay      , OUTPUT);

  EEPROM.begin(512);
  Serial.begin(115200);
  // Khởi tạo Servo
  servo1.attach(servoPin);
  servo1.write(angleMin);
  // Khởi tạo động cơ Step
  linMotSteppers.setMaxSpeed(SetMaxSpeed);  //Đặt tốc độ tối đa, mặc định là rất chậm
  extruderStepper.setMaxSpeed(SetMaxSpeed); //Đặt tốc độ tối đa, mặc định là rất chậm
  // Khởi tạo OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Đọc giá trị cài đặt trong EEPROM
  EEPROM_LoadSetting();
}  // end setup

void loop() {
  ButtonHandle();

  switch (state) {
    case 0:
      Information();
      break;
    case 1:
      CutSpeedSetting();
      break;
    case 2:
      WireSpeedSetting();
      break;
    case 3:
      CuttingAngleSetting();
      break;
    case 4:
      DiameterSetting();
      break;
    case 5:
      HomeScreen();
      break;
    case 6:
      ChooseLength();
      break;
    case 7:
      ChooseQuantity();
      break;
    case 8:
      Confirm();
      break;
    case 9:
      CurrentlyCutting();
      break;
    case 10:
      Complete();
      break;
  }
}  // end loop
