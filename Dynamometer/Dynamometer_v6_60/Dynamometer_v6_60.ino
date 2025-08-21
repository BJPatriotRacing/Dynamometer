/*
  ---------------------------------------------------------
  PROGRAM INFORMATION
  ---------------------------------------------------------
  Bob Jones Patriot Racing Motor Dynamometer
  Copyright 2016-2022, All Rights reserved
  This code cannot be used outside of Bob Jones High School
  Code for Teensy 3.2
  ---------------------------------------------------------
  COMPILE INSTRUCTIONS
  ---------------------------------------------------------
  Compile Speed:  72MHz
  Optimize:       Smallest code

  rev   author      date        code
  4.6   kasprzak    9-2022      added support for dual load cells
  4.7   kasprzak    11-2022     moved all setups to menu system
  5.0   kasprzak    6-2025      new speed monitoring system
  6.0   kasprzak    6-2025      back to single load cell (still monitor CCW and CW operations), added review capability

*/

#include <SPI.h>
#include <EEPROM.h>  // standard library that ships with Teensy
#include <ILI9341_t3.h>
#include <font_Arial.h>      // custom fonts that ships with ILI9341_t3.h
#include <font_ArialBold.h>  // custom fonts that ships with ILI9341_t3.h
#include <avr/io.h>          // standard library that ships with Teensy
#include <avr/interrupt.h>   // standard library that ships with Teensy
#include <SdFat.h>           // SdFat library for more SD functions
#include "HX711.h"           // load cell lib
#include "UTouch.h"          // touchscreen lib
#include <TimeLib.h>
#include <ILI9341_t3_Controls.h>
#include <FlickerFreePrint.h>  // library to draw w/o flicker
#include "Colors.h"
#include <ILI9341_t3_PrintScreen_SdFat.h>
#include "ILI9341_t3_Menu.h"
#include <FreqMeasure.h>  // lib for speed sensor https://github.com/PaulStoffregen/FreqMeasure

// #define DO_DEBUG

/////////////////////////////////////////////////////////////////////////
#define CODE_VERSION "v6.6"
/////////////////////////////////////////////////////////////////////////

#define LIMIT_L 1400
#define LIMIT_H 3200

///  we follow the IEC standard where the direction of rotation is always viewed from the driven end side, where the load is
#define CCW 0  // defaut original F24 direction + to +
#define CW 1   // wha we considere reversing that direction  + to -
#define REQUIRED_POINTS 80
#define CHINESE_MOTOR 1
#define UK_MOTOR 0

#define F_A20 Arial_20
#define F_A16 Arial_16
#define F_A12B Arial_12_Bold
#define F_A12 Arial_12
#define F_A08 Arial_8

#define TOR_COLOR C_CYAN
#define AMP_COLOR C_PINK
#define VOLT_COLOR C_ORANGE
#define POINT_DIA 1
#define BTN_RADIUS 4
#define BTN_THICK 1
#define TIME_HEADER "T"
#define MINIMUM_PULSES 3
#define SPI_SPEED 10
#define NTC_A 3.354016E-03  // from the data sheet
#define NTC_B 2.569850E-04  // from the data sheet
#define NTC_C 2.620131E-06  // from the data sheet
#define NTC_D 6.383091E-08  // from the data sheet
#define NTC_R1 9800.0       // resistor for thermsitor voltage divider

// pin defs
#define BUZZ_PIN A7  // alert buzzer
#define RPM_PIN 3    // pin for the RPM
#define TH_PIN A1    // thermisto measurement pin
#define AM_PIN A2    // amp sensor pin
#define VM_PIN A9    // voltage divider input
#define LOADCELL_DOUT_PIN A5
#define LOADCELL_SCK_PIN A6
#define SDCS_PIN A8  // CS for SD card
#define tr1 9800.0   // resistor for thermsitor voltage divider
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10
// UTouch Touch(6, 5, 4, A0, 2);

// variables for the locations of the keypad buttons
#define BUTTON_X 90
#define BUTTON_Y 60
#define BUTTON_W 70
#define BUTTON_H 30
#define BUTTON_SPACING_X 5
#define BUTTON_SPACING_Y 5
#define BUTTON_TEXTSIZE 2

//Resistor for Thermistor Voltage Divider
#define tr1 9800.0  // resistor for thermsitor voltage divider
#define R1 9850     // resistor for the voltage divider
#define R2 984

int BtnX, BtnY;
const int stepsPerRevolution = 2048;
uint8_t np = 0;
char dn[12];
uint16_t LineColor = C_RED;
char buf[50];
bool StartTest = false;
int LC_year = 2025, LC_month = 6, LC_day = 11, SC_year = 2025, SC_month = 6, SC_day = 11;
int LC_TestsSinceCal = 0;
int SC_TestsSinceCal = 0;
int LoadCellReads = 1;
int Update = 200;  // milliseconds between calculations and screen refresh and SD write
long tr2 = 0;
float LoadForce = 0.0f;
float Torque = 0.0f;
float MPower = 0.0f;
float LoadMass = 0.0f;
float MotorEff = 0.0f;
uint8_t MotorType = 0;
uint8_t MotorDirection = 0;
int MotorNumber = 0;
uint32_t AverageReads = 0;
float dAmps[LIMIT_H - LIMIT_L];
float dTorque[LIMIT_H - LIMIT_L];
float dEffeciency[LIMIT_H - LIMIT_L];
float dVolts[LIMIT_H - LIMIT_L];
float dLoad[LIMIT_H - LIMIT_L];
float dMPower[LIMIT_H - LIMIT_L];
uint16_t dPoint[LIMIT_H - LIMIT_L];
float LastAmps = 0.0f;
float LastTorque = 0.0f;
float LastEffeciency = 0.0f;
float LastVolts = 0.0f;
float LastLoad = 0.0f;
float LastMPower = 0.0f;
uint16_t LastPoint = 0;

float StartAmps = 0.0f;
float StartTorque = 0.0f;
float StartMotorEff = 0.0f;
float StartVolts = 0.0f;
float StartLoadMass = 0.0f;
float StartMPower = 0.0f;

// setup variables

float ForceArm = 3.375;        // size in inches
float LoadCellCal = -473.20f;  // sum of both batteries at 10.5 volt mark
float LoadCellOffset = 136240.0f;
float VDCompensation = 13.0f;  // Vin is comming through diodes before vvoltage divider--can't use standard equation.
float VMOffset = 0.75f;
uint8_t m, h, s;
uint8_t col = 240;                                 // location for the setup variables
uint8_t row = 20;                                  // height of each row in setup
float VMid = 0.396f;                               // offset for current sensor straignt from data sheet
float mVPerAmp = 26.4f;                            // sensitivity for current sensor straignt from data sheet
char FileName[17] = "MNDDD_TTT_RR.CSV";            // SD card filename
float thVolts = 0.0f, TempF = 0.0f, TempK = 0.0f;  // computed temp values
float TempOffset = 0.0f;
volatile unsigned long Counter = 0;  // the number of measurements between each display
volatile uint32_t RPMSum = 0;
volatile uint32_t RPMCount = 0;
volatile uint16_t RPMMinCount = 0;
unsigned long MRPM = 0, StartMRPM = 0;  // wheel rpm (measured)
unsigned long PreviousRPM = 15000;
float vVolts = 0.0f, Volts = 0.0f;  // computed Volts
float aVolts = 0.0f, Amps = 0.0f;   // computed Amps
float EPower = 0.0f;                // computed EPower
// resistor for the voltage divider

uint16_t Point = 0;  // Counter for the data Point
bool isSD = false;   // is SD card working
uint8_t FileCount = 0;
int top = 0, wide = 0, high = 0, Left = 0;  // holders for screen coordinate drawing
bool KeepIn = 0;
uint8_t b = 0;
// time variables
unsigned long curtime = 0;  // current time actual - startup time
float TempNum = 0.0f;       // some temp number

int pEff, pTorque, pAmps, pVolts;
int i = 0;  // just some storage variables
const char *OffOnItems[] = { "Off", "On" };
const char *ReadText[] = { "Reset" };
const char *DirectionText[] = { "CCW", "CW" };
int MenuOption1 = 0, MenuOption2 = 0, MenuOption3 = 0, MenuOption4 = 0, MenuOption5 = 0;
int MenuOption6 = 0, MenuOption7 = 0, MenuOption8 = 0, MenuOption9 = 0, MenuOption10 = 0;
int MenuOption11 = 0, MenuOption12 = 0, MenuOption13 = 0;
int MenuOption14 = 0, MenuOption15 = 0, MenuOption16 = 0, MenuOption17 = 0;

ILI9341_t3 Display(TFT_CS, TFT_DC, TFT_RST);

// create the // Touch screen object
UTouch Touch(6, 5, 4, A0, 2);

EditMenu OptionMenu(&Display, true);

Button ProfileBtn(&Display);
Button SensorsBtn(&Display);
Button ConnectionBtn(&Display);
Button ReviewBtn(&Display);
Button MotorTypeBtn(&Display);
Button StartBtn(&Display);
Button StartTestBtn(&Display);
Button StopBtn(&Display);
Button ReRunBtn(&Display);
Button SetTareBtn(&Display);
Button DirBtn(&Display);
Button PreviousNumberBtn(&Display);
Button NextNumberBtn(&Display);
Button BackBtn(&Display);
Button DoneBtn(&Display);
Button MenuDoneBtn(&Display);

SdFat sd;
SdFile DataFile;
SdFile Root;
SdFile FileFile;

HX711 LoadSensor;

time_t RTCTime;

elapsedMillis CalTime;
elapsedMillis UpdateTime;

CGraph Graph(&Display, 20, 210, 260, 160, LIMIT_L, LIMIT_H, 200, 0, 100, 10);

FlickerFreePrint<ILI9341_t3> ffCalLoad(&Display, C_WHITE, C_BLACK);

FlickerFreePrint<ILI9341_t3> ffPoint(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffTorque(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffTemp(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffVolts(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffAmps(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffRPM(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffMass(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffFileName(&Display, C_WHITE, C_BLACK);

FlickerFreePrint<ILI9341_t3> ffPercent(&Display, C_WHITE, C_DKBLUE);

/**********************************************************************************************

  program setup

***********************************************************************************************/
void setup() {

  Serial.begin(115200);

  // while (!Serial) {}

  pinMode(BUZZ_PIN, OUTPUT);

  delay(100);

  GetParameters();

  // setup display
  Display.begin();
  // set orientation--using math to flip display upside down if display is mounted that way
  Display.setRotation(1);

  Display.fillScreen(C_BLACK);

  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  RTCTime = processSyncMessage();

  if (RTCTime != 0) {
    setTime(RTCTime);
    Teensy3Clock.set(RTCTime);  // set the RTC
  }

  Touch.InitTouch(PORTRAIT);
  Touch.setPrecision(PREC_EXTREME);
  Display.fillScreen(C_BLACK);

  Display.fillRect(10, 10, 300, 10, C_RED);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A20);

  Display.setCursor(160 - Display.measureTextWidth("PATRIOT RACING") / 2, 35);
  Display.print(F("PATRIOT RACING"));

  Display.setCursor(160 - Display.measureTextWidth("DYNAMOMETER") / 2, 70);
  Display.print(F("DYNAMOMETER"));

  Display.setCursor(60, 105);
  strcpy(buf, "Version: ");
  strcat(buf, CODE_VERSION);
  Display.setCursor(160 - Display.measureTextWidth(buf) / 2, 105);
  Display.print(buf);

  sprintf(dn, "%01d/%01d/%04d", month(), day(), year());
  Display.setCursor(160 - Display.measureTextWidth(dn) / 2, 140);
  Display.print(dn);
  Display.fillRect(10, 180, 300, 10, C_BLUE);

  Display.fillRoundRect(20, 200, 280, 30, 8, C_WHITE);
  Display.fillRoundRect(20, 200, 20, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Starting"));

  LoadSensor.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);


  LoadSensor.set_gain(HX711_CHANNEL_A_GAIN_64, true);

  LoadSensor.set_scale(LoadCellCal);
  LoadSensor.set_offset(LoadCellOffset);

  asm(".global _printf_float");

  analogWriteFrequency(BUZZ_PIN, 1500);

  // chirp
  for (i = 0; i < 2; i++) {
    analogWrite(BUZZ_PIN, 100);
    delay(100);
    analogWrite(BUZZ_PIN, 0);
    delay(100);
  }

  // fire up the RPM sensor
  FreqMeasure.begin();

  // setTime(hours, minutes, seconds, days, months, years);
  // setTime(19, 58, 0, 1, 12, 2020);
  // Teensy3Clock.set(now());

  Display.fillRoundRect(20, 200, 280, 30, 8, C_WHITE);
  Display.fillRoundRect(20, 200, 80, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Building menus"));

  delay(100);

  ProfileBtn.init(80, 100, 150, 80, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Start Test", 0, 0, F_A20);
  SensorsBtn.init(80, 190, 150, 80, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Sensors", 0, 0, F_A20);
  ConnectionBtn.init(240, 100, 150, 80, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Wiring", 0, 0, F_A20);
  ReviewBtn.init(240, 190, 150, 80, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Review", 0, 0, F_A20);
  OptionMenu.init(C_WHITE, C_BLACK, C_WHITE, C_DKBLUE, C_WHITE, C_DKRED, 230, 40, 4, "Settings", F_A16, F_A20);

  MenuOption12 = OptionMenu.addNI("Direction", MotorDirection, 0, sizeof(DirectionText) / sizeof(DirectionText[0]), 1, 0, DirectionText);
  MenuOption1 = OptionMenu.addNI("Brake Arm [in]", ForceArm, 1.0, 4.0, 0.001, 3);
  MenuOption2 = OptionMenu.addNI("Calibrate at 0 kg", 0, 0, sizeof(ReadText) / sizeof(ReadText[0]), 1, 0, ReadText);
  MenuOption3 = OptionMenu.addNI("Calibrate at 1 kg", 0, 0, sizeof(ReadText) / sizeof(ReadText[0]), 1, 0, ReadText);
  MenuOption4 = OptionMenu.addNI("Samples", LoadCellReads, 1, 10, 1, 0);
  MenuOption5 = OptionMenu.addNI("Update time [ms]", Update, 200, 3000, 25, 0);
  MenuOption7 = OptionMenu.addNI("Volt Offset (0.75)", VMOffset, .1, .8, .01, 2);
  MenuOption8 = OptionMenu.addNI("Volt slope (11.0)", VDCompensation, 9, 12, .01, 2);
  MenuOption9 = OptionMenu.addNI("CODE", VMid, .4, .8, .001, 3);
  MenuOption10 = OptionMenu.addNI("Amp slope (25.9)", mVPerAmp, 21.0, 30.0, .1, 1);
  MenuOption11 = OptionMenu.addNI("Temp bias [F]", TempOffset, -10.0, 10.0, 0.1, 1);

  OptionMenu.setTitleTextMargins(50, 10);
  OptionMenu.setItemTextMargins(5, 11, 5);
  OptionMenu.setMenuBarMargins(5, 315, 3, 1);

  StopBtn.init(278, 20, 80, 35, C_DKRED, C_RED, C_WHITE, C_DKGREY, "Stop", 0, 0, F_A16);
  StartTestBtn.init(278, 20, 80, 35, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "Start", 0, 0, F_A16);
  ReRunBtn.init(278, 20, 80, 35, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "New", 0, 0, F_A16);
  BackBtn.init(BUTTON_X + (2 * (BUTTON_W + BUTTON_SPACING_X)), BUTTON_Y - BUTTON_H - BUTTON_SPACING_Y, BUTTON_W, BUTTON_H, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "<", 0, 0, F_A16);
  DoneBtn.init(BUTTON_X + (BUTTON_W + BUTTON_SPACING_X), BUTTON_SPACING_Y + BUTTON_SPACING_Y + BUTTON_Y + (4 * (BUTTON_H + BUTTON_SPACING_Y)), 3 * (BUTTON_W + BUTTON_SPACING_X), BUTTON_H, C_BLACK, C_GREEN, C_BLACK, C_DKGREY, "Done", 0, 0, F_A16);
  MenuDoneBtn.init(278, 25, 80, 40, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "Done", 0, 0, F_A16);
  SetTareBtn.init(55, 100, 80, 75, C_DKRED, C_RED, C_BLACK, C_DKGREY, "Zero", 0, 0, F_A20);
  StartBtn.init(55, 180, 80, 75, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "Start", 0, 0, F_A20);
  DirBtn.init(240, 80, 110, 50, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Rotation", 0, 0, F_A16);
  PreviousNumberBtn.init(210, 140, 50, 50, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "<", 0, 0, F_A20);
  NextNumberBtn.init(270, 140, 50, 50, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, ">", 0, 0, F_A20);
  MotorTypeBtn.init(240, 200, 110, 50, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Mfg.", 0, 0, F_A16);

  Graph.init("", "RPM x100", "", C_WHITE, C_MDGREY, C_GREY, C_BLACK, C_BLACK, F_A12, F_A08);
  Graph.showTitle(false);
  Graph.showLegend(true);
  Graph.drawLegend(LOCATION_BOTTOM);

  pEff = Graph.add("Eff.", LineColor);
  pTorque = Graph.add("Torque x10", TOR_COLOR);
  pAmps = Graph.add("Amps", AMP_COLOR);
  pVolts = Graph.add("Volts", VOLT_COLOR);

  Graph.setLineThickness(pEff, 2);
  Graph.setLineThickness(pTorque, 2);
  Graph.setLineThickness(pAmps, 2);
  Graph.setLineThickness(pVolts, 2);
  Graph.setMarkerSize(pEff, POINT_DIA);
  Graph.setMarkerSize(pTorque, POINT_DIA);
  Graph.setMarkerSize(pAmps, POINT_DIA);
  Graph.setMarkerSize(pVolts, POINT_DIA);
  Graph.setXTextOffset(5);
  Graph.setYTextOffset(18);
  Graph.setYLegendOffset(3);
  Graph.setXTextScale(.01);

  ProfileBtn.setCornerRadius(BTN_RADIUS);
  SensorsBtn.setCornerRadius(BTN_RADIUS);
  ConnectionBtn.setCornerRadius(BTN_RADIUS);
  ReviewBtn.setCornerRadius(BTN_RADIUS);
  DirBtn.setCornerRadius(BTN_RADIUS);
  PreviousNumberBtn.setCornerRadius(BTN_RADIUS);
  NextNumberBtn.setCornerRadius(BTN_RADIUS);

  MotorTypeBtn.setCornerRadius(BTN_RADIUS);
  StartBtn.setCornerRadius(BTN_RADIUS);
  StartTestBtn.setCornerRadius(BTN_RADIUS);
  StopBtn.setCornerRadius(BTN_RADIUS);
  ReRunBtn.setCornerRadius(BTN_RADIUS);
  SetTareBtn.setCornerRadius(BTN_RADIUS);
  BackBtn.setCornerRadius(BTN_RADIUS);
  DoneBtn.setCornerRadius(BTN_RADIUS);
  MenuDoneBtn.setCornerRadius(BTN_RADIUS);

  ProfileBtn.setBorderThickness(BTN_THICK);
  SensorsBtn.setBorderThickness(BTN_THICK);
  ConnectionBtn.setBorderThickness(BTN_THICK);
  ReviewBtn.setBorderThickness(BTN_THICK);
  DirBtn.setBorderThickness(BTN_THICK);
  PreviousNumberBtn.setBorderThickness(BTN_THICK);
  NextNumberBtn.setBorderThickness(BTN_THICK);

  MotorTypeBtn.setBorderThickness(BTN_THICK);
  StartBtn.setBorderThickness(BTN_THICK);
  StartTestBtn.setBorderThickness(BTN_THICK);
  StopBtn.setBorderThickness(BTN_THICK);
  ReRunBtn.setBorderThickness(BTN_THICK);
  SetTareBtn.setBorderThickness(BTN_THICK);
  BackBtn.setBorderThickness(BTN_THICK);
  DoneBtn.setBorderThickness(BTN_THICK);
  MenuDoneBtn.setBorderThickness(BTN_THICK);

  // setup analog read resolutions
  analogReadRes(12);
  analogReadAveraging(16);
  delay(100);
  Display.fillRoundRect(20, 200, 280, 30, 8, C_WHITE);
  Display.fillRoundRect(20, 200, 100, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Starting SD reader"));

  delay(100);

  isSD = sd.begin(SDCS_PIN, SD_SCK_MHZ(SPI_SPEED));

  while (!isSD) {
    Display.fillRoundRect(20, 200, 280, 30, 8, C_WHITE);
    Display.fillRoundRect(20, 200, 100, 30, 8, C_MDGREY);
    Display.setCursor(30, 203);
    Display.setTextColor(C_RED, C_MDGREY);
    Display.print(F("NO SD CARD"));
    delay(1000);
    isSD = sd.begin(SDCS_PIN, SD_SCK_MHZ(SPI_SPEED));
  }

  Root.open("/");

  Display.fillRoundRect(20, 200, 280, 30, 8, C_WHITE);
  Display.fillRoundRect(20, 200, 150, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("SD Card Found"));
  delay(500);
  Display.fillRoundRect(20, 200, 280, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Setup done"));
  delay(1500);

  // ShowInstructions();
}

/***********************************************************************************************

  Main program loop

***********************************************************************************************/

void loop() {

  MainMenu();

  GetFileName();

  PrepTest();
  StartTest = false;

  RunTest();
}

void RunTest() {

  while (1) {
    // let's back out the setup time to make stuff start at 0
    curtime = millis();

    // increment the Counter used for averaging
    // basically we are going to read values until time to display, then compute averages
    Counter++;

    // measure battery Volts
    vVolts = vVolts + (analogRead(VM_PIN));

    // measure battery Amps
    aVolts = aVolts + (analogRead(AM_PIN));

    //  LoadMass = LoadMass + LoadSensor.get_units(LoadCellReads);  // 1 is still pretty good

    if (FreqMeasure.available()) {

      RPMSum = RPMSum + FreqMeasure.read();
      RPMCount++;
    }

    if (Touch.dataAvailable()) {
      Serial.println("Line 524");
      ProcessTouch();

      if (PressIt(StartTestBtn) == true) {
        delay(100);  // debounce button?
        Display.setFont(F_A20);
        Display.setCursor(10, 10);
        Display.setTextColor(C_WHITE, C_DKBLUE);
        Display.fillRect(0, 0, 320, 40, C_DKBLUE);
        Display.print(F("Testing..."));
        //Serial.println("Testing Started");

        StartMRPM = MRPM;
        StartAmps = Amps;
        StartTorque = Torque;
        StartMotorEff = 0;
        StartVolts = Volts;
        StartLoadMass = LoadMass;
        StartMPower = MPower;

        dAmps[StartMRPM - LIMIT_L] = StartAmps;
        dTorque[StartMRPM - LIMIT_L] = StartTorque;
        dEffeciency[StartMRPM - LIMIT_L] = 100.0f * StartMotorEff;
        dVolts[StartMRPM - LIMIT_L] = StartVolts;
        dLoad[StartMRPM - LIMIT_L] = StartLoadMass;
        dMPower[StartMRPM - LIMIT_L] = StartMPower;
        dPoint[StartMRPM - LIMIT_L] = 0;

        Graph.setX(StartMRPM);
        Graph.plot(pEff, 100.0f * StartMotorEff);
        Graph.plot(pTorque, StartTorque * 10.0f);
        Graph.plot(pAmps, StartAmps);
        Graph.plot(pVolts, StartVolts);

        StartTest = true;
        Point = 0;
        AverageReads = 0;
        StartTestBtn.disable();
        StopBtn.enable();
        StopBtn.draw();
        BtnX = 0;
        BtnY = 0;
      } else if (PressIt(ReRunBtn) == true) {
        /*
        StartTest = true;
        Point = 1;
        AverageReads = 0;
        Graph.setX(StartMRPM);
        Graph.plot(pEff, 100.0f * StartMotorEff);
        Graph.plot(pTorque, StartTorque * 10.0f);
        Graph.plot(pAmps, StartAmps);
        Graph.plot(pVolts, StartVolts);
        */
        return;
      } else if (PressIt(StopBtn) == true) {
        StopBtn.disable();
        StartTestBtn.disable();
        EndTest();

        ReRunBtn.enable();
        ReRunBtn.draw();
        return;
      }
    }

    //Serial.println(UpdateTime);
    if (UpdateTime >= (unsigned long)Update) {

      UpdateTime = 0;
      if (!LoadSensor.is_ready()) {
       // Serial.println("----------------------------------------------");
      }
      ComputeData();

      DisplayData();


      if ((MRPM < LIMIT_L) && (StartTest)&& (MRPM !=0)) {
        StopBtn.disable();
        StartTestBtn.disable();
        EndTest();
        ReRunBtn.enable();
        ReRunBtn.draw();
        return;
      }

      if (StartTest) {

        if ((MRPM >= LIMIT_L) && (MRPM <= LIMIT_H) && (MotorEff >= 0.01f)) {
          // attempt to keep hysteresis out of the picture
          // and since vibrations bounce unit around only
          // record data on the decrease
          if (MRPM < (PreviousRPM)) {
            PreviousRPM = MRPM;
            dAmps[MRPM - LIMIT_L] = Amps;
            dTorque[MRPM - LIMIT_L] = Torque;
            dEffeciency[MRPM - LIMIT_L] = 100.0f * MotorEff;
            dVolts[MRPM - LIMIT_L] = Volts;
            dLoad[MRPM - LIMIT_L] = LoadMass;
            dMPower[MRPM - LIMIT_L] = MPower;
            dPoint[MRPM - LIMIT_L] = Point;

            LastAmps = Amps;
            LastTorque = Torque;
            LastEffeciency = 100.0f * MotorEff;
            LastVolts = Volts;
            LastLoad = LoadMass;
            LastMPower = MPower;
            LastPoint = Point;

            // plot the data
            Graph.setX(MRPM);
            Graph.plot(pEff, 100.0f * MotorEff);
            Graph.plot(pTorque, Torque * 10.0f);
            Graph.plot(pAmps, Amps);
            Graph.plot(pVolts, Volts);
            Point++;
          }
        }
      }


      // reset the counters
      vVolts = 0.0f;
      aVolts = 0.0f;
      thVolts = 0.0f;
      Counter = 0;

      LoadMass = 0;
      // MRPM = 0;
      RPMSum = 0;
      RPMCount = 0;
    }
  }
}

/***********************************************************************************************

  Main calculation code

***********************************************************************************************/

void ComputeData() {


  if (AverageReads == 0) {

    AverageReads = Counter;
  }
  // get car speed from wheel RPM
  MRPM = 0;

  if (RPMCount >= MINIMUM_PULSES) {
    MRPM = 60 * FreqMeasure.countToFrequency(RPMSum / RPMCount);
  }

  // get the battey voltage
  vVolts = vVolts / Counter;
  vVolts = vVolts / (4096.0f / 3.3f);
  Volts = (vVolts * VDCompensation) + VMOffset;

  // get current draw
  aVolts = aVolts / Counter;
  aVolts = aVolts / (4096.0f / 3.3f);

  Amps = (aVolts - VMid) / (mVPerAmp / 1000.0f);

  // measure temp volts
  // just one shot is good enough
  thVolts = analogRead(TH_PIN);
  thVolts = thVolts / (4096.0f / 3.3f);

  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r1
  tr2 = (thVolts * tr1) / (3.3f - thVolts);

  //equation from data sheet
  TempK = 1.0 / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
  TempF = (TempK * 1.8f) - 459.67f;
  TempF = TempF + TempOffset;

  // compute EPower
  EPower = Volts * Amps;

  //Serial.print("LoadCellCal "); Serial.println(LoadCellCal );
  //Serial.print("LoadCellOffset "); Serial.println(LoadCellOffset );

  // LoadSensor.set_LoadSensor(LoadCellCal);
  // LoadSensor.set_offset(LoadCellOffset);

  //Serial.print("683 LoadSensor.get_LoadSensor() ");
  //Serial.print(LoadSensor.get_LoadSensor());
  //Serial.print(", LoadSensor.get_offset() ");
  //Serial.println(LoadSensor.get_offset());
  //LoadSensor.set_LoadSensor(LoadCellCal);
  //LoadSensor.set_offset(LoadCellOffset);

  //LoadMass = LoadMass / Counter;

  LoadMass = LoadSensor.get_units(LoadCellReads);  // 1 is still pretty good


  LoadMass = LoadMass / 1000.0f;  // in kg

  //Serial.print("LoadMass ");
  //Serial.print(": ");
  //Serial.print(LoadMass);
  if (MotorDirection == CCW) {
    LoadMass = LoadMass * 1.0f;
  } else {
    LoadMass = LoadMass * -1.0f;
  }

  //Serial.println(LoadMass);
  //Serial.print(",converted ");
  //Serial.println(LoadMass);

  LoadForce = LoadMass * 9.80665;  // in newtons

  // downward force for CCW cases (in the direction of the load cell arrow), force is negative
  // upward force for CW cases (in the opposite direction of the load cell arrow), force is positive
  // to get positive x -1

  Torque = LoadForce * ForceArm * 0.0254;  // Nm
  MPower = (Torque * MRPM * 2.0 * 3.14159) / 60.0;

  // compute efficiency
  MotorEff = MPower / EPower;

  // Serial.println(LoadForce);
/*
  Serial.print("Amps ");
  Serial.print(Amps);
  Serial.print(", Volts ");
  Serial.print(Volts);
  Serial.print(", MRPM ");
  Serial.print(MRPM);
  Serial.print(", ForceArm ");
  Serial.print(ForceArm);
  Serial.print(", LoadMass ");
  Serial.print(LoadMass);
  Serial.print(", Torque ");
  Serial.print(Torque);
  Serial.print(", MPOWER ");
  Serial.print(MPower);
  Serial.print(", EPower ");
  Serial.print(EPower);
  Serial.print(", MotorEff ");
  Serial.print(MotorEff * 100.0, 3);
  Serial.println();
*/

  /*
  if (MRPM > StartMRPM) {

    StartMRPM = MRPM;
    StartAmps = Amps;
    StartTorque = Torque;
    StartMotorEff = 0;
    StartVolts = Volts;
    StartLoadMass = LoadMass;
    StartMPower = MPower;
  }
  */
}

void PrepTest() {

  Display.fillScreen(C_BLACK);

  // clear array
  for (i = LIMIT_L; i < (LIMIT_H - LIMIT_L); i++) {
    dAmps[i] = 0.0f;
    dTorque[i] = 0.0f;
    dEffeciency[i] = 0.0f;
    dVolts[i] = 0.0f;
    dLoad[i] = 0.0f;
    dMPower[i] = 0.0f;
    dPoint[i] = 0;
  }

  // reset conditions
  PreviousRPM = 15000;

  // reset the starting point
  // needed only for restarts
  Graph.resetStart(pEff);
  Graph.resetStart(pTorque);
  Graph.resetStart(pAmps);
  Graph.resetStart(pVolts);

  Graph.drawGraph();

  Display.fillRect(0, 0, 319, 40, C_DKBLUE);
  Display.setFont(F_A20);

  Display.setCursor(10, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);

  for (i = 0; i < 12; i++) {
    Display.print(FileName[i]);
  }

  StopBtn.disable();
  ReRunBtn.disable();
  StartTestBtn.enable();
  StartTestBtn.draw();
  UpdateTime = 0;
}

void DisplayData() {

  int Col = 282;
  int TextOffset = 12;
  // Display.fillRect(Col - 5,  50, 55 , 250, C_BLACK);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A08);

  // point
  ffPoint.setTextColor(C_GREEN, C_BLACK);
  Display.setTextColor(C_GREEN, C_BLACK);
  if (Point < REQUIRED_POINTS) {
    ffPoint.setTextColor(C_RED, C_BLACK);
    Display.setTextColor(C_RED, C_BLACK);
  }
  Display.setCursor(Col, 50);
  Display.print(F("Point"));
  Display.setFont(F_A08);
  Display.setCursor(Col + 5, 50 + TextOffset);
  ffPoint.print(Point);

  Display.setTextColor(C_WHITE, C_BLACK);
  // torque
  Display.setCursor(Col, 75);
  Display.print(F("Torque: "));
  Display.setCursor(Col + 5, 75 + TextOffset);
  ffTorque.setTextColor(C_WHITE, C_BLACK);
  ffTorque.print(Torque, 2);

  // temp
  Display.setCursor(Col, 100);
  Display.print(F("Temp: "));
  Display.setCursor(Col + 5, 100 + TextOffset);
  ffTemp.setTextColor(C_WHITE, C_BLACK);
  ffTemp.print(TempF, 1);

  // Volts
  Display.setCursor(Col, 125);
  Display.print(F("Volts: "));
  Display.setCursor(Col + 5, 125 + TextOffset);
  ffVolts.setTextColor(C_WHITE, C_BLACK);
  ffVolts.print(Volts, 1);

  // Amps
  Display.setCursor(Col, 150);
  Display.print(F("Amps: "));
  Display.setCursor(Col + 5, 150 + TextOffset);
  ffAmps.setTextColor(C_WHITE, C_BLACK);
  ffAmps.print(Amps, 1);

  // RPM
  Display.setCursor(Col, 175);
  Display.print(F("RPM: "));
  Display.setCursor(Col + 5, 175 + TextOffset);
  ffRPM.setTextColor(C_WHITE, C_BLACK);
  ffRPM.print(MRPM);

  // mass
  Display.setCursor(Col, 200);
  Display.print(F("Mass: "));
  Display.setCursor(Col + 5, 200 + TextOffset);
  ffMass.setTextColor(C_WHITE, C_BLACK);
  ffMass.print(LoadMass, 3);
}

void GetParameters() {

  // use this to force default values for new chip
  if (1 == 0) {
    EEPROM.put(10, Update);
    EEPROM.put(20, LoadCellReads);
    EEPROM.put(40, TempOffset);
    EEPROM.put(50, MotorNumber);
    EEPROM.put(60, MotorType);
    EEPROM.put(90, VMOffset);
    EEPROM.put(100, MotorDirection);
    EEPROM.put(110, VDCompensation);
    EEPROM.put(120, ForceArm);
    EEPROM.put(130, LoadCellCal);
    EEPROM.put(140, LoadCellOffset);
    EEPROM.put(220, VMid);
    EEPROM.put(230, mVPerAmp);
    EEPROM.put(240, LC_year);
    EEPROM.put(250, LC_month);
    EEPROM.put(260, LC_day);
    EEPROM.put(270, SC_year);
    EEPROM.put(280, SC_month);
    EEPROM.put(290, SC_day);
    EEPROM.put(300, LC_TestsSinceCal);
    EEPROM.put(310, SC_TestsSinceCal);
  }

  EEPROM.get(10, Update);
  EEPROM.get(20, LoadCellReads);
  EEPROM.get(40, TempOffset);
  EEPROM.get(50, MotorNumber);
  EEPROM.get(60, MotorType);
  EEPROM.get(90, VMOffset);
  EEPROM.get(100, MotorDirection);
  EEPROM.get(110, VDCompensation);
  EEPROM.get(120, ForceArm);
  EEPROM.get(130, LoadCellCal);
  EEPROM.get(140, LoadCellOffset);
  EEPROM.get(220, VMid);
  EEPROM.get(230, mVPerAmp);
  EEPROM.get(240, LC_year);
  EEPROM.get(250, LC_month);
  EEPROM.get(260, LC_day);
  EEPROM.get(270, SC_year);
  EEPROM.get(280, SC_month);
  EEPROM.get(290, SC_day);
  EEPROM.get(300, LC_TestsSinceCal);
  EEPROM.get(310, SC_TestsSinceCal);

  SetParametersForDirection();

#ifdef DO_DEBUG
  Serial.print("Update ");
  Serial.println(Update);
  Serial.print("LoadCellReads ");
  Serial.println(LoadCellReads);
  Serial.print("ForceArm ");
  Serial.println(ForceArm);
  Serial.print("LoadCellCal ");
  Serial.println(LoadCellCal);
  Serial.print("LoadCellOffset ");
  Serial.println(LoadCellOffset);
  Serial.print("TempOffset ");
  Serial.println(TempOffset);
  Serial.print("MotorNumber ");
  Serial.println(MotorNumber);
  Serial.print("MotorType ");
  Serial.println(MotorType);
  Serial.print("MotorDir ");
  Serial.println(MotorDirection);
  Serial.print("VMOffset ");
  Serial.println(VMOffset);
  Serial.print("VDCompensation ");
  Serial.println(VDCompensation);
  Serial.print("VMid ");
  Serial.println(VMid);
  Serial.print("mVPerAmp ");
  Serial.println(mVPerAmp);
  Serial.print("LC_year ");
  Serial.println(LC_year);
  Serial.print("LC_month ");
  Serial.println(LC_month);
  Serial.print("LC_day ");
  Serial.println(LC_day);
  Serial.print("SC_year ");
  Serial.println(SC_year);
  Serial.print("SC_month ");
  Serial.println(SC_month);
  Serial.print("SC_day ");
  Serial.println(SC_day);
  Serial.print("LC_TestsSinceCal ");
  Serial.println(LC_TestsSinceCal);
  Serial.print("SC_TestsSinceCal ");
  Serial.println(SC_TestsSinceCal);

#endif

  delay(10);
}

void SetParametersForDirection() {

  if (MotorDirection == CCW) {
    LineColor = C_YELLOW;
  } else if (MotorDirection == CW) {
    LineColor = C_GREEN;
  }
  Graph.setLineColor(pEff, LineColor);
  LoadSensor.set_scale(LoadCellCal);
  LoadSensor.set_offset(LoadCellOffset);

  EEPROM.put(100, MotorDirection);
}

void ShowInstructions() {

  bool KeepIn = true;
  //nothing fancy, just a header and some buttons
  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE);
  Display.setFont(F_A20);
  Display.setCursor(5, 13);
  Display.print(F("CHECK LIST"));

  Display.setFont(F_A16);

  Display.setTextColor(C_WHITE, C_BLACK);

  Display.setCursor(5, 50);
  Display.print(F("1. Is the brake clear?"));
  Display.setCursor(5, 80);
  Display.print(F("2. Is the load sensor calibrated?"));
  Display.setCursor(5, 110);
  Display.print(F("3. Is the disk secure to the hub?"));
  Display.setCursor(5, 140);
  Display.print(F("4. Is the load cell connected?"));


  MenuDoneBtn.draw();

  while (KeepIn) {

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(MenuDoneBtn)) {
        KeepIn = false;
      }
    }
  }
  delay(100);
}


void WaitForPress() {

  while (!Touch.dataAvailable()) {
    delay(50);
  }
}

void WaitForRelease() {

  while (Touch.dataAvailable()) {
    delay(50);
  }
}

void MainMenu() {

  bool KeepIn = true;

  DrawMainMenuScreen();

  while (KeepIn) {

    // if touch screen pressed handle it
    delay(50);
    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(ProfileBtn) == true) {
        KeepIn = false;
      }
      if (PressIt(SensorsBtn) == true) {

        CalibrateSensors();
        DrawMainMenuScreen();
      }
      if (PressIt(ConnectionBtn) == true) {

        Connections();
        DrawMainMenuScreen();
      }
      if (PressIt(ReviewBtn) == true) {
        ShowResultImages();
        DrawMainMenuScreen();
      }
    }
  }
}

/*

  service function UI screen

*/
void DrawMainMenuScreen() {

  //nothing fancy, just a header and some buttons
  Display.fillScreen(C_BLACK);

  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE);
  Display.setFont(F_A20);
  Display.setCursor(5, 13);
  Display.print(F("Main Menu"));

  ProfileBtn.draw();
  SensorsBtn.draw();
  ConnectionBtn.draw();
  ReviewBtn.draw();
}

void CalibrateSensors() {

  bool LCalDone = false;
  bool SCalDone = false;
  int EditMenuOption = 1;

  RPMSum = 0;
  RPMCount = 0;

  CalTime = 0;

  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 205, 320, 40, C_LTGREY);
  // draw headers
  Display.setTextColor(C_BLACK, C_LTGREY);
  Display.setFont(F_A12B);

  Display.setCursor(5, 210);
  Display.print(F("Volts: "));

  Display.setCursor(70, 210);
  Display.print(F("Amps: "));

  Display.setCursor(135, 210);
  Display.print(F("Temp: "));

  Display.setCursor(200, 210);
  Display.print(F("RPM: "));

  Display.setCursor(260, 210);
  Display.print(F("Load: "));

  Counter = 0;
  aVolts = 0.0;
  aVolts = analogRead(AM_PIN);
  Counter++;

  ComputeData();

  sprintf(buf, "Amp offset (%.3f)", aVolts);
  OptionMenu.setItemText(MenuOption9, buf);

  sprintf(buf, "Cal. at 0 kg: %ld", (int32_t)LoadCellCal);
  OptionMenu.setItemText(MenuOption2, buf);
  sprintf(buf, "Cal. at 1 kg: %ld", (int32_t)LoadCellOffset);
  OptionMenu.setItemText(MenuOption3, buf);

  OptionMenu.draw();

  while (EditMenuOption != 0) {

    vVolts = vVolts + analogRead(VM_PIN);
    aVolts = aVolts + analogRead(AM_PIN);
    Counter++;

    if (FreqMeasure.available()) {
      RPMSum = RPMSum + FreqMeasure.read();
      RPMCount++;
    }

    if (CalTime > 250) {
      CalTime = 0;

      Display.setFont(F_A12);
      SetParametersForDirection();
      ComputeData();

      Display.setFont(F_A12);

      Display.setCursor(5, 226);
      ffVolts.setTextColor(C_BLACK, C_LTGREY);
      ffVolts.print(Volts, 2);

      Display.setCursor(70, 226);
      ffAmps.setTextColor(C_BLACK, C_LTGREY);
      ffAmps.print(Amps, 2);

      Display.setCursor(135, 226);
      ffTemp.setTextColor(C_BLACK, C_LTGREY);
      ffTemp.print(TempF, 1);

      Display.setCursor(200, 226);
      ffRPM.setTextColor(C_BLACK, C_LTGREY);
      ffRPM.print(MRPM);

      Display.setCursor(260, 226);
      ffCalLoad.setTextColor(C_BLACK, C_LTGREY);
      ffCalLoad.print(LoadMass, 3);

      vVolts = 0.0;
      aVolts = 0.0;
      thVolts = 0.0;
      Counter = 0;
      //MRPM = 0;
      RPMSum = 0;
      RPMCount = 0;
    }

    if (Touch.dataAvailable()) {

      ProcessTouch();
      Click();
      if (!OptionMenu.isEditing()) {
        WaitForRelease();
      }

      EditMenuOption = OptionMenu.press(BtnX, BtnY);

      OptionMenu.drawRow(EditMenuOption);
      if (EditMenuOption == MenuOption1) {
        ForceArm = OptionMenu.value[MenuOption1];
        LCalDone = true;
      } else if (EditMenuOption == MenuOption2) {
        WaitForRelease();
        LCalDone = true;
        LoadSensor.tare();
        delay(100);
        LoadCellCal = LoadSensor.get_scale();
        LoadCellOffset = LoadSensor.get_offset();

        LoadSensor.set_scale(LoadCellCal);
        LoadSensor.set_offset(LoadCellOffset);

        sprintf(buf, "Cal. at 0 kg: %ld", (int32_t)LoadCellCal);
        OptionMenu.setItemText(MenuOption2, buf);
        sprintf(buf, "Cal. at 1 kg: %ld", (int32_t)LoadCellOffset);
        OptionMenu.setItemText(MenuOption3, buf);
        OptionMenu.drawRow(MenuOption2);
        OptionMenu.drawRow(MenuOption3);
      } else if (EditMenuOption == MenuOption3) {
        WaitForRelease();
        LCalDone = true;
        LoadSensor.calibrate_scale(1000, 5);
        delay(100);
        LoadCellCal = LoadSensor.get_scale();
        LoadCellOffset = LoadSensor.get_offset();

        LoadSensor.set_scale(LoadCellCal);
        LoadSensor.set_offset(LoadCellOffset);


        sprintf(buf, "Cal. at 0 kg: %ld", (int32_t)LoadCellCal);
        OptionMenu.setItemText(MenuOption2, buf);
        sprintf(buf, "Cal. at 1 kg: %ld", (int32_t)LoadCellOffset);
        OptionMenu.setItemText(MenuOption3, buf);
        OptionMenu.drawRow(MenuOption2);
        OptionMenu.drawRow(MenuOption3);
      }
      if ((EditMenuOption == MenuOption7) || (EditMenuOption == MenuOption8) || (EditMenuOption == MenuOption9) || (EditMenuOption == MenuOption10)) {
        WaitForRelease();
        VDCompensation = OptionMenu.value[MenuOption8];
        VMid = OptionMenu.value[MenuOption9];
        mVPerAmp = OptionMenu.value[MenuOption10];
        VMOffset = OptionMenu.value[MenuOption7];
        SCalDone = true;
      } else if (EditMenuOption == MenuOption12) {
        MotorDirection = (int)OptionMenu.value[MenuOption12];
      }
    }
  }

  MotorDirection = (int)OptionMenu.value[MenuOption12];
  Update = (int)OptionMenu.value[MenuOption5];
  LoadCellReads = (int)OptionMenu.value[MenuOption4];
  TempOffset = OptionMenu.value[MenuOption11];
  VDCompensation = OptionMenu.value[MenuOption8];
  VMid = OptionMenu.value[MenuOption9];
  mVPerAmp = OptionMenu.value[MenuOption10];
  VMOffset = OptionMenu.value[MenuOption7];

  EEPROM.put(10, Update);
  EEPROM.put(20, LoadCellReads);
  EEPROM.put(40, TempOffset);
  EEPROM.put(90, VMOffset);
  EEPROM.put(100, MotorDirection);
  EEPROM.put(110, VDCompensation);
  EEPROM.put(120, ForceArm);
  EEPROM.put(130, LoadCellCal);
  EEPROM.put(140, LoadCellOffset);
  EEPROM.put(220, VMid);
  EEPROM.put(230, mVPerAmp);

  if (LCalDone) {
    EEPROM.put(240, year());
    EEPROM.put(250, month());
    EEPROM.put(260, day());
    EEPROM.put(300, 0);  // SC_TestsSinceLCCal
  }
  if (SCalDone) {
    EEPROM.put(270, year());
    EEPROM.put(280, month());
    EEPROM.put(290, day());
    EEPROM.put(310, 0);  // SC_TestsSinceLCCal
  }

  SetParametersForDirection();

  RPMSum = 0;
  RPMCount = 0;

  delay(10);
}
void Connections() {

  bool KeepIn = true;

  Display.fillScreen(C_BLACK);

  Display.fillRect(0, 0, 320, 50, C_DKBLUE);

  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(5, 13);

  Display.print(F("Wiring Load Cell"));

  Display.setFont(F_A16);
  Display.setTextColor(C_WHITE);

  Display.drawRect(20, 70, 280, 160, C_WHITE);  // outer case

  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A20);
  Display.setCursor(145, 180);
  Display.print(F("TOP"));

  Display.drawRect(30, 90, 100, 70, C_WHITE);    // left USB
  Display.drawLine(30, 125, 128, 125, C_WHITE);  // left USB

  Display.drawRect(180, 90, 105, 70, C_WHITE);    // righ USB
  Display.drawLine(180, 125, 283, 125, C_WHITE);  // left USB

  Display.setTextColor(C_CYAN);
  Display.setCursor(40, 170);
  Display.print(F("Upper"));
  Display.setTextColor(C_GREEN);
  Display.setCursor(40, 195);
  Display.print(F("Lower"));

  Display.setTextColor(C_CYAN);
  Display.setCursor(40, 98);
  Display.print(F("Temp"));
  Display.setTextColor(C_GREEN);
  Display.setCursor(40, 130);
  Display.print(F("Amp"));

  Display.setTextColor(C_CYAN);
  Display.setCursor(188, 98);
  Display.print(F("Load"));

  Display.setTextColor(C_GREEN);
  Display.setCursor(188, 130);
  Display.print(F("Speed"));

  MenuDoneBtn.draw();

  while (KeepIn) {
    delay(50);
    if (Touch.dataAvailable()) {
      ProcessTouch();
      if (PressIt(MenuDoneBtn)) {
        KeepIn = false;
      }
    }
  }
}

void PrintSuffix() {

  DataFile.print(F("_"));
  DataFile.print(FileName[0]);
  DataFile.print(FileName[1]);
  DataFile.print(FileName[2]);
  DataFile.print(FileName[3]);
  DataFile.print(FileName[4]);
  DataFile.print(F("_"));
  DataFile.print(FileName[6]);
  DataFile.print(FileName[7]);
  DataFile.print(FileName[8]);
}

void EndTest() {

  // assume test was completed so bump the counters

  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(10, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Test Complete."));

  LC_TestsSinceCal++;
  SC_TestsSinceCal++;
  EEPROM.put(300, LC_TestsSinceCal);
  EEPROM.put(310, SC_TestsSinceCal);

  delay(100);

  if (Point < REQUIRED_POINTS) {
    StartTest = false;
    Display.setFont(F_A16);
    Display.setTextColor(C_WHITE, C_BLACK);
    Display.fillRect(50, 35, 220, 170, C_RED);
    Display.drawRect(50, 35, 220, 170, C_WHITE);
    Display.setCursor(60, 55);
    Display.print(F("INVALID TEST"));
    Display.setCursor(60, 85);
    Display.print(F("Data points is < "));
    Display.print(REQUIRED_POINTS);
    Display.setCursor(60, 115);
    Display.print(F("Points collected: "));
    Display.print(Point);
    Display.setCursor(60, 145);
    Display.print(F("Run the test slower."));
    Display.setCursor(60, 175);
    Display.print(F("Retracting, wait."));
    Display.fillRect(51, 166, 218, 30, C_RED);
    Display.setCursor(60, 175);
    Display.print(F("Press to continue."));

    while (1) {
      delay(50);
      if (Touch.dataAvailable()) {
        Click();

        WaitForRelease();
        break;
      }
    }
    return;
  }

  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(10, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Saving data..."));


  Display.setFont(F_A12);
  Display.setCursor(270, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(268, 2, 50, 35, C_DKBLUE);
  Display.print(F("1401"));

  StartTest = false;

  isSD = DataFile.open(FileName, O_WRITE | O_CREAT);

  if (!isSD) {
    isSD = sd.begin(SDCS_PIN, SD_SCK_MHZ(SPI_SPEED));
    isSD = DataFile.open(FileName, O_WRITE | O_CREAT);
    if (!isSD) {
      Display.setFont(F_A20);
      Display.setCursor(10, 10);
      Display.setTextColor(C_WHITE, C_DKBLUE);
      Display.fillRect(0, 0, 320, 40, C_DKBLUE);
      Display.print(F("Saving failed..."));
    }
  }
  // write date info to file
  DataFile.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());
  DataFile.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
  DataFile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());

  // print header
  DataFile.print(F("RPM"));
  DataFile.write(44);

  DataFile.print(F("Volts"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("Amps"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("Load"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("Torque"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("Efficiency"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("E-Power"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("M-Power"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.print(F("Point"));
  PrintSuffix();
  DataFile.write(44);

  DataFile.write(44);

  DataFile.print(F("_"));
  DataFile.print(F("Counter"));
  PrintSuffix();
  DataFile.write(44);
  DataFile.write(44);

  DataFile.print(F("PATRIOT RACING - DYNAMOMETER TEST"));

  DataFile.println(F(""));

  // dump stuff from array to SD card

  DataFile.print(LIMIT_L);
  DataFile.write(44);
  DataFile.print(LastVolts, 2);
  DataFile.write(44);
  DataFile.print(LastAmps, 3);
  DataFile.write(44);
  DataFile.print(LastLoad, 3);
  DataFile.write(44);
  DataFile.print(LastTorque, 4);
  DataFile.write(44);
  DataFile.print(LastEffeciency, 2);
  DataFile.write(44);
  DataFile.print(LastVolts * LastAmps, 0);
  DataFile.write(44);
  DataFile.print(LastMPower, 2);
  DataFile.write(44);
  DataFile.print(LastPoint);
  DataFile.write(44);

  DataFile.println("");
  Display.setFont(F_A12);
  Display.setCursor(270, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(268, 2, 50, 35, C_DKBLUE);
  Display.print(F("1493"));
  Display.fillRect(268, 2, 50, 35, C_DKBLUE);
  // now print the array of data
  for (i = 0; i < (LIMIT_H - LIMIT_L); i++) {

    DataFile.println("");
    Display.setFont(F_A12);
    Display.setCursor(270, 10);
    ffPercent.setTextColor(C_WHITE, C_DKBLUE);

    //Display.fillRect(268, 2, 50, 35, C_DKBLUE);
    ffPercent.print((i * 100) / (LIMIT_H - LIMIT_L));

    DataFile.print(i + LIMIT_L + 1);  // this is RPM, add 1 since we already printed the first point
    DataFile.write(44);

    if (dVolts[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dVolts[i], 2);
    }
    DataFile.write(44);

    if (dAmps[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dAmps[i], 3);
    }
    DataFile.write(44);

    if (dLoad[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dLoad[i], 3);
    }
    DataFile.write(44);

    if (dTorque[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dTorque[i], 4);
    }
    DataFile.write(44);

    if (dEffeciency[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dEffeciency[i], 2);
    }
    DataFile.write(44);

    if (dEffeciency[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dVolts[i] * dAmps[i], 0);
    }
    DataFile.write(44);

    if (dMPower[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dMPower[i], 2);
    }
    DataFile.write(44);

    if (dPoint[i] == 0) {
      DataFile.print(F(""));
    } else {
      DataFile.print(dPoint[i]);
    }

    DataFile.write(44);

    if (i < 15) {

      DataFile.write(44);
      DataFile.write(44);
      switch (i) {

        case 0:
          DataFile.print(F("Load sensor calibration date: "));
          DataFile.write(44);
          DataFile.print(LC_month);
          DataFile.write(47);
          DataFile.print(LC_day);
          DataFile.write(47);
          DataFile.print(LC_year);
          DataFile.write(44);
          DataFile.print(F("Tests since load cell calibration: "));
          DataFile.write(44);
          DataFile.print(LC_TestsSinceCal);
          break;

        case 1:
          DataFile.print(F("Sensor calibration date: "));
          DataFile.write(44);
          DataFile.print(SC_month);
          DataFile.write(47);
          DataFile.print(SC_day);
          DataFile.write(47);
          DataFile.print(SC_year);
          DataFile.write(44);
          DataFile.print(F("Tests since sensor calibration: "));
          DataFile.write(44);
          DataFile.print(SC_TestsSinceCal);
          break;

        case 2:
          DataFile.print(F("Code v.: "));
          DataFile.write(44);
          DataFile.print(CODE_VERSION);
          DataFile.write(44);
          DataFile.print(F("Date: "));
          DataFile.write(44);
          DataFile.print(month());
          DataFile.print(F(" / "));
          DataFile.print(day());
          DataFile.print(F(" / "));
          DataFile.print(year());
          DataFile.write(44);
          DataFile.print(hour());
          DataFile.print(F(": "));
          if (minute() < 10) {
            DataFile.print(F("0"));
          }
          DataFile.print(minute());
          break;

        case 3:
          DataFile.print(F("Motor MFG: "));
          DataFile.write(44);
          if (FileName[0] == 'U') {
            DataFile.print(F("UK"));
          } else {
            DataFile.print(F("Chinese"));
          }

          DataFile.write(44);
          DataFile.print(F("Rotation: "));
          if (MotorDirection == CCW) {
            DataFile.print(F("CCW (original direction)"));
          } else {
            DataFile.print(F("CW (reverse direction))"));
          }

          DataFile.write(44);
          DataFile.print(F("Motor number: "));
          DataFile.write(44);
          DataFile.print(FileName[1]);
          DataFile.write(44);
          DataFile.print(F("Motor Temp [deg F]: "));
          DataFile.write(44);
          DataFile.print(TempF, 1);
          break;
        case 4:
          DataFile.print(F("Update time [ms]: "));
          DataFile.write(44);
          DataFile.print(Update);
          DataFile.write(44);
          DataFile.print(F("Averages: "));
          DataFile.write(44);
          DataFile.print(LoadCellReads);
          DataFile.write(44);
          DataFile.print(F("Data Averages: "));
          DataFile.write(44);
          DataFile.print(AverageReads);
          break;
        case 5:
          DataFile.print(F("Force arm [in]: "));
          DataFile.write(44);
          DataFile.print(ForceArm, 3);
          DataFile.write(44);
          DataFile.print(F("Load cell cal: "));
          DataFile.write(44);
          DataFile.print(LoadCellCal);
          DataFile.write(44);
          DataFile.print(F("Load cell offset: "));
          DataFile.write(44);
          DataFile.print(LoadCellOffset);
          break;
        case 6:
          DataFile.print(F("Voltage sensor offset: "));
          DataFile.write(44);
          DataFile.print(VMOffset, 2);
          DataFile.write(44);
          DataFile.print(F("Voltage divider: "));
          DataFile.write(44);
          DataFile.print(VDCompensation, 2);
          break;
        case 7:
          DataFile.print(F("Current sensor offset: "));
          DataFile.write(44);
          DataFile.print(VMid, 4);
          DataFile.write(44);
          DataFile.print(F("Current sensor mVPerAmp: "));
          DataFile.write(44);
          DataFile.print(mVPerAmp, 4);
          break;
        case 8:
          DataFile.print(F("Temp Offset: "));
          DataFile.write(44);
          DataFile.print(TempOffset);
          break;
        case 9:
          DataFile.print(F("Data points: "));
          DataFile.write(44);
          DataFile.print(Point);

          break;
      }
    }

    DataFile.println("");
  }


  DataFile.println("");
  Display.setFont(F_A12);
  Display.setCursor(270, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(268, 2, 50, 35, C_DKBLUE);
  Display.print(F("1706"));

  delay(20);
  DataFile.close();
  delay(200);

  Display.setFont(F_A20);
  Display.setCursor(10, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Saving BMP..."));

  delay(2000);

  // "MNDD_TTT_RR.CSV"
  // reset the file extension
  FileName[13] = 'b';
  FileName[14] = 'm';
  FileName[15] = 'p';

  Display.setFont(F_A20);
  Display.setCursor(30, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  for (i = 0; i < 12; i++) {
    Display.print(FileName[i]);
  }

  SaveBMP24(&Display, SDCS_PIN, FileName, SPI_SPEED);
  delay(100);

  Display.setFont(F_A20);
  Display.setCursor(10, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Test Complete"));
  delay(100);
  Display.setFont(F_A20);
  Display.setCursor(30, 10);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);

  for (i = 0; i < 12; i++) {
    Display.print(FileName[i]);
  }

  Point = 0;
  AverageReads = 0;

  WaitForPress();
}

void ProcessTouch() {

  Touch.read();

  BtnX = Touch.getX();
  BtnY = Touch.getY();
  /*
#ifdef DO_DEBUG
  Serial.print("real coordinates: ");
  Serial.print(BtnX);
  Serial.print(", ");
  Serial.println(BtnY);
#endif
*/

  BtnX = map(BtnX, 0, 235, 0, 320);
  BtnY = map(BtnY, 20, 396, 0, 240);

#ifdef DO_DEBUG
  Display.fillCircle(BtnX, BtnY, 3, C_RED);
#endif
  /*
#ifdef DO_DEBUG
  Serial.print(", Mapped coordinates: ");
  Serial.print(BtnX);
  Serial.print(", ");
  Serial.println(BtnY);

#endif
*/
}

bool PressIt(Button TheButton) {

  if (TheButton.press(BtnX, BtnY)) {
    Click();
    TheButton.draw(B_PRESSED);
    while (Touch.dataAvailable()) {
      delay(50);
      if (TheButton.press(BtnX, BtnY)) {
        TheButton.draw(B_PRESSED);
      } else {
        TheButton.draw(B_RELEASED);
        return false;
      }
      ProcessTouch();
    }
    TheButton.draw(B_RELEASED);
    //delay(100);
    return true;
  }
  //delay(100);
  return false;
}

void ShowResultImages() {

  FileCount = 0;
  uint16_t FileNumber = 0;

  Display.fillScreen(C_BLACK);
  Display.setFont(F_A20);
  Display.setCursor(10, 80);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.print(F("Reading files: "));

  Root.rewind();

  // get file count
  while (FileFile.openNext(&Root, O_RDONLY)) {
    FileFile.getName(FileName, sizeof(FileName));

    if ((FileName[13] == 'b') && (FileName[14] == 'm') && (FileName[15] == 'p') && (!FileFile.isDirectory())) {

      FileCount++;
      Display.setCursor(185, 80);
      Display.fillRect(184, 77, 100, 50, C_BLACK);
      Display.print(FileCount);
    }

    FileFile.close();
    delay(100);
  }
  delay(100);
  Root.rewind();

  // show first file
  if (FileCount > 0) {
    Display.setCursor(10, 110);
    Display.setTextColor(C_WHITE, C_BLACK);
    Display.print(F("Opening file"));
    FileNumber = 1;
    ShowFile(FileNumber);
  }

  // enter loop to display previous / next / exit
  while (true) {
    ProcessTouch();
    if ((BtnY < 50) && (BtnX > 0) && (BtnX < 50)) {
      FileNumber--;
      if (FileNumber == 0) {
        FileNumber = FileCount;
      }
      Click();
      ShowFile(FileNumber);
    }
    if ((BtnY < 50) && (BtnX > 270) && (BtnX < 320)) {
      FileNumber++;
      if (FileNumber > FileCount) {
        FileNumber = 1;
      }
      Click();
      ShowFile(FileNumber);
    }
    if ((BtnY < 50) && (BtnX > 50) && (BtnX < 270)) {
      Click();
      break;
    }
  }
}


void ShowFile(uint16_t DrawFileNumber) {

  uint16_t FileNumber = 0;

  Display.setFont(F_A20);

  Display.setTextColor(C_WHITE, C_DKGREY);
  Display.fillRect(40, 50, 240, 140, C_DKGREY);
  Display.drawRect(40, 50, 240, 140, C_WHITE);

  Display.setCursor(50, 70);
  Display.print(F("Retrieving: #"));
  Display.print(DrawFileNumber);



  Root.rewind();

  while (FileFile.openNext(&Root, O_RDONLY)) {

    FileFile.getName(FileName, sizeof(FileName));

    if ((FileName[13] == 'b') && (FileName[14] == 'm') && (FileName[15] == 'p') && (!FileFile.isDirectory())) {
      FileNumber++;

      if (DrawFileNumber == FileNumber) {

        Display.setFont(F_A16);

        Display.setCursor(50, 110);
        Display.print(FileName);
        delay(500);
        if (!DrawBMP24(&Display, SDCS_PIN, FileName, SPI_SPEED)) {
          Display.setFont(F_A20);
          Display.setCursor(10, 210);
          Display.setTextColor(C_RED, C_BLACK);
          Display.fillRect(0, 200, 320, 40, C_BLACK);
          Display.print(F("File Corrupt...."));
        }
        Display.fillTriangle(5, 20, 25, 5, 25, 35, C_GREEN);
        Display.fillTriangle(315, 20, 295, 5, 295, 35, C_GREEN);

        break;
      }
    }
  }

  FileFile.close();

  delay(100);
}

void GetFileName() {

  bool FNKeepIn = true;

  DrawFileNameScreen();

  vVolts = 0.0;
  aVolts = 0.0;
  thVolts = 0.0;
  Counter = 0;
  MRPM = 0;
  RPMSum = 0;
  RPMCount = 0;

  MRPM = 0;
  FileCount = 0;

  delay(100);

  Root.rewind();

  while (FileFile.openNext(&Root, O_RDONLY)) {
    FileCount++;
    FileFile.close();
  }


  delay(100);

  if (MotorType == CHINESE_MOTOR) {
    FileName[0] = 'C';
  } else {
    FileName[0] = 'U';
  }

  FileName[1] = MotorNumber + '0';

  if (MotorDirection == CCW) {
    FileName[2] = 'C';
    FileName[3] = 'C';
    FileName[4] = 'W';
  } else {
    FileName[2] = '_';
    FileName[3] = 'C';
    FileName[4] = 'W';
  }

  FileName[10] = (int)((FileCount / 10) % 10) + '0';
  FileName[11] = (int)(FileCount % 10) + '0';
  FileName[13] = 'c';
  FileName[14] = 's';
  FileName[15] = 'v';

  while (FNKeepIn) {

    curtime = millis();
    vVolts = vVolts + (analogRead(VM_PIN));
    aVolts = aVolts + (analogRead(AM_PIN));
    Counter++;

    if (UpdateTime >= 500) {

      ComputeData();

      if (abs(LoadMass) > 0.05) {
        SetTareBtn.setColors(C_RED, C_RED, C_WHITE, C_BLACK, C_GREY, C_GREY);
      } else {
        SetTareBtn.setColors(C_GREY, C_GREY, C_BLACK, C_BLACK, C_GREY, C_GREY);
      }
      sprintf(buf, "%.2f", LoadMass);
      SetTareBtn.setText(buf);
      SetTareBtn.draw();

      FileName[6] = ((int)(TempF / 100)) + '0';
      FileName[7] = ((int)(TempF / 10) % 10) + '0';
      FileName[8] = ((int)(TempF - (int)(TempF / 10) * 10)) + '0';

      //ffFileName.setTextColor(C_WHITE, C_DKBLUE);
      Display.setFont(F_A20);
      Display.setCursor(100, 13);
      Display.setTextColor(C_WHITE, C_DKBLUE);
      Display.fillRect(90, 0, 220, 40, C_DKBLUE);

      for (i = 0; i < 12; i++) {
        Display.print(FileName[i]);
      }

      // reset the counters
      vVolts = 0.0;
      aVolts = 0.0;
      thVolts = 0.0;
      Counter = 0;
      UpdateTime = 0;
      MRPM = 0;
      RPMSum = 0;
      RPMCount = 0;
    }

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(StartBtn) == true) {
        FNKeepIn = false;
      }

      if (PressIt(SetTareBtn) == true) {

        //Serial.println(LoadSensor.get_LoadSensor());
        LoadSensor.tare();
        LoadSensor.set_scale(LoadSensor.get_scale());
        //Serial.println(LoadSensor.get_LoadSensor());
      }

      if (PressIt(DirBtn) == true) {
        // reverse the directioon
        if (MotorDirection == CCW) {
          MotorDirection = CW;
          FileName[2] = '_';
          FileName[3] = 'C';
          FileName[4] = 'W';
        } else {
          MotorDirection = CCW;
          FileName[2] = 'C';
          FileName[3] = 'C';
          FileName[4] = 'W';
        }

        SetParametersForDirection();
      }
      if (PressIt(PreviousNumberBtn) == true) {

        if (MotorNumber == 0) {
          MotorNumber = 10;
        }
        MotorNumber--;
        FileName[1] = MotorNumber + '0';
      }

      if (PressIt(NextNumberBtn) == true) {
        MotorNumber++;
        if (MotorNumber > 9) {
          MotorNumber = 0;
        }
        FileName[1] = MotorNumber + '0';
      }
      if (PressIt(MotorTypeBtn) == true) {
        if (FileName[0] == 'U') {
          FileName[0] = 'C';
          MotorType = 1;
        } else {
          FileName[0] = 'U';
          MotorType = 0;
        }
      }
    }
  }

  EEPROM.put(50, MotorNumber);
  EEPROM.put(60, MotorType);
}

/*

  service function to write the filename screen

*/

void DrawFileNameScreen() {

  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(5, 13);
  Display.println(F("File: "));

  // for manufacture
  Display.drawLine(110, 60, 110, 200, C_WHITE);   // vert
  Display.drawLine(110, 200, 220, 200, C_WHITE);  // horiz

  // for motor number
  Display.drawLine(130, 60, 130, 140, C_WHITE);   // vert
  Display.drawLine(130, 140, 220, 140, C_WHITE);  // horiz

  // for direction number
  Display.drawLine(160, 60, 160, 80, C_WHITE);  // vert
  Display.drawLine(160, 80, 220, 80, C_WHITE);  // horiz

  MotorTypeBtn.draw();
  DirBtn.draw();
  PreviousNumberBtn.draw();
  NextNumberBtn.draw();

  StartBtn.draw();
  SetTareBtn.draw();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600;  // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if (pctime < DEFAULT_TIME) {  // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L;                // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void drawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, uint8_t w, uint8_t h, uint16_t color) {

  uint8_t sbyte = 0;
  uint8_t uint8_tWidth = 0;
  int jj, ii;

  uint8_tWidth = (w + 7) / 8;

  for (jj = 0; jj < h; jj++) {
    for (ii = 0; ii < w; ii++) {
      if (ii & 7) sbyte <<= 1;
      else sbyte = pgm_read_byte(bitmap + jj * uint8_tWidth + ii / 8);
      if (sbyte & 0x80) Display.drawPixel(x + ii, y + jj, color);
    }
  }
}

void Click() {

  analogWriteFrequency(BUZZ_PIN, 700);
  analogWrite(BUZZ_PIN, 127);
  delay(5);
  analogWrite(BUZZ_PIN, 0);
  delay(5);
}

///////////////////////////////////////////////////
