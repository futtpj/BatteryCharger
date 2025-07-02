// Arduino standard libraries
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Custom user created libraries
#include <DigitalInputDebounce.h>
#include "PIDController.h"
#include <AnalogInputs.h>

//    Name:       BatteryCharger.ino
//    Created:	 22-June-2023 06:40
//    Author:     KEPLER\futtp

String Version = "02-00-03-01";

/*===============================================================================
                    Common Character equivalents
===============================================================================*/
const char comma = 44;
const char UScore = 45;

/*===============================================================================
                      Mega - Input digital pin labels
===============================================================================*/
#define enc_a 2 //Rotary encoder input pin
#define enc_b 3 //Rotary encoder input pin
#define LCDPagePBpin 4
#define BatteryType_pin 5
#define Spare4 6
#define SaveSettings_pin 7
#define SetRunPin 8
#define Spare1 9
#define Spare3 10

/*===============================================================================
                       Mega - digital Output pins
===============================================================================*/
constexpr bool UsePullUp = true;
constexpr bool NoPullup = false;
#define PWMpin  12              //output to High side Mosfet power controller
#define TestLed  11            // used for misc tests
#define Arduino_ledPin  13  //Used for testing

                          // SPI Setup

//#define Mega_chipSelect 53  //Chip select (blue)
//#define Mega_SS 53          //Same as CS
//#define Mega_MOSI 51        //Master out slave in
//#define Mega_MISO 50        //Master in slave out
//#define Mega_SCK 52         //Serial clock

//#define Nano_chipSelect 14  //Chip select (blue)
//#define Nano_SS 14          //Same as CS
//#define Nano_MOSI 11        //Master out slave in
//#define Nano_MISO 12        //Master in slave out
//#define Nano_SCK 13         //Serial clock

/*===============================================================================
               SD Setup for MEGA
===============================================================================*/
//File ChargerSetup;
//File EventsLog;
//File ERRORSLog;
//String ChargerSetupFileName = "ChargerSetup.csv";
//String EventsFileName = "Events.txt";
//String ERRORSFileName = "ERRORS.txt";

/*===============================================================================
                          Debounced Digital I/O
===============================================================================*/
volatile bool LCDPagePBState;   //current debounced input value
volatile bool BatteryTypeState;
volatile bool SaveSettingsState;
volatile bool RequestedChargerMode;      //current debounced input value
/*===============================================================================
       All Analog inputs declarations
===============================================================================*/
bool NoFilter = true;
bool UseFilter = false;
//constexpr auto LRL = 0; //Lower range limit index
//constexpr auto URL = 1; //Upper range limit index
bool m = 0; //Gain   -----   Calc to be done is y=mx+b
bool b = 1; //Offset
float TotalFaults = 0;
bool DeltaDetected = 1;
//Arrays needed by ReadAnalogInputs() function
int  Battery_Volts = 0;
int  Battery_Amps = 1;
int  Battery_Temperature = 2;

int UintAnalogInputPins[] = { A0, A1, A2 };  //Assigns a pin number to a place in the array
float fltAIN_LRL[] = { 0, 0, 0 };       //LRL for analog inputs
float fltAIN_URL[] = { 16, 10, 50 };    //URL for analog inputs
float AIN_Eng_Gain[] = { 0.015640274, 0.009775171, 0.048875855 };  //must be same dim as A_Inputs_to_Read
float AIN_Eng_Offset[] = { 0.0, 0.0, 0.0 };                       //must be same dim as A_Inputs_to_Read
float fltInput_UpdateDeltas[] = { 0.1, 0.2, 1.0 };  //change in input values that force update to SD
const unsigned int NumberOfArrayElements = sizeof(UintAnalogInputPins) / sizeof(UintAnalogInputPins[0]);
const unsigned int LastArrayElementIndex = NumberOfArrayElements - 1;
int Raw_New[NumberOfArrayElements]; //fresh input from A/D in A2D units (counts)
float fltAIN_New[NumberOfArrayElements]; //fresh input from A/D but in floating point and eng units
float fltAIN_Current[NumberOfArrayElements]; //Storage for input values  after filtering - what is actually used and displayed
String AnalogInputNames[NumberOfArrayElements] = { "Volts   ", "Amps    ", "Temp C  " };  //Assigns a pin number to a place in the array
float InputFilterTC[] = { 1, 1, 1 };
/*===============================================================================
      Alarms
===============================================================================*/
bool AmpsInAlarm = false;
String AmpAlarmMessage = "   ";
String VoltAlarmMessage = "   ";
/*===============================================================================
       Now make pointers to the various arrays so thye information can be accessed by both the main
       programs and by the library calls
===============================================================================*/ 
//{ "xxxxxxxx", "xxxxxxxx", "xxxxxxxx" };
int* pUintAnalogInputPins = UintAnalogInputPins; //means byte - intended to make this varaible cross platform
int* pRaw_New = Raw_New;
float* pfltAIN_New = fltAIN_New;
float* pfltAIN_Current = fltAIN_Current;
float* pfltAIN_LRL = fltAIN_LRL;
float* pfltAIN_URL = fltAIN_URL;
float* pAIN_Eng_Gain = AIN_Eng_Gain;
float* pAIN_Eng_Offset = AIN_Eng_Offset;
float* pfltInput_UpdateDeltas = fltInput_UpdateDeltas;
float* pInputFilterTC= InputFilterTC;

/*===============================================================================
Battery Setup
===============================================================================*/
//Battery types
const int SLA_Battery_cint = 00;
const int GEL_Battery_cint = 01;
const int AGM_Battery_cint = 02;
const int FLooded_Battery_cint = 03;
const int LithFePo_Battery_cint = 04;
const int NoBatterySelected_cint = 05;
//Charger stages
const int bulk_Stage_cint = 0;  //Bulk charging
const int absorb_Stage_cint = 1;//Absorption charging
const int float_Stage_cint = 2; //Float charging
String StageName[] = { "Bulk      ","Absorb    ","Float     ","Asses     "};
//                     "123456789 " "123456789 " "123456789 ","123456789 "
//Battery Profiles - indexs to items in battery profile arrays
const int Bulk_V_cint = 0;
const int Absorb_V_cint = 1;
const int Float_V_cint = 2;
const int TempCoef_cint = 3;
const int Volts_100PC_cint = 4;
const int Volts_80PC_cint = 5;
const int VoltsMin_cint = 6;
const int VoltsMax_cint = 7;
const int AmpChargeRatio = 8;
//Battery Profiles arrays
//                     Bulk    Absorb Float  TempCo  100pc  80pc VoltsMin VoltsMax AmpChargeRatio
float SLA_data[] =     {14.10, 14.10 ,13.50, -25.00, 12.89, 12.65, 11.96, 14.10, 0.25}; 
float GEL_data[] =     {14.10, 14.10, 13.50, -25.00, 12.89, 12.65, 11.96, 14.10, 0.25};
float AGM_data[] =     {14.40, 14.40, 13.80, -20.00, 13.00, 12.50, 11.81, 14.40, 0.25};
float Flood_data[] =   {14.40, 14.40, 13.80, -25.00, 12.89, 12.65, 11.96, 14.40, 0.30};
float LiFePO4_data[] = {13.88, 13.88, 13.88, 00.000, 13.88, 13.25, 12.00, 13.88, 1.0};
String BatteryTypeName[] = { "SLA      ","GEL      ","AGM      ","Flooded  ", "LthFePO  " };
unsigned int BatteryType; //battery type selected (0 to 4)
//                           "123456789" "123456789" "123456789","123456789", "123456789"
const size_t  arraylength = (sizeof SLA_data) / (sizeof SLA_data[0]);
float WorkingBatteryProfile[arraylength];
float ChargerAmpLimit = 8;
float StateOfCharge; //in %
byte byteBatteryRatedAmpHours;//in use
//Creat pointers to the Arrays
float* pSLA_data = SLA_data;
float* pGEL_data = GEL_data;
float* pAGM_data = AGM_data;
float* pFlood_data = Flood_data;
float* pLiFePO4_data = LiFePO4_data;
float* pWorkingBatteryProfile;
//A change in the input measurement has been detected
bool Pressed = LOW;
bool Released = HIGH;
int Stage = bulk_Stage_cint;
//Charger Modes
const bool RunState = LOW; // SetupMode must be HIGH

//1 hour timer setting
long unsigned int MaxAbsorbTimer = 10000;
//Battery Capacity
uint16_t MaxAmpHour = 200;
uint16_t MinAmpHour = 1;

/*===============================================================================
Serial Coms Setup
===============================================================================*/
long BaudRate = 9600;
/*===============================================================================
                            LCD
                        LCD Details
===============================================================================*/
byte LCD_Address = 0x27;
const unsigned int Columns = 20;
const unsigned int Rows = 4;

//LCD Rows
const unsigned int  TopLine = 0;
const unsigned int  SecLine = 1;
const unsigned int  ThirdLine = 2;
const unsigned int  LastLine = 3;
//LCD Pages
const unsigned int  BatteryChargerPage = 1;
const unsigned int  UpDatePIDVoltsControlPage = 2;
const unsigned int  UpDatePIDAmpsControlPage = 3;
const unsigned int  UpDatePIDTuningPage = 4;
const unsigned int  UpdateBatterySetupPage = 5;

volatile  static unsigned int SelectedPageLCD = BatteryChargerPage; //index to the LCD page to be displayed

/*===============================================================================
       Battery PID 
===============================================================================*/
const unsigned int Manual = 1;
const unsigned int Auto = 2;
const unsigned int Remote = 3;
const bool Direct = false;
const bool Reverse = false;
const bool MPR = true;
const bool RPM = false;
//Voltage Control
float Charger_Prop_Gain = 5;
float Charger_Integral = 0.0166666667;  //Repeats per minute 1 rpm = 1 mpr, 10 rpm = .1 mpr (6 secs)
float Charger_Derivative_Gain = 0.0;
float Charger_LowLimit = 0.0;     //PWM value
float Charger_HighLimit = 255.0;  //PWM value
//Maximum charge current Control
float CurrentLimit_Prop_Gain = 5;
float CurrentLimit_Integral = .001;  //Repeats per minute 1 rpm = 1 mpr, 10 rpm = .1 mpr (6 secs)
float CurrentLimit_Derivative_Gain = 0.0;
float CurrentLimit_LowLimit = 0.0;     //PWM value  
float CurrentLimit_HighLimit = 255.0;  //PWM value
//float PID_Action = Direct;
unsigned long PID_SampleTime = 100;  //milli secs
//equivalent to 102msecs  per input or 306msec for 3
unsigned int long MaxNumberloops = 10; 

/*===============================================================================
                Instantiation
===============================================================================*/
AnalogInputs  BatteryAnalogs;           //analog input 
PIDController VoltageControl;           //PID controller for voltage
PIDController CurrentControl;           //PID control for current limiting
LiquidCrystal_I2C lcd(LCD_Address, 20, 4);// Set the LCD I2C address
DigitalInputDebounce LCDPage_pb;       //allocate a debounce for page select PB
DigitalInputDebounce BatteryType_pb;   //    "    "     "      " battery type PB
DigitalInputDebounce SaveSettings_pb;  //    "    "     "      " save settings PB
DigitalInputDebounce SetRun_sw;        //    "    "     "      " SetRun PB

/*===============================================================================
                         RotaryEncoder
===============================================================================*/

volatile byte encoderValue = 0;
//Notes on PWM setup
/*This is for Mega----------------------->
 TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500 Hz pins D4, D13
TCCR0B = TCCR0B & B11111000 | B00000010;  // for PWM frequency of 7812.50 Hz
TCCR1B = TCCR1B & B11111000 | B00000001;  // for PWM frequency of 31372.55Hz pins D11, D12
TCCR1B = TCCR1B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz
TCCR2B = TCCR2B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz pins D9 & D10
TCCR2B = TCCR2B & B11111000 | B00000010;  // for  PWM frequency of 3921.16 Hz
TCCR3B = TCCR3B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz pins D2, D3 & D5
TCCR3B = TCCR3B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz
<--------------------------------------*/

/*===============================================================================
                setup()
===============================================================================*/
void setup()
{     //PWM Setup
    TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55Hz pins D11, D12
    //Analog input setup ie pass on the addresses of all the required arrays to the PID controlers
    BatteryAnalogs.SetArrayLocations(LastArrayElementIndex, pUintAnalogInputPins, pRaw_New, pfltAIN_New, pfltAIN_Current, pfltAIN_LRL,
        pfltAIN_URL, pAIN_Eng_Gain, pAIN_Eng_Offset, pfltInput_UpdateDeltas, pInputFilterTC);
    //LCD Setup
    Wire.begin();
    lcd.init();  // initialize the lcd
    lcd.clear();
    lcd.backlight();
    //Serial Setup
    Serial.begin(BaudRate);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB 
    }
    //SD Setup
    //pinMode(Mega_SS, OUTPUT);    //  Chip select
    //pinMode(Mega_MOSI, OUTPUT);  //  Master out slave in
    //pinMode(Mega_MISO, INPUT);   //  Master in slave out
    //pinMode(Mega_SCK, OUTPUT);   //  Serial clock   
    //SPI.begin();    // initialise the SPI library
    //SDcardSetup();  // setup for the SD card
    SetupDigital_IO();
    LoadBatterySetupEprom(); //setups battery size and type
    BatteryChargerPIDsSetup();
    InitilBatteryAssessment();                    //Check what chargeing stage to start with
    ReportVersions();
    SelectedPageLCD = BatteryChargerPage; //Default starting page
}

/*===============================================================================
                loop()
===============================================================================*/
void loop()
{
    GetDigitalInputs(); //detect all PBs and switches and determine Charger mode
    DisplaySelectedLCDPage();
    // Determines if in setup or run mode requested
    if (RequestedChargerMode == RunState)  
    {

        RunCharger();
    }
    else //Progarm is in setup mode
    {

        SetUpCharger();
    }
}

/*===============================================================================
                         SetupDigital_IO() Version 01.00.00.00

===============================================================================*/
void SetupDigital_IO()
{
    //RotaryEncodersetup
    pinMode(enc_a, INPUT_PULLUP);
    pinMode(enc_b, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc_a), ISR_ReadEncoder, CHANGE);

    //  These Switches and Pushbutton require debounce
    //Changes the LCD page. Low/false is the pressed state
    LCDPage_pb.InputDebounceSetUp(LCDPagePBpin, UsePullUp); 
    LCDPage_pb.DebounceTime(2);
    //SLA battery select. 
    BatteryType_pb.InputDebounceSetUp(BatteryType_pin, UsePullUp); 
    BatteryType_pb.DebounceTime(2);
    //Lithium battery selected.
    SaveSettings_pb.InputDebounceSetUp(SaveSettings_pin, UsePullUp); 
    SaveSettings_pb.DebounceTime(2);
    //Selection of Run or set.
    SetRun_sw.InputDebounceSetUp(SetRunPin, NoPullup); 
    SetRun_sw.DebounceTime(2);

    LCDPage_pb.ResetLatchedValue();
    BatteryType_pb.ResetLatchedValue();
    SaveSettings_pb.ResetLatchedValue();

    /*...................Mega - digital Output pins.......................*/

        //pinMode(PWMpin, OUTPUT);
    pinMode(TestLed, OUTPUT);
    pinMode(Spare3, OUTPUT);
    pinMode(Spare1, OUTPUT);
}

/*===============================================================================
                      LoadBatterySetupEprom()  Version 01.00.00.00

===============================================================================*/
void LoadBatterySetupEprom()
{
    byte BRAmpHours;
    uint8_t BatType;
    //Get 2 bytes from EEPROM
    BatType = EEPROM.read(0);
    BRAmpHours = EEPROM.read(1); // move eprom byte into upper part of int
    if (BRAmpHours < MinAmpHour)             //check bat size
    {
        BRAmpHours = MinAmpHour;
    }
    if (BRAmpHours > MaxAmpHour)             //check bat size
    {
        BRAmpHours = MaxAmpHour;
    }
    byteBatteryRatedAmpHours = BRAmpHours;
    //Serial.println("EEprom settings loaded");
    //Serial.print(BRAmpHours);
    //Check that battery type is legal/reasonalble   
    if (BatType <  SLA_Battery_cint || BatType > LithFePo_Battery_cint)  //check bat type
        BatteryType = AGM_Battery_cint;
    else
    {
        BatteryType = BatType;
    }
}

/*===============================================================================
                      SaveBatterySetupEprom()  Version 01.00.00.00

===============================================================================*/
void SaveBatterySetupEprom()
{
    byte ByteAhours;
    // Extract low byte of 16 bit integer  
    ByteAhours = byteBatteryRatedAmpHours;
    EEPROM.update(0, BatteryType);
    EEPROM.update(1, ByteAhours);
    Serial.print("EEprom updated  ");
    Serial.println(ByteAhours);
}

/*===============================================================================
                      BatteryChargerPIDsSetup()  Version 01.00.00.00
    Battery Charger PID controlers Setup
===============================================================================*/
void BatteryChargerPIDsSetup()
{
    VoltageControl.SetPIDRunPeriod(PID_SampleTime);
    VoltageControl.SetIntegralUnits(MPR);
    VoltageControl.Action(Direct);
    VoltageControl.Track(false);
    VoltageControl.SetAutoManualRemote(Manual);
    VoltageControl.Tuning(Charger_Prop_Gain, Charger_Integral, Charger_Derivative_Gain);
    VoltageControl.SetLowLimits(Charger_LowLimit);  //%
    VoltageControl.SetHighLimits(Charger_HighLimit);  //%
    //Start up with charger output off and PID in manual
    VoltageControl.SetMV(00); //PWM output set to default value = 00% duty Cycle
    VoltageControl.SetPV(fltAIN_Current[Battery_Volts]);  //Load the PV to the PID 
    CurrentControl.SetPIDRunPeriod(PID_SampleTime);
    CurrentControl.SetIntegralUnits(MPR);
    CurrentControl.Action(Reverse);
    CurrentControl.Track(false);
    CurrentControl.SetAutoManualRemote(Manual);
    CurrentControl.Tuning(CurrentLimit_Prop_Gain, CurrentLimit_Integral, CurrentLimit_Derivative_Gain);
    CurrentControl.SetLowLimits(CurrentLimit_LowLimit);  //
    CurrentControl.SetHighLimits(CurrentLimit_HighLimit);  //
    //Start up with charger output off and PID in manual
    CurrentControl.SetMV(00); //PWM output set to default value = 00% duty Cycle
    CurrentControl.SetPV(fltAIN_Current[Battery_Amps]);  //Load the PV to the PID
    CurrentControl.SetSV(fltAIN_Current[Battery_Amps]);  //Load the SV to the PID
}

/*===============================================================================
                         InitilBatteryAssessment() Version 02.00.00.00
    Assess the  battery condition and select the correct voltage and stage for 
    optimum charging. 
    Note 1. this sub does not start the charging process.
    Note 2. It can not assess whether the battery is at its equilibrium voltage
            as the Equilibrium voltage is not reached until 20 to 24 hours after a 
            charge/discharge cycle, this program has no way of knowing that 
            elapsed time.
    Note 3. It assumes anything less than pWorkingBatteryProfile[Absorb_V_cint] value 
            to be not at full charge and will set the stage to bulk charge.
            
===============================================================================*/

void InitilBatteryAssessment()
{
    //Read Battery type and size. 
      //Start up with charger output off and PID in manual
     Charger_OFF();
    //What is the battery voltage? if < 80% then bulk charge
     if (fltAIN_Current[Battery_Volts] <  pWorkingBatteryProfile[Absorb_V_cint])
     {
         Stage = bulk_Stage_cint;
         VoltageControl.SetSV(pWorkingBatteryProfile[Bulk_V_cint]);  //Load the set point to the PID
     }
     else if (fltAIN_Current[Battery_Volts] >= pWorkingBatteryProfile[Absorb_V_cint])
    {
        Stage = absorb_Stage_cint;
                VoltageControl.SetSV(pWorkingBatteryProfile[Absorb_V_cint]);  //Load the set point to the PID
    }
    else if (fltAIN_Current[Battery_Volts] >= pWorkingBatteryProfile[Float_V_cint])
    {
        Stage = float_Stage_cint;
        VoltageControl.SetSV(pWorkingBatteryProfile[Float_V_cint]);  //Load the set point to the PID
    }
}

/*===============================================================================
                         GetDigitalInputs() Version 02.00.00.00
    This debounces high and low going transitions using a compare and count method.
===============================================================================*/
void GetDigitalInputs() //Runs in Loop() unconditionly every time
{
    static bool Do_Once_1 = false;
    static bool Do_Once_2 = false;
    static bool Do_Once_3 = false;
    //Input Setup/Run
    RequestedChargerMode = SetRun_sw.GetDebouncedValue();  //Read the Run/Setup switch
    //LCD select Pushbutton
    LCDPagePBState = LCDPage_pb.GetDebouncedValue();  //Read the LCD select PB
    if (RequestedChargerMode == RunState)           //If the Run/Setup switch is "RUN"
    {
        /*if (LCDPagePBState == UpdateBatterySetupPage)
        {
            LCDPagePBState = BatteryChargerPage;
        }*/
        if (LCDPagePBState == LOW)  //looking for PB being released ie High
        {   //  Set the LCD dislay to a new page every time the LCD page select PB is pressed 
            Do_Once_1 = true;
        }
        else
        {
            if (Do_Once_1 == true)
            {
                SelectedPageLCD++;
                if (SelectedPageLCD > UpdateBatterySetupPage) //make valid pages possible
                {
                    SelectedPageLCD = BatteryChargerPage;
                }
                Do_Once_1 = false;
            }
        }
    }
    else
    {
        BatteryTypeState = BatteryType_pb.GetDebouncedValue();  //LOW/false is the pressed state 
        if (BatteryTypeState == LOW)  //looking for PB being released ie High
        {   //  Set the LCD dislay to a new page every time the LCD page select PB is pressed 
            Do_Once_2 = true;
        }
        else
        {
            if (Do_Once_2 == true)
            {
                BatteryType++;
                if (BatteryType > 4) //make valid pages possible
                {
                    BatteryType = 0;
                }
                Do_Once_2 = false;
            }
        }
        SaveSettingsState = SaveSettings_pb.GetLatchedValue();  //LOW/false is the pressed state  
    }
    //For testing and commissioning 
    digitalWrite(TestLed, BatteryTypeState);
    digitalWrite(Spare3, RequestedChargerMode);
    digitalWrite(Spare1, SaveSettingsState);
    //MonitorPBsTEST();
    //byteBatteryRatedAmpHours = encoderValue;
}

/*===============================================================================
           Calculate Temp compensation for SV()  Version 01.00.00.00
===============================================================================*/
float CalcTempComp_SV(float SVTempTarget)
{
    float StandardTemp = 20; //Degrees C    
    return  SVTempTarget + ((fltAIN_Current[Battery_Temperature] - StandardTemp) * 
        (pWorkingBatteryProfile[TempCoef_cint] / 1000));
}

/*===============================================================================
           Estimate % Charged  Version 01.00.00.00
===============================================================================*/
float PCCharge(float volts)
{
    float temp = 0;
    float A = 125.0;
    float B = 4695.2;
    float C = 58872.0;
    float D = 246329.0;
    return (A * pow(volts, 3)) - (B * pow(volts, 2)) + (C * volts) - D;
}

/*===============================================================================
                       CalcBulkChargeAmps()  Version 01.00.00.00
===============================================================================*/
float CalcBulkChargeAmps()
{
    return float(byteBatteryRatedAmpHours) * 0.3;
}

/*===============================================================================
           Calculate Recharge Time  Version 01.00.00.00
===============================================================================*/
float CalculateRechargeTime()
{
    return (float(byteBatteryRatedAmpHours) - StateOfCharge) / 
        (0.9 * ChargerAmpLimit);
}

/*===============================================================================
                       OneHourTimer()  Version 01.00.00.00
===============================================================================*/
boolean OneHourTimer(boolean StartTimer)
{
    long unsigned int time = 0;
    if (StartTimer == true)
    {
        time++;
        if (time >= MaxAbsorbTimer)
            return true;
        else
            return false;
    }
    else
        return false;
}

/*===============================================================================
                         void ISR_ReadEncoder()
                         Interupt service routine
                           Version 01.00.00.00
===============================================================================*/
void ISR_ReadEncoder()
{
    noInterrupts();
    if (RequestedChargerMode != RunState)
    {
        if (digitalRead(enc_a) == digitalRead(enc_b))
        {
            encoderValue--;
            if (encoderValue < MinAmpHour)
                encoderValue = MinAmpHour; //limit lower values to 1 amp/hour        
        }
        else
        {
            encoderValue++;
            if (encoderValue > MaxAmpHour)
                encoderValue = MaxAmpHour; //limit upper values to 500 amp/hours
        }
        byteBatteryRatedAmpHours = encoderValue;
    }
    interrupts();
}

/*===============================================================================
                          DisplaySelectedLCDPage()  Version 02.00.00.00
    Controls the selection of pages to dispaly on the LCD
    The update time is governed by a counter which is updated by the ISR
    This means that update is always multiples of 1 sec.
    This is because the ISR is in turn triggered by the RTC generation on a 
    1 sec pulse
===============================================================================*/
void DisplaySelectedLCDPage()
{
    static int long DisplayRefreshCounter = 0;
    static unsigned int CurrentDisplay = 1;

    if (CurrentDisplay != SelectedPageLCD) 
    {
        //Clear the LCD only if the page is changed
        lcd.clear();
        CurrentDisplay = SelectedPageLCD;
    }
    //   Display the Current LCDPage every nth pass
    if (DisplayRefreshCounter++ > 100)  
    {
        switch (CurrentDisplay)
        {
        case BatteryChargerPage:  //
            BatteryChargerLCD();
            break;
        case UpDatePIDVoltsControlPage:  //
            UpDatePIDVoltsLCD();
            break;
        case UpDatePIDAmpsControlPage:  //
            UpDatePIDAmpsLCD();
            break;
        case UpDatePIDTuningPage:  //
            UpDatePIDTuningLCD();
            break;
        case UpdateBatterySetupPage:
            UpdateBatterySetupLCD();
            break;
        default:  //Restart page number because illegal page number detected
            SelectedPageLCD = BatteryChargerPage;
        }
        DisplayRefreshCounter = 0;
    }
}

/*===============================================================================
                        RunCharger()  Version 01.00.00.00

    ===============================================================================*/
void  RunCharger()
{
    static int long RunControlCounter = 0;
    static unsigned int Phase = 0;
    if (RunControlCounter++ > MaxNumberloops)  //
    {
        // Reads volts, amps, & temperature convert to eng units, provides filtering  
        BatteryAnalogs.ReadAnalogsInputs(); 
        SelectBatteryType();
        BatteryCharger_Stratergy_1();
        //BatteryCharger_Stratergy_2();     
        RunControlCounter = 0;
    }

}

/*===============================================================================
                       BatteryCharger_Stratergy_1()  Version 02.00.00.00
WorkingBatteryProfile contains the neccessary voltage setings for each battery type
Bulk stage is by constant current control 
===============================================================================*/
void BatteryCharger_Stratergy_1() {
    VoltageControl.SetAutoManualRemote(Auto); //Turn on the PID controllers
    CurrentControl.SetAutoManualRemote(Auto);
    //Set the SV based on charger stage  
    switch (Stage)
    {
    case bulk_Stage_cint:
        VoltageControl.SetSV(CalcTempComp_SV(pWorkingBatteryProfile[Bulk_V_cint]));
        if (fltAIN_Current[Battery_Volts] >= pWorkingBatteryProfile[Bulk_V_cint])
            Stage = absorb_Stage_cint; // step to the next stage
        break;
    case absorb_Stage_cint:
        VoltageControl.SetSV(CalcTempComp_SV(pWorkingBatteryProfile[Absorb_V_cint]));
        OneHourTimer(true);
        Stage = float_Stage_cint;
        break;
    case float_Stage_cint:
        VoltageControl.SetSV(CalcTempComp_SV(pWorkingBatteryProfile[Float_V_cint]));
        break;
    default:
        Stage = float_Stage_cint;
        break;
    } 
    CalcAmpsLimit();         
    CurrentControl.SetPV(fltAIN_Current[Battery_Amps]);
    CurrentControl.PIDControllerExec();
    AmpAlarm();
    VoltageControl.SetPV(fltAIN_Current[Battery_Volts]);
    VoltageControl.SetHighLimits(CurrentControl.GetMV()); 
    VoltageControl.PIDControllerExec();
    //PWMpin goes high to turn on regulator FET and low to turn it off.
    //Timer 2 Reprogrammed to run at 31kHz 
    analogWrite(PWMpin, uint8_t(VoltageControl.GetMV())); 
}

/*===============================================================================
                       BatteryCharger_Stratergy_2()  Version 02.00.00.00
WorkingBatteryProfile contains the neccessary voltage setings for each battery type
===============================================================================*/
void BatteryCharger_Stratergy_2() {
    VoltageControl.SetAutoManualRemote(Auto); //Turn on the PID controllers
    CurrentControl.SetAutoManualRemote(Auto);
    //Set the SV based on charger stage
    switch (Stage)
    {
    case bulk_Stage_cint:
        VoltageControl.SetSV(CalcTempComp_SV(pWorkingBatteryProfile[Bulk_V_cint]));
        if (fltAIN_Current[Battery_Volts] >= pWorkingBatteryProfile[Bulk_V_cint])
            Stage = absorb_Stage_cint; // step to the next stage
        break;
    case absorb_Stage_cint:
        VoltageControl.SetSV(CalcTempComp_SV(pWorkingBatteryProfile[Absorb_V_cint]));
        OneHourTimer(true);
        Stage = float_Stage_cint;
        break;
    case float_Stage_cint:
        VoltageControl.SetSV(CalcTempComp_SV(pWorkingBatteryProfile[Float_V_cint]));
        break;
    default:
        Stage = float_Stage_cint;
        break;
    }
    CalcAmpsLimit();
    CurrentControl.SetPV(fltAIN_Current[Battery_Amps]);
    CurrentControl.PIDControllerExec();
    AmpAlarm();
    VoltageControl.SetPV(fltAIN_Current[Battery_Volts]);
    VoltageControl.SetHighLimits(CurrentControl.GetMV());
    VoltageControl.PIDControllerExec();
    //PWMpin goes high to turn on regulator FET and low to turn it off.
    //Timer 2 Reprogrammed to run at 31kHz
    analogWrite(PWMpin, uint8_t(VoltageControl.GetMV())); 
}

/*===============================================================================
                       CalcAmpsLimit()  Version 01.00.00.00
    Charge amps is Amp/Hour * AmpChargeRatio (100amp/hrs  * 0.25 = 4amps
    Maximum available charging 8 amp current  begins decreaseing below 32 Amp/hrs 
    battery capacities(for 25% AmpChargeRatio)
===============================================================================*/
void CalcAmpsLimit()
{
    float ChargerampsTarget = pWorkingBatteryProfile[AmpChargeRatio] * 
        float(byteBatteryRatedAmpHours);
    if (ChargerampsTarget >= ChargerAmpLimit)
    {
        CurrentControl.SetSV(ChargerAmpLimit);
    }
    else
    {
        CurrentControl.SetSV(ChargerampsTarget);
    }
}

/*===============================================================================
              AmpAlarm()
===============================================================================*/
void AmpAlarm()
{
    if ((VoltageControl.HighLimitIndicator() == true) && 
        CurrentControl.GetMV() < 255)
    {
        AmpsInAlarm = true;
        AmpAlarmMessage = "CTL";
        VoltAlarmMessage = "   ";
    }
    else
    {
        AmpsInAlarm = false;
        AmpAlarmMessage = "   ";
        VoltAlarmMessage = "CTL";
    }
}

/*===============================================================================
              Charger_OFF()
===============================================================================*/
void Charger_OFF()
{
    VoltageControl.SetAutoManualRemote(Manual);
    CurrentControl.SetAutoManualRemote(Manual);
    VoltageControl.SetMV(00); //PWM output set to 0% duty Cycle
    VoltageControl.SetSV(00); //Load the set point to the PID   
    CurrentControl.SetMV(00);   //PWM output set to 0% duty Cycle
    CurrentControl.SetSV(00);   //Load the set point to the PID
    analogWrite(PWMpin, uint8_t(00));
}


/*===============================================================================
                        SetUpCharger()  Version 01.00.00.00

    ===============================================================================*/
void  SetUpCharger()
{
    SelectedPageLCD = UpdateBatterySetupPage;
    DisplaySelectedLCDPage();
    SelectBatteryType();
    if (SaveSettingsState == Pressed)  //LOW/false is the pressed state
    {
        SaveBatterySetupEprom(); //Only updates eeprom if it has changed
        SaveSettings_pb.ResetLatchedValue();
        SaveSettingsState = Released;
    }
}
/*===============================================================================
                       SelectBatteryType()  Version 01.00.00.00
===============================================================================*/
void SelectBatteryType()
{
    switch (BatteryType)
    {
    case SLA_Battery_cint:
        //Make battery SLA type profile is the working array;
        pWorkingBatteryProfile = pSLA_data;      
        break;
    case GEL_Battery_cint:
        pWorkingBatteryProfile = pGEL_data;
        break;
    case AGM_Battery_cint:
        pWorkingBatteryProfile = pAGM_data;
        break;
    case FLooded_Battery_cint:
        pWorkingBatteryProfile = pFlood_data;
        break;
    case LithFePo_Battery_cint:
        pWorkingBatteryProfile = pLiFePO4_data;
        break;
    default:  //Always make sure there is a default set
        pWorkingBatteryProfile = pAGM_data;
        BatteryType = AGM_Battery_cint;
    }
}

/*===============================================================================
                         IsNextLCDPageSelected() Version 02.00.00.00
    Sets the LCD dislay to new page every time the LCD page select PB is pressed
===============================================================================*/
void IsNextLCDPageSelected()
{
    static bool Armed = false;
    if (LCDPagePBState == false)  //looking for PB being released ie High
    {
        Armed = true;
    }
    if (LCDPagePBState == true && Armed == true)
    {
        lcd.clear();
        SelectedPageLCD++;
        if (SelectedPageLCD > UpdateBatterySetupPage)
        {
            SelectedPageLCD = BatteryChargerPage;
        }
        Armed = false;
    }
}

/*===============================================================================
    BatteryChargerLCD()  Version 01.00.00.00
    LCD display
===============================================================================*/
void BatteryChargerLCD()
{
    float Kp, Ki, Kd;
    String Unitlabel = "";
    lcd.backlight();
    lcd.setCursor(0, TopLine);
    //        "0123456789123456789"
    lcd.print(BatteryTypeName[BatteryType]);
    lcd.setCursor(10, TopLine);
    lcd.print(StageName[Stage]);//Volts Value
    lcd.setCursor(0, SecLine);
    lcd.print(AnalogInputNames[0]);  //Volts name
    lcd.setCursor(10, SecLine);
    lcd.print(pfltAIN_Current[0]);    //Volts Value
    lcd.setCursor(16, SecLine);
    lcd.print(VoltAlarmMessage);
    lcd.setCursor(0, ThirdLine);
    lcd.print(AnalogInputNames[1]);  //Amps name
    lcd.setCursor(10, ThirdLine);
    lcd.print(pfltAIN_Current[1]);   //Amps Value
    lcd.setCursor(16, ThirdLine);
    lcd.print(AmpAlarmMessage); 
    lcd.setCursor(0, LastLine);
    lcd.print(AnalogInputNames[2]);  //Temp Value
    lcd.setCursor(10, LastLine);
    lcd.print(pfltAIN_Current[2]);
    Serial.print("Volts  " + String(Raw_New[0]) + ",   " + String(fltAIN_Current[0]) + ",   ");
    Serial.print("Amps   " + String(Raw_New[1]) + ",   " + String(fltAIN_Current[1]) + ",   ");
    Serial.print("Temp   " + String(Raw_New[2]) + ",   " + String(fltAIN_Current[2]) + ",   ");
    Serial.println("AmpHrs " + String(byteBatteryRatedAmpHours));
}

/*=============================================================================
BatterySetup()  Version 01.00.00.00
LCD display
============================================================================== = */
void UpdateBatterySetupLCD()
{
    String temp = "";
    lcd.backlight();
    lcd.setCursor(0, TopLine);
    lcd.print("   Battery Setup   ");
    //        "0123456789123456789"
    lcd.setCursor(0, SecLine);
    lcd.print("Type    ");  //Battery Type
    lcd.setCursor(10, SecLine);
    lcd.print(BatteryTypeName[BatteryType]);   //Volts Value
    lcd.setCursor(0, ThirdLine);
    lcd.print("A/Hour  ");  //Battery A/H
    lcd.setCursor(10, ThirdLine);
    temp = String(byteBatteryRatedAmpHours) + "    ";
    lcd.print(temp);  //Amps Value
    lcd.setCursor(0, LastLine);
    lcd.print("Charge %");  //Estimated Capacity
    //        "01234567"
    lcd.setCursor(10, LastLine);
    lcd.print("100.00");
}

/*===============================================================================
    UpDatePIDVoltsLCD()  Version 02.00.00.00
    LCD display
===============================================================================*/
void UpDatePIDVoltsLCD()
{
    //static bool firsttime = true;
    //int LCDRow;
    // int InputNumber;
    float PV, SV_Target, MV, SV_Inuse;
    String temp = "";
    // GetCurrentDateTime();
    lcd.backlight();
    {
        //The SV target value after bumpless has concluded
        SV_Target = VoltageControl.GetSV(); 
        PV = VoltageControl.GetPV();
        MV = VoltageControl.GetMV();
        //The intermediate SV as it tracks towards SV 
        SV_Inuse = VoltageControl.GetSVRamp();
        lcd.setCursor(0, TopLine);
        lcd.print("  Volts Controler   ");
        //        "01234567890123456789"
        lcd.setCursor(0, SecLine);
        lcd.print("PV  ");  // label
        lcd.setCursor(4, SecLine);
        lcd.print(PV);  // label
        lcd.setCursor(0, ThirdLine);
        lcd.print("SV  ");  // value
        lcd.setCursor(4, ThirdLine);
        lcd.print(SV_Inuse);  // label
        lcd.setCursor(11, ThirdLine);
        lcd.print("<-- ");  // value
        lcd.setCursor(15, ThirdLine);
        lcd.print(SV_Target);  // label
        lcd.setCursor(0, LastLine);
        lcd.print("MV  ");  // alarm
        lcd.setCursor(4, LastLine);
        lcd.print(MV);  // alarm
    }
}
/*===============================================================================
    UpDatePIDAmpsLCD()  Version 01.00.00.00
    LCD display
===============================================================================*/
void UpDatePIDAmpsLCD()
{
    //static bool firsttime = true;
    //int LCDRow;
    // int InputNumber;
    float PV, SV_Target, MV, SV_Inuse;
    String temp = "";
    // GetCurrentDateTime();
    lcd.backlight();
    {
        //The SV target value after bumpless has concluded
        SV_Target = CurrentControl.GetSV(); 
        PV = CurrentControl.GetPV();
        MV = CurrentControl.GetMV();
        //The intermediate SV as it tracks towards SV 
        SV_Inuse = CurrentControl.GetSVRamp();
        lcd.setCursor(0, TopLine);
        lcd.print("    Amps Limiter    ");
        //        "01234567890123456789"
        lcd.setCursor(0, SecLine);
        lcd.print("PV  ");  // label
        lcd.setCursor(4, SecLine);
        lcd.print(PV);  // label
        lcd.setCursor(0, ThirdLine);
        lcd.print("SV  ");  // value
        lcd.setCursor(4, ThirdLine);
        lcd.print(SV_Inuse);  // label
        lcd.setCursor(11, ThirdLine);
        lcd.print("<-- ");  // value
        lcd.setCursor(15, ThirdLine);
        lcd.print(SV_Target);  // label
        lcd.setCursor(0, LastLine);
        lcd.print("MV  ");  // alarm
        lcd.setCursor(4, LastLine);
        lcd.print(MV);  // alarm
    }
}
/*===============================================================================
    UpDatePIDTuningLCD()  Version 02.00.00.00
    LCD display
===============================================================================*/
void UpDatePIDTuningLCD()
{
    float Kp, Ki, Kd;
    String Unitlabel = "";
    lcd.backlight();
    Kp = VoltageControl.GetKp();
    Ki = VoltageControl.GetKI();
    Kd = VoltageControl.GetKd();
    lcd.setCursor(0, TopLine);
    lcd.print(" PID Control Tuning ");
    lcd.setCursor(0, SecLine);
    lcd.print("Kp Gain");
    lcd.setCursor(8, SecLine);
    lcd.print(Kp);
    if (VoltageControl.GetIntegralUnits() == MPR) {
        Unitlabel = "Ki MPR ";
    }
    else {
        Unitlabel = "Ki RPM ";
    }
    lcd.setCursor(0, ThirdLine);
    lcd.print(Unitlabel);
    lcd.setCursor(8, ThirdLine);
    lcd.print(Ki);  // label
    lcd.setCursor(0, LastLine);
    lcd.print("Kd Mins ");
    lcd.setCursor(8, LastLine);
    lcd.print(Kd);
}

/*===============================================================================
    UpDatePIDTuningLCD()  Version 02.00.00.00
    LCD display
===============================================================================*/
void ReportVersions()
{   
    Serial.println("-----Charger Booting-----");
    Serial.println("VoltageControl.ino Ver.  " + Version);
    Serial.println("PIDController.h Ver.  " + VoltageControl.GetVersion());
    Serial.println("DigitalInputDebounce.h Ver.  " + LCDPage_pb.GetVersion());
    Serial.println("AnalogInputs.h Ver.  " + BatteryAnalogs.GetVersion());
    delay(5000);
}
