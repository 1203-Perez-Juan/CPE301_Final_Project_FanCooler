
#include <DHT_U.h>

#include <ServoTimer2.h>

#include <LiquidCrystal.h>

#include <DFRobot_DHT11.h>

#include <Arduino.h>

#include "DHT.h"

#include <TimeLib.h>

#include <ServoTimer2.h>

#include <Stepper.h>

#include <RTClib.h>

//-------------------------------------------------------------------------//
// stepper motor pin 1
#define PINONE A1 
         
// stepper motor pin 2
#define PINTWO A2 
         
// stepper motor pin 3
#define PINTHREE A3 

// stepper motor pin 4
#define PINFOUR A4         

// This variable is for steps on motor calculation
#define STEPS 32

// This is the pin for the DHT sensor
#define DHT_11 7     

// This for the pin 7 connected to DHT
#define DHTPIN 7           

// This is for DHT connected for Temerature & humidity measurements
#define DHTTYPE DHT11
       
// This is related to the character requesting a time sync message 
#define TIME_REQUEST  7 

// below is for the serial time sync message         
#define TIME_HEADER  'T'

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

DHT mySensor(DHTPIN,DHTTYPE);
Stepper stepper(STEPS,PINONE,PINTWO,PINTHREE,PINFOUR);
RTC_DS1307 rtc;

//ServoTimer2 ventilationMovement;

volatile unsigned int PreviousValue;

int SensorWaterIdentify = 0;

const int LowThresholdForTemp = 65;

const int waterCapacityThreshold = 150;

int InitialTemperature = 0;

int InitialHumidity = 0;


// The Following is the starting states variables
bool Idle = false;

bool AcceptedWaterLevel = false;

bool TempOverload = false;

bool StateCoolFan = false;

bool Error = false;

bool Recompensated = false;



ServoTimer2 vent_control;

void setup();

void loop();

void adc_init();

void SystemTemperatureDisp();

void ErrorLCDdisplay();

void setType(uint8_t type = 11);

unsigned int adc_read(unsigned char adc_channel_num);

void logTime(bool b);

// The Following is for assigning variables to the Port Registers

// ddrs
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;

volatile unsigned char* ddr_k  = (unsigned char*) 0x107;

volatile unsigned char* ddr_l = (unsigned char*) 0x10A;

volatile unsigned char* ddr_h = (unsigned char*) 0x101;

//Ports
volatile unsigned char* port_b = (unsigned char*) 0x25;
 
volatile unsigned char* port_h = (unsigned char*) 0x102;

volatile unsigned char* port_k = (unsigned char*) 0x108;

volatile unsigned char* port_l = (unsigned char*) 0x10B;

volatile unsigned char* pin_l = (unsigned char*) 0x109;

volatile unsigned char* pin_k  = (unsigned char*) 0x106;



// The following is for adsigning variables to the ADC Registers
volatile unsigned int * my_ADC_DATA = (unsigned int *) 0x78;

volatile unsigned char * my_ADMUX = (unsigned char *) 0x7C; 

volatile unsigned char * my_ADCSRA = (unsigned char *) 0x7A;

volatile unsigned char * my_ADCSRB = (unsigned char *) 0x7B;


//registers D4 through D7
LiquidCrystal lcd(6,5,4,3,8,2); 

DateTime t;


ISR(TIMER3_COMPA_vect){

// The following is where the code checks for water level measured by the water sensor
int currentValue = adc_read(SensorWaterIdentify); 
  
  PreviousValue = currentValue;

if((currentValue < waterCapacityThreshold) && (AcceptedWaterLevel == true))
{ 
  Error = true; 
}
  
if(!(currentValue < waterCapacityThreshold) && (AcceptedWaterLevel == false)) 
{ 
  Recompensated = true; 
}
  
if(currentValue < waterCapacityThreshold) 
{ 
  AcceptedWaterLevel = false;
}

else 
{ 
  AcceptedWaterLevel = true; 
}
  
if(InitialTemperature > LowThresholdForTemp) 
{ 
  TempOverload = true; 
}
  else { TempOverload = false; }
}

void setup() 
{   

//OCR3A = 255;

// The following compares the CTC mode and internally clks
//TCCR3A = (1 << WGM01);
   
//TCCR3B = 0x03;
  
// The following enables timer and intterrupts
//sei();
TIMSK3 = (1 << OCIE0A);
  
//sei();                  
*ddr_b |= 0b11110000;
  
*ddr_k &= 0b01111111;
  
*port_k |= 0b10000000;
  
// The following makes the fan pin's output pins
*ddr_l |= 0b00101000;
  
// The following makes the fans go to low
*port_l &= 0b11010111;

  
Serial.begin(9600);

rtc.begin();

rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
mySensor.begin();

  
// The following sets the spacing of the LCD will display
lcd.begin(16, 2); 

// The following will be displayed when the program starts
lcd.print("Measuring");

lcd.setCursor(0,2);

lcd.print("Temperature");

//The following sets the stepper motor or vent speed
stepper.setSpeed(200);  

}


void loop() 
{ 
// This code will loop when the system is operating
  
SystemTemperatureDisp();

if(!(*pin_k & 0b10000000))
{
    
for (volatile unsigned int i = 0; i < 50; i++);
      
  if(!(*pin_k & 0b10000000))
  {
    Idle = !Idle;
      if(Idle)
      {
        cli();
      }
      else
      {
        *port_b &= 0b11101111;
         //sei();
      }
  }
}

if(Idle)
{
  
// The following turna on the yellow LED and turns off the fan and other LEDs
*port_b |= 0b00010000;
 
*port_b &= 0b00011111; 

// The following turns off the stepper motor or vent
stepper.step(0);
  if(StateCoolFan) 
  { 
    logTime(!StateCoolFan);
    *port_l &= 0b11110111; 
  } 
  // The follow stepper.step(0)
  }
  else
 {
  if(!AcceptedWaterLevel)
  {
  stepper.step(0);

  ErrorLCDdisplay();
      
  // The following turns the red LED on and turns the blue LED off 
  *port_b |= 0b00100000;
      
  *port_b &= 0b00111111;    

    if(StateCoolFan)
    { 
      //The following turns off the fan
      logTime(!StateCoolFan);
      
      *port_l &= 0b11110111;
    }                                  
  }
  else if(AcceptedWaterLevel)
  {
    //The follwoing turns off the red LED and turn on the green LED
    SystemTemperatureDisp();
    
    *port_b &= 0b11011111;

    if(!TempOverload)
    { 
      *port_b |= 0b01000000; 
    }

      if(TempOverload)
      {
        //The following turns on the blue LED and Comes out of the Idle state turning off green LED
        *port_b &= 0b10111111; 
        
        *port_b |= 0b10000000; 
       
        if(!StateCoolFan) 
        { 
          //The following turns on the fan when coming off of idle state
          logTime(true); 
          *port_l |= 0b00001000;
          
        } 
        
        //The following turns on the stepper motor when the fan state is on
        stepper.step(8);

        StateCoolFan = true;
      }
      else
      {
        stepper.step(0);

        StateCoolFan =  false;

        // The following makes the system go from running to a stopped state without turning any other LED
        *port_h &= 0b10111111;
        
        if(StateCoolFan) 
        { 
          logTime(!StateCoolFan); 
          *port_l &= 0b11110111; 
        } 

       }
  }
}

t = rtc.now();
} 

void ErrorLCDdisplay()
{
  if(Error)
  {
    // The Following is what the LCD will display when the water sensor detects a low water level
    lcd.clear();
    
    lcd.print("Warning");
    
    lcd.setCursor(0,2);
    
    lcd.print("H20 Supply Low!");
     
  }
}

void SystemTemperatureDisp()
{
  // The following creates 
  if(AcceptedWaterLevel)
  {
    
    InitialTemperature = (mySensor.readTemperature() * 9/5) + 32;

    InitialHumidity = mySensor.readHumidity();

    //The following is what will be displayed on the LCD Display
    
    lcd.clear();

    lcd.print("Temp: ");

    lcd.print(InitialTemperature);

    lcd.print("F");

    lcd.setCursor(0,2);

    lcd.print("Humidity: ");

    lcd.print(InitialHumidity);

    lcd.print("%");
    Recompensated = false;
  }
  
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  
uint8_t low, high;

  // The following will be used for using ADC to read multiple pins 0 through 15
  if (adc_channel_num >= 54) adc_channel_num -= 54; 

    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adc_channel_num >> 3) & 0x01) << MUX5);

    ADMUX = (1 << 6) | (adc_channel_num & 0x07);

    #if defined(ADCSRA) && defined(ADCL)
 
      sbi(ADCSRA, ADSC);

      while (bit_is_set(ADCSRA, ADSC));
  
        low = ADCL;
        
        high = ADCH;
        
      #else
      
        low  = 0;
        
        high = 0;
      #endif

return (high << 8) | low;

}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void logTime(bool timeVar)
{
  if(timeVar) 
  { 
    Serial.print("Fan Enabled "); 
  }
  else 
  { 
    Serial.print("Fan Disabled "); 
  } 
  
}
