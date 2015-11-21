/*

 Arduino Controlled Dual Output Si5351A VFO
 
 A VFO project that uses an Arduino Uno or Nano to control a SI5351A clock 
 generator breakout board. This version of the VFO is used in stand-alone mode.

 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 Copyright (C) 2014,  Gene Marcus W3PM GM4YRE
 
 16 December,2014
 
 v5 12 July, 2015  Corrected band select algorithm
  
 v5.1.2 16 November, 2015 Changed Clock Frequency to 27 MHz, added logon msg,
 changed starting resolution (fstep) to 1 KHz, added 27 MHz calibration frequency
 as last Band,changed QRP Band frequencies for Region 1, added some of my popular
 QRP frequencies, added the Valise QRP offsets for 20 and 40 meters,
 set Frequency start/lower & stop/upper limits.Deleted GPS assisted frequency
 correction function as it was out of my simple project scope   
 Mods by Konstantinos SV1ONW
 
 ---------------------------------------------------------------------------
 Nano Digital Pin Allocation
 
 D0/RX  spare
 D1/TX  spare
 D2     spare
 D3  Rotary encoder pin A - if used
 D4  Rotary encoder pin B - if used
 D5  2.7 MHz input from Si5351 CLK0 pin
 D6  Frequency resolution button
 D7  LCD RS 
 D8  LCD enable
 D9  LCD DB4
 D10 LCD DB5
 D11 LCD DB6
 D12 LCD DB7
 D13     spare
 A0 Decrease frequency button
 A1 Increase frequency button
 A2 Offset enable
 A3 Band Select button               
 A4 Si5351 SDA
 A5 Si5351 SCL 
 
 ----------------------------------------------------------------
 */

/*
_________________________________________________________________
 Band Select frequency format:
 
 Column 1 = CLK1 VFO start frequency in Hz
 Column 2 = CLK2 LO frequency in Hz
 Column 3 = LCD display arithmetic operation 
 0 = no change
 1 = Add CLK2 and CLK1
 2 = Subtract CLK2 from CLK1 
 
 Example: {11008200,3276800,1}, will result in
 LCD display: 14,285000 MHz
 CLK1 output: 11,008200 MHz 
 CLK2 output: 3,276800 MHz
 
 Enter any number of Band Select frequencies.  
 Use (0,0,0) as the last entry.    
 
 Restrict frequency entries to > 1 MHz and < 112,5 MHz
 
 ___________Enter Band Select frequencies below_____________________
 */
const unsigned long Freq_array [] [3] = {
  { 14070000,0,0       },     // CLK1=14.070 MHz, CLK2=0 MHz, Display=14,070.000 KHz
  { 14060000,0,0       },            
  {  7030000,0,0       },
  {  3560000,0,0       },
  {  1843000,0,0       },
  { 10106000,0,0       },
  { 18086000,0,0       },
  { 21060000,0,0       },
  { 24906000,0,0       },
  { 28060000,0,0       },
  { 50060000,0,0       },
  { 11008200,3276800,1 },      // CLK1:11,082200 MHz, CLK2:3,276800 MHz, Display:14,285.000 KHz
  { 12005200,4915200,2 },      // CLK1:12,005200 MHz, CLK2:4,915200 MHz, Display:7,090.000 KHz
  { 12100200,4915200,2 },      // CLK1:12,100200 MHz, CLK2:4,915200 MHz, Display:7,185.000 KHz    
  { 27000000,0,0       },      // CLK1 has the Calibration Frequency of 27 MHz for easy selection
  (0,0,0)
  };


//_________________________Enter offset frequency (Hz) below:__________________________________
int fOffset = -600;        // -600 Hz offset for CW operation activated by Pin A2 becoming low             


//___________________________Enter stand-alone calibration factor:______________________________
//  - Connect VFO to a frequency counter
//  - Set VFO to 27 MHz
//  - Annotate counter frequency in Hz
//  - Subtract 27 MHz from counter reading 
//  - Enter the difference in Hz (i.e. -245)
int CalFactor = 0; //This value needs to be modified accordingly

//_________________________Enter frequency limits (KHz) below:__________________________________
const unsigned long F_min = 1000000;    // Lower Frequency Limit 1 MHz
const unsigned long F_max = 112500000;  // Upper Frequency Limit 112,5 MHz

// include the library code:
#include <LiquidCrystal.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Wire.h>

// Set up MCU pins
#define encoderPinA              3 
#define encoderPinB              4
#define Resolution               6
#define RS                       7 
#define E                        8 
#define DB4                      9
#define DB5                     10 
#define DB6                     11 
#define DB7                     12
#define FreqDown                A0
#define FreqUp                  A1
#define Offset                  A2
#define BandSelect              A3

// Set sI5351A I2C address
#define Si5351A_addr          0x60 

// Define Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// configure variables 
byte fStepcount=3,offsetFlag=0,band;
String resolution = "1 KHz  ";
char buffer[300] = "";
int IndiceCount=0,StartCount=0,counter=0,indices[13];
unsigned int tcount=2,encoderA,encoderB,encoderC=1;
unsigned long time,fStep=1000,XtalFreq=27000000;
unsigned long mult=0,Freq_1,Freq_2,prevFreq,debounce,DebounceDelay=500000;

//******************************************************************
// Clock - interrupt routine used as master timekeeper
//******************************************************************
void PPSinterrupt()
{
  tcount++;
  if (tcount == 4)                               // Start counting the 2.7 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
    loop();  
  }
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {     
    TCCR1B = 0;                                  //Turn off counter
    XtalFreq = mult * 0x10000 + TCNT1;           //Calculate correction factor
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
  }
  
}

// Timer 1 overflow intrrupt vector.
ISR(TIMER1_OVF_vect) 
{ 
  mult++;                                          //Increment multiplier
  TIFR1 = (1<<TOV1);                               //Clear overlow flag 
}


void setup()
{

  Wire.begin(1);                  // join I2C bus (address = 1)
  si5351aStart();

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag

  // Add CalFactor to the Si5351 crystal frequency
  {
    XtalFreq += CalFactor;
    detachInterrupt(0); // Disable the 1pps interrupt
  } 

  // Make XtalFreq compatible with correction variable
  XtalFreq *= 4;

  // Set up the LCD's number of columns and rows 
  lcd.begin(16,2); 

  // Set up rotary encoder
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);         // internal pull-up enabled 
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);         // internal pull-up enabled  

  // Set up push buttons
  pinMode(Resolution, INPUT);
  digitalWrite(Resolution, HIGH);         // internal pull-up enabled
  pinMode(BandSelect, INPUT);
  digitalWrite(BandSelect, HIGH);         // internal pull-up enabled
  pinMode(FreqDown, INPUT);
  digitalWrite(FreqDown, HIGH);           // internal pull-up enabled
  pinMode(Offset, INPUT);
  digitalWrite(Offset, HIGH);             // internal pull-up enabled
  pinMode(FreqUp, INPUT);
  digitalWrite(FreqUp, HIGH);             // internal pull-up enabled
  pinMode(FreqDown, INPUT);
  digitalWrite(FreqDown, HIGH);           // internal pull-up enabled

  Serial.begin(4800);                     // connect to the   port @ 4800 baud

  lcd.display();                          // initialize LCD
  lcd.setCursor(0,0);
  lcd.print("Si5351 PLL Synth");
  lcd.setCursor(0,1);
  lcd.print("V.5.1  de SV1ONW");
  delay (3000);
  lcd.clear();
  lcd.setCursor(0,1);
  {
    lcd.print("Step:");                   // Frequency step resolution will display  
    lcd.setCursor(6,1);                   // in this position here. 
    lcd.print(resolution);                // Show it.
  } 

  TCCR1B = 0;                             //Disable Timer5

  Freq_1 = Freq_array [0] [0];            // At start up load the first band in the the
  Freq_2 = Freq_array [0] [1];            // Freq_array variable.

  // Set CLK0 to 2.7 MHz
  si5351aSetFreq(SYNTH_MS_0,2700000); 

  if(Freq_2 == 0)
  {
    Si5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2
  }
  else
  {
    Si5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2
    Freq_2 = Freq_array [band] [1];              // Load CLK2
    si5351aSetFreq(SYNTH_MS_2, Freq_2);          // Set CLK2 frequency
  }

  setfreq();                                     // Now display and set CLK1 frequency 

}



//******************************************************************
// Loop starts here:
// Loops consecutively to check MCU pins for activity
//******************************************************************
void loop()
{   
  {
    if(tcount==1)                                  
    { // Update the SI5351A after every correction 
      si5351aSetFreq(SYNTH_MS_1, Freq_1);
      if(Freq_2 > 0) si5351aSetFreq(SYNTH_MS_2, Freq_2);
      tcount=2;
    }

    prevFreq = Freq_1;
    // Rotary encoder algorithm begins here
    byte encoderA = digitalRead(encoderPinA); 
    byte encoderB = digitalRead(encoderPinB);
    if ((encoderA == HIGH) && (encoderC == LOW))
    {
      if (encoderB == LOW) 
      {
        // Decrease frequency
        Freq_1 -= fStep; 
      }
      else
      {
        // Increase frequency
        Freq_1 += fStep;
      }
        if (Freq_1 > F_max) //Check the upper Frequency limit
        {
            Freq_1 = F_max; 
        }
        else if (Freq_1 < F_min) //Check the lower Frequency limit  
        {                         
            Freq_1 = F_min;
        } 
      setfreq();         //Update and display new frequency
      resDisplay();      //Update resolution display and set display timer
    }
    encoderC = encoderA;


    // The frequency step resolution selection begins here:
    if(digitalRead(Resolution) == LOW)
    {
      for(debounce=0; debounce < DebounceDelay; debounce++) {
      };
      fStepcount++;
      if(fStepcount>6)fStepcount=0;
      setResolution();     // Call the set resolution subroutine
    }

    // Band selection begins here:
    if(digitalRead(BandSelect) == LOW)
    {
      for(debounce=0; debounce < DebounceDelay; debounce++) {
      };
      band=band+1;                        // Increment band selection
      if(Freq_array [band] [0]==0)band=0; // Check for end of frequency array
      if(Freq_array [band] [1] == 0)      // Is CLK2 = 0? 
      {
        Si5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2
      }
      else
      {
        Si5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2
        Freq_2 = Freq_array [band] [1];              // Load CLK2 frequency
        si5351aSetFreq(SYNTH_MS_2, Freq_2);          // Set CLK2 frequency
      }
      Freq_1 = Freq_array [band] [0];                // Load CLK1 frequency 
      setfreq();                                     // Display and set CLK1 frequency
    }

    // Frequency offset algorithm begins here:  
    if(digitalRead(Offset) == LOW && offsetFlag == 0) // Check for offset pin A2 LOW
    { 
      offsetFlag = 1;                                 // Set flag
      Freq_1 += fOffset;                              // Add offset frequency
      lcd.setCursor(15,0);                            // Display a "*" on the LCD
      lcd.print("*");
      setfreq();                                      // Display and set CLK1 frequency + offset
    }

    if(digitalRead(Offset) == HIGH && offsetFlag == 1) // Check for offset pin A2 HIGH
    {
      offsetFlag = 0;                                  // Reset flag
      Freq_1 -= fOffset;                               // Subtract the offset frequency
      lcd.setCursor(15,0);                             // Clear the "*" on the LCD
      lcd.print(" ");
      setfreq();                                       // Display and set CLK1 frequency - offset
    }

    // Frequency Up/Down pushbutton algorithm begin here:
    if(digitalRead(FreqUp) == LOW)  // Check for frequency up pushbutton A1 LOW
    {
      for(debounce=0; debounce < DebounceDelay; debounce++) {
      };
      // Increase frequency by the selected frequency step 
      Freq_1 += fStep;                                // Increase CLK1 by frequency step   
      if (Freq_1 > F_max)
     {
     Freq_1 = F_max;
     }
      setfreq();                                      // Set and display new frequency
      resDisplay();                                   // Call the resolution display subroutine
    }

    if(digitalRead(FreqDown) == LOW) // Check for frequency up pushbutton A1 LOW
    {
      for(debounce=0; debounce < DebounceDelay; debounce++) {
      };
      // Decrease frequency by the selected frequency step and check for 1-80 MHz limits 
      Freq_1 -= fStep;                               // Decrease CLK1 by frequency step 
      if (Freq_1 < F_min)
     {
     Freq_1 = F_min;
     }
      setfreq();                                     // Set and display new frequency
      resDisplay();                                  // Call the resolution display subroutine
    }     
  }
}


//******************************************************************
// Display and set the Si5351A frequency
//******************************************************************
void setfreq()
{
  unsigned long  Freq_temp = Freq_1; // Temporarily store Freq_1
  switch(Freq_array [band] [2])      // Get math function from frequency array
  {
  case 1:  // If math function is 1 then add Freq_2 and Freq_1 for display
    Freq_temp = Freq_1 + Freq_2;
    break;
  case 2:  // If math function is 2 then subtract Freq_2 from Freq_1 for display
    Freq_temp = Freq_1 - Freq_2;
    break;
  }
  if (Freq_temp > F_max) //Check the upper Frequency limit
        {
            Freq_temp = F_max; 
            Freq_1 = prevFreq;
        }
  else if (Freq_temp < F_min) //Check the lower Frequency limit  
        {                         
            Freq_temp = F_min;
            Freq_1 = prevFreq;
        } 
        
  char buf[10];

  // Print frequency to the LCD
  lcd.setCursor(0,0);

  ltoa(Freq_temp,buf,10);

  if (Freq_temp < 1000000)
  {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(" KHz     ");         
  }

  if (Freq_temp >= 1000000 && Freq_temp < 10000000)
  {
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(" KHz   ");
  }

  if (Freq_temp >= 10000000 && Freq_temp < 100000000)
  {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(" KHz  ");
  }

  if (Freq_temp >= 100000000)
  {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(',');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print('.');
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(buf[8]);
    lcd.print(" KHz ");
  }  
  
  si5351aSetFreq(SYNTH_MS_1, Freq_1); // Set CLK1 frequency
}


//******************************************************************
// Set the frequency step resolution
//******************************************************************
void setResolution()
{
  switch(fStepcount)
  {
  case 0:
    fStep=1;
    resolution = "1 Hz  ";
    break;
  case 1:
    fStep=10;
    resolution = "10 Hz  ";
    break;
  case 2:
    fStep=100;
    resolution = "100 Hz ";
    break;
  case 3:
    fStep=1000;
    resolution = "1 KHz  ";
    break;
  case 4:
    fStep=10000;
    resolution = "10 KHz ";
    break;
  case 5:
    fStep=100000;
    resolution = "100 KHz";
    break;
  case 6:
    fStep=1000000;
    resolution = "1 MHz  ";
    break;
  }
  resDisplay();
}

//******************************************************************
// Display the frequency step resolution and 
// set a 10 second display timer
//******************************************************************
void resDisplay()
{
  lcd.setCursor(6,1);
  lcd.print(resolution);
  time = millis()+10000;
}

//******************************************************************
//  Si5351 Multisynch processing
//******************************************************************
void si5351aSetFreq(int synth, unsigned long  freq)
{
  unsigned long long CalcTemp;
  unsigned long  a, b, c, p1, p2, p3;

  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  a = ((XtalFreq * 33) /4) / freq; // 33 is derived from 900/27 MHz
  CalcTemp = round((XtalFreq * 33) /4) % freq;
  CalcTemp *= c;
  CalcTemp /= freq ; 
  b = CalcTemp;  // Calculated numerator


  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);  
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}


//******************************************************************
//  Si5351 initialization routines
//******************************************************************
void si5351aStart()
{
  // Initialize Si5351A
  Si5351_write(XTAL_LOAD_CAP,0b11000000);      // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL,0b00000000); // Enable all outputs
  Si5351_write(CLK0_CONTROL,0b00001111);       // Set PLLA to CLK0, 8 mA output
  Si5351_write(CLK1_CONTROL,0b00001111);       // Set PLLA to CLK1, 8 mA output
  Si5351_write(CLK2_CONTROL,0b00101111);       // Set PLLB to CLK2, 8 mA output
  Si5351_write(PLL_RESET,0b10100000);          // Reset PLLA and PLLB


  // Set PLLA and PLLB to 900 MHz
  unsigned long  a, b, c, p1, p2, p3;

  a = 33;           // Derived from 900/27 MHz
  b = 0;            // Numerator
  c = 0xFFFFF;      // Denominator derived from max bits 2^20

  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to PLL registers
  Si5351_write(SYNTH_PLL_A, 0xFF);
  Si5351_write(SYNTH_PLL_A + 1, 0xFF);
  Si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  Si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  Si5351_write(SYNTH_PLL_B, 0xFF);
  Si5351_write(SYNTH_PLL_B + 1, 0xFF);
  Si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  Si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));

}

//******************************************************************
//Write I2C data routine
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}
//------------------------------------------------------------------------------------------------------
