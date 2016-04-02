// This #include statement was automatically added by the Particle IDE.
//#include "reent.h"

// This #include statement was automatically added by the Particle IDE.
#include "Neopix.h"

// This #include statement was automatically added by the Particle IDE.
#include "math.h"

// This #include statement was automatically added by the Particle IDE.
#include "ieeefp.h"

// This #include statement was automatically added by the Particle IDE.
//#include "ansi.h"

// This #include statement was automatically added by the Particle IDE.
//#include "reent.h"

// This #include statement was automatically added by the Particle IDE.
//#include "ansi.h"

// This #include statement was automatically added by the Particle IDE.
#include "ieeefp.h"

// This #include statement was automatically added by the Particle IDE.
#include "math.h"

// This #include statement was automatically added by the Particle IDE.
#include "Neopix.h"
#include "math.h"
// This #include statement was automatically added by the Particle IDE.
//#include "SparkFunLSM9DS1.h"

// This #include statement was automatically added by the Particle IDE.
//#include "SparkFunLSM9DS1.h"

// This #include statement was automatically added by the Particle IDE.
#include "Neopix.h"
#define PIXEL_COUNT 16
#define PIXEL_PIN WKP
#define PIXEL_TYPE WS2812


  const char RED_LED_ON = 'R';
    const char RED_LED_OFF = 'r';
    const char BLUE_LED_ON = 'B';
    const char BLUE_LED_OFF = 'b';
    const char GREEN_LED_ON = 'G';
    const char GREEN_LED_OFF = 'g';
    const char YELLOW_LED_ON = 'Y';
    const char YELLOW_LED_OFF = 'y';

    //ON / OFF messages for the ring of LED's
    const char ENCODER_STATE_0 = '0';
    const char ENCODER_STATE_1 = '1';
    const char ENCODER_STATE_2 = '2';
    const char ENCODER_STATE_3 = '3';
    const char ENCODER_STATE_4 = '4';
    const char ENCODER_STATE_5 = '5';
    const char ENCODER_STATE_6 = '6';
    const char ENCODER_STATE_7 = '7';
    const char ENCODER_STATE_8 = '8';
    const char ENCODER_STATE_9 = '9';
    const char ENCODER_STATE_10 = 'A';
    const char ENCODER_STATE_11 = 'H';
    const char ENCODER_STATE_12 = 'C';
    const char ENCODER_STATE_13 = 'D';
    const char ENCODER_STATE_14 = 'E';
    const char ENCODER_STATE_15 = 'F';
    int i;

    const char ENCODER_STATE[]={ENCODER_STATE_0,ENCODER_STATE_1,ENCODER_STATE_2,ENCODER_STATE_3,ENCODER_STATE_4,ENCODER_STATE_5,ENCODER_STATE_6,ENCODER_STATE_7,ENCODER_STATE_8,ENCODER_STATE_9,ENCODER_STATE_10,ENCODER_STATE_11,ENCODER_STATE_12,ENCODER_STATE_13,ENCODER_STATE_14,ENCODER_STATE_15};




//-------------*/                             
#include <vector>
#include <limits.h>
#include <string>

///////IMU SETUP
//LSM9DS1 imu;                // lsm9d1 is a class from the library....
// SDO_XM and SDO_G are both pulled high, so our addresses are:
//#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
//#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// http://www.ngdc.noaa.gov/geomag-web/#declination
//#define DECLINATION 4.76 // Declination (degrees) in Boulder, CO.
unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 500;
//////END IMU SETUP



SYSTEM_MODE(MANUAL);  //set this if using websockets (requires changing)

////////////////////////////////////////////////////////////////
//function declarations... for use if compiling on a machine (compiles as .cpp not .ino on machine)
void COMMUNICATE();
void UPDATELIGHTS();
void UPDATEENCODERLIGHTS();
void CHECKENCODERCHANGED();
void doEncoderA();
// Interrupt on B changing state, same as A above
void doEncoderB();
//int PREVIOUS_ENCODERLEDSTATUS;  //to replace prevPos
//////////////////////////////////////////////////////////////////


int REDLED = D2;
int BLUELED = D3;
int GREENLED = D4;
int YELLOWLED = D5;
///int POT = A5;
bool RESULT=0;
bool RESULT2=0;
String ENCODERSTRING;
String BLUELEDSTRING;
int ENCODERINT;


char EncoderPosBuffer[0];
int EncoderPosInt=EncoderPosBuffer[0]-'0';
int EncoderPos;
int RINGOUTPUT=WKP;  //MAKE SURE THIS ISNT BEING SHARED BY ANOTHER PIN ON REDBOARD
int REDBUTTON = D6;
int BLUEBUTTON = D7;
int GREENBUTTON = A0;
int YELLOWBUTTON = A1;





/////ROT. ENCODER//////////
volatile bool A_set = false;    //code may POSSIBLY optimize better without 
volatile bool B_set = false;    //volatile, may be worth experimenting if latency becomes an issue
volatile int encoderPos = 0;
volatile int ENCODERLEDSTATUS=0; //to replace encoderPos

int ENCODERA=A3;    //you can swap these to change encoder orientation/direction
int ENCODERB=A2;
int prevPos = 0;
int PREVIOUS_ENCODERLEDSTATUS;  //to replace prevPos
int value = 0;
int PREVIOUS_ENCODERA_STATUS;
int PREVIOUS_ENCODERB_STATUS;
//////////END ROTARY ENCODER////////////
int REDBUTTONSTATUS=LOW;
int BLUEBUTTONSTATUS=LOW;
int GREENBUTTONSTATUS=LOW;
int YELLOWBUTTONSTATUS=LOW;
int PREV_REDBUTTONSTATUS=LOW;
int PREV_BLUEBUTTONSTATUS=LOW;
int PREV_GREENBUTTONSTATUS=LOW;
int PREV_YELLOWBUTTONSTATUS=LOW;

int REDBUTTONPRESSED=0;
int BLUEBUTTONPRESSED=0;
int GREENBUTTONPRESSED=0;
int YELLOWBUTTONPRESSED=0;

int REDLEDSTATUS=LOW;
int BLUELEDSTATUS=LOW;
int GREENLEDSTATUS=LOW;
int YELLOWLEDSTATUS=LOW;
int LEDSTATUS[4]={REDLEDSTATUS,BLUELEDSTATUS,GREENLEDSTATUS,YELLOWLEDSTATUS};
int BUTTONSTATUS[4]={REDBUTTONSTATUS,BLUEBUTTONSTATUS,GREENBUTTONSTATUS,YELLOWBUTTONSTATUS};
int PREV_BUTTONSTATUS[4]={REDBUTTONSTATUS,BLUEBUTTONSTATUS,GREENBUTTONSTATUS,YELLOWBUTTONSTATUS};

int BUTTONPRESSEDARRAY[4]={REDBUTTONPRESSED,BLUEBUTTONPRESSED,GREENBUTTONPRESSED,YELLOWBUTTONPRESSED};
int RINGLEDSTATUS=0;

//FLAGS ARE USED FOR COMMUNICATION

int BUTTONPRESSED=0;
int REDBUTTONFLAG=0;
int BLUEBUTTONFLAG=0;
int GREENBUTTONFLAG=0;
int YELLOWBUTTONFLAG=0;
int ENCODERFLAG=0;         //behaves different from other flags, used to indicate update lights on ring, else, is lowered by communication
int ENCODERFLAGFROMUNITY=0;
int WAITFORRELEASE=0;
int COMMUNICATEFLAG=0;
int DEBOUNCETIMER=10;    // number of milliseconds to wait before allowing another input
int REDBUTTONTIMER=0;
int BLUEBUTTONTIMER=0;
int GREENBUTTONTIMER=0;
int YELLOWBUTTONTIMER=0;
char currentchar;



Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);




//SYSTEM_MODE(MANUAL);




// It runs only once when the device boots up or is reset.
void setup() {

 //open serial connection
 Serial1.begin(115200);
  //Serial1.begin(115200); //begin serial connection at 9600 bps  
  ///initiate LED ring
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(100);
  //initialize the led ring to 0, (top, slightly to the right)
  strip.clear();
  strip.setPixelColor(encoderPos, 0, 255, 255);
  strip.show();
  
  //attachInterrupt(ENCODERA, doEncoderA, CHANGE);               // previously used interrupts, but caused issues with neopixel
  //attachInterrupt(ENCODERB, doEncoderB, CHANGE);  
    
  pinMode(REDLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(YELLOWLED, OUTPUT);
  pinMode(RINGOUTPUT, OUTPUT);
  
  pinMode(ENCODERA,INPUT_PULLUP);
  pinMode(ENCODERB,INPUT_PULLUP);
  
  pinMode(REDBUTTON,INPUT_PULLUP);
  pinMode(BLUEBUTTON,INPUT_PULLUP);
  pinMode(GREENBUTTON,INPUT_PULLUP);
  pinMode(YELLOWBUTTON,INPUT_PULLUP);

  // imu setup
  //imu.settings.device.commInterface = IMU_MODE_I2C;
  //imu.settings.device.mAddress = LSM9DS1_M;
  //imu.settings.device.agAddress = LSM9DS1_AG;

 //  uint16_t status = initLSM9DS1();  
// startTime = millis();

  
// rotary 
PREVIOUS_ENCODERA_STATUS=digitalRead(ENCODERA);
PREVIOUS_ENCODERB_STATUS=digitalRead(ENCODERB);


//pinMode(RX, OUTPUT);                    //not sure if this is necessary
//pinMode(TX, INPUT);

//Serial1.print("AT+BAUD8");
delay(1000);
ENCODERLEDSTATUS=0;

}//end setup








void loop()                     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{

//POLL INPUTS AT BEGINNING OF LOOP
REDBUTTONSTATUS=digitalRead(REDBUTTON);
BLUEBUTTONSTATUS=digitalRead(BLUEBUTTON);
GREENBUTTONSTATUS=digitalRead(GREENBUTTON);
YELLOWBUTTONSTATUS=digitalRead(YELLOWBUTTON);

  if(digitalRead(ENCODERA)!=PREVIOUS_ENCODERA_STATUS)
  {PREVIOUS_ENCODERA_STATUS=digitalRead(ENCODERA);doEncoderA();}
  
  if(digitalRead(ENCODERB)!=PREVIOUS_ENCODERB_STATUS)
  {PREVIOUS_ENCODERB_STATUS=digitalRead(ENCODERB);doEncoderB();}

CHECKENCODERCHANGED();


//POLL OUTPUTS AT BEGINNING OF LOOP
REDLEDSTATUS=digitalRead(REDLED);
BLUELEDSTATUS=digitalRead(BLUELED);
GREENLEDSTATUS=digitalRead(GREENLED);
YELLOWLEDSTATUS=digitalRead(YELLOWLED);

BUTTONSTATUS[0]=REDBUTTONSTATUS;BUTTONSTATUS[1]=BLUEBUTTONSTATUS;BUTTONSTATUS[2]=GREENBUTTONSTATUS;BUTTONSTATUS[3]=YELLOWBUTTONSTATUS;


//STATES

//1 - NO BUTTON PRESSED      //2 - BUTTON PRESSED (BEING HELD DOWN WAITING FOR RELEASE)      //3 - BUTTON PRESSED FOR FIRST TIME




BUTTONPRESSED=((REDBUTTONSTATUS==LOW)||(BLUEBUTTONSTATUS==LOW)||(GREENBUTTONSTATUS==LOW)||(YELLOWBUTTONSTATUS==LOW));

if((BUTTONPRESSED==1))//&&(WAITFORRELEASE==0))          //button pressed for first time,so write values and wait for release flag to go high
 {
       
            //    WAITFORRELEASE=1;
             
               if(REDBUTTONSTATUS==LOW)                               //raise a button pressed flag
               {REDBUTTONPRESSED=1;}
            
               if(REDBUTTONPRESSED&&(REDBUTTONSTATUS==HIGH))          ///lower a button pressed flag
               {REDBUTTONPRESSED=LOW;}
            
               if(BLUEBUTTONSTATUS==LOW)
               {BLUEBUTTONPRESSED=1;}
            
               if(BLUEBUTTONPRESSED&&(BLUEBUTTONSTATUS==HIGH))
               {BLUEBUTTONPRESSED=0;}
               
               if(GREENBUTTONSTATUS==LOW)
               {GREENBUTTONPRESSED=1;}
            
               if(GREENBUTTONPRESSED&&(GREENBUTTONSTATUS==HIGH))
               {GREENBUTTONPRESSED=LOW;}
               
               if(YELLOWBUTTONSTATUS==LOW)
               {YELLOWBUTTONPRESSED=1;}
            
               if(YELLOWBUTTONPRESSED&&(YELLOWBUTTONSTATUS==HIGH))
               {YELLOWBUTTONPRESSED=LOW;}
               
               UPDATELIGHTS();                       // update the light, set a debounce timer before 
                                                     //allowing another input from respective button
               COMMUNICATE();                        //update lights upon pressing down
               
    
    }
/*
if(Serial1!=true)
{
    Serial1.begin(9600);
    Serial1.flush();
}
*/
if(Serial1.available()>0)
    {
    COMMUNICATE();
    }
    /*
if(millis()-lastPrint>PRINT_RATE)
{
     printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
}   
*/
}//end loop/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void COMMUNICATE() //OCCURS UPON SERIAL DATA RECEIVED OR INPUTS ON BOX CHANGED
{
  
             // RED
    currentchar=Serial1.peek();
    
    if(currentchar==RED_LED_ON)
             {
             REDLEDSTATUS=1;
             Serial1.read();
             currentchar='q';                // set character to trash so it doesnt get read twice
             digitalWrite(REDLED,REDLEDSTATUS);             
             }
     
     currentchar=Serial1.peek();
     
     if(currentchar==RED_LED_OFF)
             {
             REDLEDSTATUS=0;
             Serial1.read();
             currentchar='q';
             digitalWrite(REDLED,REDLEDSTATUS);
             }

             //BLUE

    if(currentchar==BLUE_LED_ON)
             {
             BLUELEDSTATUS=1;
             Serial1.read();
             currentchar='q';                // set character to trash so it doesnt get read twice
             digitalWrite(BLUELED,BLUELEDSTATUS);             
             }
     
     currentchar=Serial1.peek();
     
     if(currentchar==BLUE_LED_OFF)
             {
             BLUELEDSTATUS=0;
             Serial1.read();
             currentchar='q';
             digitalWrite(BLUELED,BLUELEDSTATUS);
             }

             // GREEN

    if(currentchar==GREEN_LED_ON)
             {
             GREENLEDSTATUS=1;
             Serial1.read();
             currentchar='q';                // set character to trash so it doesnt get read twice
             digitalWrite(GREENLED,GREENLEDSTATUS);             
             }
     
     currentchar=Serial1.peek();
     
     if(currentchar==GREEN_LED_OFF)
             {
             GREENLEDSTATUS=0;
             Serial1.read();
             currentchar='q';
             digitalWrite(GREENLED,GREENLEDSTATUS);
             }

             // YELLOW

    if(currentchar==YELLOW_LED_ON)
             {
             YELLOWLEDSTATUS=1;
             Serial1.read();
             currentchar='q';                // set character to trash so it doesnt get read twice
             digitalWrite(YELLOWLED,YELLOWLEDSTATUS);             
             }
     
     currentchar=Serial1.peek();
     
     if(currentchar==YELLOW_LED_OFF)
             {
             YELLOWLEDSTATUS=0;
             Serial1.read();
             currentchar='q';
             digitalWrite(YELLOWLED,YELLOWLEDSTATUS);
             }
    
   
    

     
         currentchar=Serial1.peek();
         
         for(i=0;i<16;i++)
         {
            if(currentchar==ENCODER_STATE[i])
                {   
                    Serial1.read();
                    currentchar='q';
                    //ENCODERSTRING=Serial1.readStringUntil('X'); //read the value of ringled after 'e'
                    ENCODERLEDSTATUS=i;
                    PREVIOUS_ENCODERLEDSTATUS=i;
                    strip.clear();                              //next 3 lines to display on ringled using library
                    strip.setPixelColor(ENCODERLEDSTATUS, 0, 255, 255);
                    strip.show();

                }
         }
    //////////////////////////////////////////////////////////////////////////////
    
       if(REDBUTTONFLAG)                               // Sending state to unity
         {
             if(REDLEDSTATUS==1)
            //Serial1.printf("%c",'R');
            Serial1.printf("%c",RED_LED_ON); 
             else 
             //Serial1.printf("%c",'r');
             Serial1.printf("%c",RED_LED_OFF);
          REDBUTTONFLAG=0;
         }
         
       if(BLUEBUTTONFLAG)                               // Sending state to unity
         {
             if(BLUELEDSTATUS==1)
            //Serial1.printf("%c",'R');
            Serial1.printf("%c",BLUE_LED_ON); 
             else 
             //Serial1.printf("%c",'r');
             Serial1.printf("%c",BLUE_LED_OFF);
          BLUEBUTTONFLAG=0;
         }
         
       if(GREENBUTTONFLAG)                               // Sending state to unity
         {
             if(GREENLEDSTATUS==1)
            //Serial1.printf("%c",'R');
            Serial1.printf("%c",GREEN_LED_ON); 
             else 
             //Serial1.printf("%c",'r');
             Serial1.printf("%c",GREEN_LED_OFF);
          GREENBUTTONFLAG=0;
         }
         
       if(YELLOWBUTTONFLAG)                               // Sending state to unity
         {
             if(YELLOWLEDSTATUS==1)
            //Serial1.printf("%c",'R');
            Serial1.printf("%c",YELLOW_LED_ON); 
             else 
             //Serial1.printf("%c",'r');
             Serial1.printf("%c",YELLOW_LED_OFF);
          YELLOWBUTTONFLAG=0;
         }
         
      if(ENCODERFLAG==1)
         {
             Serial1.printf("%c",ENCODER_STATE[ENCODERLEDSTATUS]);
             ENCODERFLAG=0;
         }
         
         
}//end of communicate

void UPDATELIGHTS()
{       
        //UPDATE LEDS
            if((REDBUTTONPRESSED==1)&&((millis()-REDBUTTONTIMER)>=DEBOUNCETIMER))          // millis() - redbuttontimer will grow and grow
        {
            digitalWrite(REDLED,!REDLEDSTATUS);
            REDLEDSTATUS=!REDLEDSTATUS;
            REDBUTTONFLAG=1;
            REDBUTTONTIMER=millis();
        }
        else{
            if(REDBUTTONPRESSED==1){REDBUTTONTIMER=millis();}
            }
        if((BLUEBUTTONPRESSED==1)&&((millis()-BLUEBUTTONTIMER)>=DEBOUNCETIMER))
        {
            digitalWrite(BLUELED,!BLUELEDSTATUS);
            BLUELEDSTATUS=!BLUELEDSTATUS;
            BLUEBUTTONFLAG=1;
            BLUEBUTTONTIMER=millis();
        }
        else if(BLUEBUTTONPRESSED==1){BLUEBUTTONTIMER=millis();}

        if((GREENBUTTONPRESSED==1)&&((millis()-GREENBUTTONTIMER)>=DEBOUNCETIMER))
        {
            digitalWrite(GREENLED,!GREENLEDSTATUS);
            GREENLEDSTATUS=!GREENLEDSTATUS;
            GREENBUTTONFLAG=1;
            GREENBUTTONTIMER=millis();
        }
        else  if(GREENBUTTONPRESSED==1){GREENBUTTONTIMER=millis();}

        if((YELLOWBUTTONPRESSED==1)&&((millis()-YELLOWBUTTONTIMER)>=DEBOUNCETIMER))
        {
            digitalWrite(YELLOWLED,!YELLOWLEDSTATUS);
            YELLOWLEDSTATUS=!YELLOWLEDSTATUS;
            YELLOWBUTTONFLAG=1;
            YELLOWBUTTONTIMER=millis();
        }
        else if(YELLOWBUTTONPRESSED==1){YELLOWBUTTONTIMER=millis();}
        
}

void UPDATEENCODERLIGHTS()
{
            
                if(ENCODERFLAG==1)
        {
            strip.clear();
             strip.setPixelColor(ENCODERLEDSTATUS, 0, 255, 255);
             strip.show();
        }   
        
    
    
}

void CHECKENCODERCHANGED(){
            if (PREVIOUS_ENCODERLEDSTATUS != ENCODERLEDSTATUS) {
        PREVIOUS_ENCODERLEDSTATUS = ENCODERLEDSTATUS;
        //Serial.print(ENCODERLEDSTATUS);
        ENCODERFLAG=1;
        UPDATEENCODERLIGHTS();
        COMMUNICATE();
        
    }
}

void doEncoderA(){
  if( digitalRead(ENCODERA) != A_set ) {  // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set ) 
    {
        if(ENCODERLEDSTATUS==15)
        {ENCODERLEDSTATUS=0;}else
      ENCODERLEDSTATUS += 1;
    
    }
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
   if(digitalRead(ENCODERB) != B_set) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set )
    {
    if(ENCODERLEDSTATUS==0)
    {ENCODERLEDSTATUS=15;}else
      ENCODERLEDSTATUS -= 1;
    }
  }
}




/*
void setupGyro()
{
  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3; // 59.5Hz ODR
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 8; // Set accel scale to +/-8g.
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 1; // Set accel to 10Hz.
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution 
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;  
}

void setupMag()
{
  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
  // [tempCompensationEnable] enables or disables 
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
}

uint16_t initLSM9DS1()
{
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  //setupTemperature(); // Set up temp sensor parameter
  
  return imu.begin();
}

void printSensorReadings()
{
  //float runTime = (float)(millis() - startTime) / 1000.0;
  //float accelRate = (float)accelReadCounter / runTime;
  //float gyroRate = (float)gyroReadCounter / runTime;
  //float magRate = (float)magReadCounter / runTime;

  Serial.print("A: ");
  Serial.print(imu.calcAccel(imu.ax));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az));
  Serial.print(" g \t| ");

  Serial.print("G: ");
  Serial.print(imu.calcGyro(imu.gx));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz));
  Serial.print(" dps \t| ");

  Serial.print("M: ");
  Serial.print(imu.calcMag(imu.mx));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz));
  Serial.print(" Gs \t| ");
 
  //Serial.println();
}


void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(
float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

*/


