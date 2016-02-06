// This #include statement was automatically added by the Particle IDE.
#include "Neopix.h"

// This #include statement was automatically added by the Particle IDE.
#include "SparkFunLSM9DS1/SparkFunLSM9DS1.h"

// This #include statement was automatically added by the Particle IDE.
#include "SparkFunLSM9DS1/SparkFunLSM9DS1.h"

// This #include statement was automatically added by the Particle IDE.
#include "Neopix.h"
#define PIXEL_COUNT 16
#define PIXEL_PIN WKP
#define PIXEL_TYPE WS2812
//-------------*/                             
#include <vector>
#include <limits.h>
#include <string>

///////IMU SETUP
LSM9DS1 imu;                // lsm9d1 is a class from the library....
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 4.76 // Declination (degrees) in Boulder, CO.
unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 500;
//////END IMU SETUP



//SYSTEM_MODE(MANUAL);  //set this if using websockets (requires changing)

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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);


// It runs only once when the device boots up or is reset.
void setup() {

 //open serial connection
  Serial.begin(115200); //begin serial connection at 9600 bps  
  ///initiate LED ring
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(233);
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
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
// rotary 
PREVIOUS_ENCODERA_STATUS=digitalRead(ENCODERA);
PREVIOUS_ENCODERB_STATUS=digitalRead(ENCODERB);


}//end setup




void loop()                     //////////////////////////////////////////////////////////////////////////
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



if(Serial.available()>0)
    {
    COMMUNICATE();
    }
    
      // imu.accelAvailable() returns 1 if new accelerometer
  // data is ready to be read. 0 otherwise.
  if (imu.accelAvailable())
  {
    imu.readAccel();
    accelReadCounter++;
  }
  // imu.gyroAvailable() returns 1 if new gyroscope
  // data is ready to be read. 0 otherwise.
  if (imu.gyroAvailable())
  {
    imu.readGyro();
    gyroReadCounter++;
  }
  // imu.magAvailable() returns 1 if new magnetometer
  // data is ready to be read. 0 otherwise.
  if (imu.magAvailable())
  {
    imu.readMag();
    magReadCounter++;
  }
  // Every PRINT_RATE milliseconds, print sensor data:
  if ((lastPrint + PRINT_RATE) < millis())
  {
    printSensorReadings();
    lastPrint = millis();
  }


}//end loop/////////////////////////////////////////////////////////////////////////////////////


void COMMUNICATE() //OCCURS UPON SERIAL DATA RECEIVED OR INPUTS ON BOX CHANGED
{
//get info from virtual reality
//doesnt use flags, updates status variables, and writes to leds directly in the code
    if(Serial.peek()=='r')
    {
     Serial.read();
     if(Serial.peek()=='1')
     {REDLEDSTATUS=1;
     Serial.read();
     digitalWrite(REDLED,REDLEDSTATUS);
     }
     if(Serial.peek()=='0')
     {REDLEDSTATUS=0;
     Serial.read();
     digitalWrite(REDLED,REDLEDSTATUS);
     }
     
    }
        if(Serial.peek()=='b')
      {
     Serial.read();
         if(Serial.peek()=='1')
     {BLUELEDSTATUS=1;
     Serial.read();
     digitalWrite(BLUELED,BLUELEDSTATUS);
     }
        if(Serial.peek()=='0')
     {BLUELEDSTATUS=0;
     Serial.read();
     digitalWrite(BLUELED,BLUELEDSTATUS);
     }
    }
        if(Serial.peek()=='g')
    {
    {
     Serial.read();
       if(Serial.peek()=='1')
     {GREENLEDSTATUS=1;
     Serial.read();
     digitalWrite(GREENLED,GREENLEDSTATUS);
     }
       if(Serial.peek()=='0')
     {GREENLEDSTATUS=0;
     Serial.read();
     digitalWrite(GREENLED,GREENLEDSTATUS);
     }
    }
      
    }
        if(Serial.peek()=='y')
    {
     Serial.read();
     if(Serial.peek()=='1')
     {YELLOWLEDSTATUS=1;
     Serial.read();
     digitalWrite(YELLOWLED,YELLOWLEDSTATUS);
     }
     
     if(Serial.peek()=='0')
     {YELLOWLEDSTATUS=0;
     Serial.read();
     digitalWrite(YELLOWLED,YELLOWLEDSTATUS);
     }
     
    }
           
            if(Serial.peek()=='e')
    {
        Serial.read();
        ENCODERSTRING=Serial.readStringUntil('X'); //read the value of ringled after 'e'
        ENCODERLEDSTATUS=ENCODERSTRING.toInt();          //convert to int
        strip.clear();                              //next 3 lines to display on ringled using library
        strip.setPixelColor(ENCODERLEDSTATUS, 0, 255, 255);
        strip.show();
    }
    
    /*
         if(Serial.peek()=='t'){
     Serial.read();
     
     strip.clear();
        strip.setPixelColor(5, 0, 255, 255);
        strip.show();
        
     }
    
    
    
             if(Serial.peek()=='k'){
     Serial.read();
     ENCODERSTRING=Serial.readStringUntil('X');
        ENCODERINT=ENCODERSTRING.toInt();
     strip.clear();
        strip.setPixelColor(13, 0, 255, 255);
        strip.show();
        
     }
     */
//send info to virtual reality if flag was raised by real inputs, lower flags
//////////////////////////////////////////////

       
       if(REDBUTTONFLAG)                               // WHAT DO WE DO IF THERE IS INCOMING AND OUTGOING CHANGE???
         {
             Serial.printf("r%dX",REDLEDSTATUS);
               //Serial.print("r");
          REDBUTTONFLAG=0;
         }
        if(BLUEBUTTONFLAG)                              
         {
          Serial.printf("b%dX",BLUELEDSTATUS);
          BLUEBUTTONFLAG=0;
         }
      if(GREENBUTTONFLAG)                              
         {
          Serial.printf("g%dX",GREENLEDSTATUS);
          GREENBUTTONFLAG=0;
         }
      if(YELLOWBUTTONFLAG)                              
         {
          Serial.printf("y%dX",YELLOWLEDSTATUS);
          YELLOWBUTTONFLAG=0;
         } 
         
         if(ENCODERFLAG==1)
         {
             Serial.printf("e%dX",ENCODERLEDSTATUS);
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
  float runTime = (float)(millis() - startTime) / 1000.0;
  float accelRate = (float)accelReadCounter / runTime;
  float gyroRate = (float)gyroReadCounter / runTime;
  float magRate = (float)magReadCounter / runTime;

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