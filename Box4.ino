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

//SYSTEM_MODE(MANUAL);

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
  
  attachInterrupt(ENCODERA, doEncoderA, CHANGE);
  attachInterrupt(ENCODERB, doEncoderB, CHANGE);  
    
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
}//end setup




void loop()                     //////////////////////////////////////////////////////////////////////////
{
//POLL INPUTS AT BEGINNING OF LOOP
REDBUTTONSTATUS=digitalRead(REDBUTTON);
BLUEBUTTONSTATUS=digitalRead(BLUEBUTTON);
GREENBUTTONSTATUS=digitalRead(GREENBUTTON);
YELLOWBUTTONSTATUS=digitalRead(YELLOWBUTTON);
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


// if((WAITFORRELEASE==1)&&(BUTTONPRESSED==0))                           //if waiting for release and release happened
 //           {
  //             WAITFORRELEASE=0;
//            }

//}

//check if virtual world has changed
if(Serial.available()>0)
    {
    COMMUNICATE();
    }
    
    
    //UPDATELIGHTS();
    //COMMUNICATE();
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
