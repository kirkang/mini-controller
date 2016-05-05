#include <Wire.h>
#include <JY901.h>
//#include <avr/wdt.h>
#include <MotorControl.h>
#include <PIDcontroller.h>
#include <MatrixMath.h>
#define motor_N  (4)
using namespace std;

//initialize pid
float Atransinvtemp[motor_N*3] =
{
	 0, 0, 0.25 ,
	 0, 0, 0.25 ,
	 0, 0, 0.25 ,
	 0, 0, 0.25 
};
float Arotinvtemp[motor_N*3] =
{
	 -10, 0, 0 ,
	 10, 0, 0 ,
	 0, -5, 0 ,
	 0, 5, 0 
};
PIDcontroller myPID((float*)Atransinvtemp, (float*)Arotinvtemp, motor_N);


       float pidrottemp[motor_N*3] =
	{
		 0.15, 0.0005, 1.5 ,
		 0.15, 0.0005, 1.5 ,
		 0.30, 0.0005, 2.5 ,
		 0.30, 0.0005, 2.5 
	};
	float pidtranstemp[motor_N*3] =
	{
		 1.2, 0.0015, 30 ,
		 1.2, 0.0015, 30 ,
		 1.2, 0.0015, 30 ,
		 1.2, 0.0015, 30 
	};

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

float Ttoatal[motor_N];
float eta_target[6] = { 0, 0, 0, 0, 0, 0 };
float eta_n[6]={0,0,0,0,0,0};
float T = 0.1;

uint32_t timer1=0;
int timer_for_second=1;
float roll,pitch,yaw;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  JY901.StartIIC();   //IIC
  while(Serial.available() != 0){Serial.read();}       //不满足要求，清楚缓存
  myPID.SetPID((float*)pidrottemp, (float*)pidtranstemp);
  //wdt_enable(WDTO_4S);
  timer1=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  //OUT_30ms();
   JY901.GetAngle();
   roll=(float)JY901.stcAngle.Angle[0]/32768*PI;
   pitch=(float)JY901.stcAngle.Angle[1]/32768*PI;
   yaw=(float)JY901.stcAngle.Angle[2]/32768*PI;
   eta_n[3]=roll;
   eta_n[4]=pitch;
   eta_n[5]=yaw;
   myPID.controller((float*)eta_target, (float*)eta_n, T, (float*)Ttoatal);
   DO((float*)eta_n);
     

  while(millis()-timer1<=100){delay(1);}   //delay for 100ms
  
  
  if(timer_for_second==10)
  {
      Matrix.Print(eta_n,1,6,"Eta_n: ");
      Matrix.Print(Ttoatal, 1, 4, "Ttoatl:"); 
      Serial.println(millis());
      timer_for_second=0;
  }
    
  timer_for_second++; 
  timer1=millis();
}


void serialEvent() {
  if (Serial.available()) {
    // get the new byte:
    while (Serial.available()){   
        char inChar = (char)Serial.read();
        delay(2);
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n') {
          stringComplete = true;
        }
      }
     if(stringComplete){
       char   Buffer[100];
       inputString.toCharArray(Buffer, 20);
       Serial.print(inputString);
       if(Buffer[18]==10)
       {
         pidrottemp[0]=((Buffer[0]-'0')*10+Buffer[1]-'0')*(float)pow(0.1,Buffer[2]-'0');  //P1 
         pidrottemp[3]=pidrottemp[0];                                                     //P2
         pidrottemp[6]=((Buffer[3]-'0')*10+Buffer[4]-'0')*(float)pow(0.1,Buffer[5]-'0');  //P3
         pidrottemp[9]=pidrottemp[6];                                                     //P4
         pidrottemp[1]=((Buffer[6]-'0')*10+Buffer[7]-'0')*(float)pow(0.1,Buffer[8]-'0');  //I1
         pidrottemp[4]=pidrottemp[1];                                                     //I2
         pidrottemp[7]=((Buffer[9]-'0')*10+Buffer[10]-'0')*(float)pow(0.1,Buffer[11]-'0');//I3
         pidrottemp[10]=pidrottemp[7];                                                    //I4
         pidrottemp[2]=((Buffer[12]-'0')*10+Buffer[13]-'0')*(float)pow(0.1,Buffer[14]-'0');//D1
         pidrottemp[5]=pidrottemp[2];                                                      //D2
         pidrottemp[8]=((Buffer[15]-'0')*10+Buffer[16]-'0')*(float)pow(0.1,Buffer[17]-'0');//D3
         pidrottemp[11]=pidrottemp[8];                                                     //D4
         
         Matrix.Print(pidrottemp,3,4,"rottemp");
         myPID.SetPID((float*)pidrottemp, (float*)pidtranstemp);
         
                    //float pidrottemp[motor_N*3] =
                    //	{
                    //		 0.15, 0.0005, 1.5 ,
                    //		 0.15, 0.0005, 1.5 ,
                    //		 0.30, 0.0005, 2.5 ,
                    //		 0.30, 0.0005, 2.5 
                    //	};         
       }
     }
     inputString = "";  
   }
}

