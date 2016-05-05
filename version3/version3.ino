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

String inputString="";         // a string to hold incoming data
int stringComplete = 0;  // whether the string is complete

float Ttoatal[motor_N];
float eta_target[6] = { 0, 0, 0, 0, 0, 0 };
float eta_n[6]={0,0,0,0,0,0};
float T = 0.1;

uint32_t timer1=0;
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
   myPID.controller((float*)eta_target, (float*)eta_n, T, (float*)Ttoatal);
   Matrix.Print(Ttoatal, 1, 4, "Ttoatl:");   

  while(millis()-timer1<=100){delay(1);}
  Serial.println(roll);
  Serial.println(pitch);
  Serial.println(yaw);
  Serial.println(millis()-timer1);
  timer1=millis();
}
