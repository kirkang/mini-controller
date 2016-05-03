#include <PIDcontroller.h>
#include <MatrixMath.h>
#define motor_N  (4)
using namespace std;


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

//initialize pid
float Atransinvtemp[motor_N][3] =
{
	 0, 0, 0.25 ,
	 0, 0, 0.25 ,
	 0, 0, 0.25 ,
	 0, 0, 0.25 
};
float Arotinvtemp[motor_N][3] =
{
	 -10, 0, 0 ,
	 10, 0, 0 ,
	 0, -5, 0 ,
	 0, 5, 0 
};
PIDcontroller myPID((float*)Atransinvtemp, (float*)Arotinvtemp, motor_N);



void setup() {
	// initialize serial:
	Serial.begin(57600);
	// reserve 200 bytes for the inputString:
	inputString.reserve(37);
	//set pid
	float pidrottemp[motor_N][3] =
	{
		 0.15, 0.0005, 0.015 ,
		 0.15, 0.0005, 0.015 ,
		 0.30, 0.0005, 0.025 ,
		 0.30, 0.0005, 0.025 
	};
	float pidtranstemp[motor_N][3] =
	{
		 1.2, 0.0015, 0.3 ,
		 1.2, 0.0015, 0.3 ,
		 1.2, 0.0015, 0.3 ,
		 1.2, 0.0015, 0.3 
	};
	myPID.SetPID((float*)pidrottemp, (float*)pidtranstemp);
}

float Ttoatal[motor_N];
float eta_target[6] = { 0, 0, 0, 0, 0, 0 };
float eta_n[6];
float T = 0.001;
void loop() {
	// print the string when a newline arrives:
	if (stringComplete)
	{
		Serial.print(inputString);
		Serial.println();
		char buffer[100];
		inputString.toCharArray(buffer, 37);
		for (int i = 0; i < 37; i++)
		{
			Serial.print(buffer[i]);
		}
		if (buffer[36] == 0x0A)
		{
			for (int i = 0; i < 6; i++)
				eta_n[i] = pow(0.1, (buffer[i * 6 + 5] - '0'))*((buffer[i * 6 + 0] - '0')
				+ 0.1*(buffer[i * 6 + 1] - '0')
				+ 0.01*(buffer[i * 6 + 2] - '0')
				+ 0.001*(buffer[i * 6 + 3] - '0')
				+ 0.0001*(buffer[i * 6 + 4] - '0'));
			Serial.println("Successful!");
			delay(15);
		}

		// clear the string:
		inputString = "";
		stringComplete = false;

		//control
		Matrix.Print(eta_n,1,motor_N,"Eta_n: ");
		myPID.controller((float*)eta_target, (float*)eta_n, T, (float*)Ttoatal);
		//    float X[motor_N] = {1.23, 0.101, 0.001, 2.23};
		//    myPID.printTtotal((float*)Ttoatal, 4);
		//    char X[motor_N] = { 'a', 'b', 'c', 'd' };
		//    printTest((float*)X, 4);
		Matrix.Print(Ttoatal, 1, 4, "Ttoatl:");
	}
}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
*/
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			stringComplete = true;
		}
	}
}