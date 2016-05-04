// 
// 
// 

#include "PIDcontroller.h"

PIDcontroller::PIDcontroller(float* Atransinvtemp, float* Arotinvtemp, int motor_n)//motorN*3, motorN*3, motorN
{
	Atransinv = new float[motor_n * 3];
	Arotinv = new float[motor_n * 3];
	memcpy(Atransinv,Atransinvtemp,motor_n * 3);
	memcpy(Arotinv, Arotinvtemp, motor_n * 3);
	delete[] Atransinvtemp;
	delete[] Arotinvtemp;

	Erot=new float[motor_n];
	Etrans = new float[motor_n];
	Erot1 = new float[motor_n];
	Etrans1 = new float[motor_n];
	Eroti = new float[motor_n];
	Etransi = new float[motor_n];
	for (int i = 0; i < motor_n; i++)
	{
		Erot[i] = 0.0f;
		Etrans[i] = 0.0f;
		Erot1[i] = 0.0f;
		Etrans1[i] = 0.0f;
		Eroti[i] = 0.0f;
		Etransi[i] = 0.0f;
	}

	motorN = motor_n;
	pidrot = new float[motorN * 3];
	pidtrans = new float[motorN * 3];
}

void PIDcontroller::SetPID(float* pidrottemp, float* pidtranstemp)//motorN*3
{
	for (int i = 0; i < motorN; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			pidrot[i * 3 + j] = pidrottemp[i * 3 + j];
			pidtrans[i * 3 + j] = pidtranstemp[i * 3 + j];
		}
	}
}

void PIDcontroller::controller(float* eta_target, float* eta_n, float T, float* Ttotal)//6*1, 6*1, _, motorN*1 
{
	//
	for (int i = 0; i < motorN; i++)
	{
		Erot1[i] = Erot[i];
		Etrans1[i] = Etrans[i];
	}
	//Rcmd
	float Ris[3 * 3];
	float Rgoal[3 * 3];
	float Rcmd[3 * 3];
	eta2R(eta_n, (float*)Ris);
	eta2R(eta_target, Rgoal);
	Matrix.Invert((float*)Ris, 3);
	Matrix.Multiply((float*)Rgoal, (float*)Ris, 3, 3, 3, (float*)Rcmd);

	// axis r and angle w of rotation
	bool flag = true;
	float r[3];
	float w;
	w = acos((trace(Rcmd) - 1) / 2);
	if (w == 0)
		flag = false;
	else
	{
		r[0] = (Rcmd[2 * 3 + 1] - Rcmd[1 * 3 + 2]) / (2 * sin(w));
		r[1] = (Rcmd[0 * 3 + 2] - Rcmd[2 * 3 + 0]) / (2 * sin(w));
		r[2] = (Rcmd[1 * 3 + 0] - Rcmd[0 * 3 + 1]) / (2 * sin(w));
	}

	//Erot
	if (flag)
	{
		Matrix.Scale((float*)r, 3, 1, w);
		Matrix.Multiply((float*)Arotinv, (float*)r, motorN, 3, 1, (float*)Erot);
	}
	else
		Matrix.Copy(Erot1,1,motorN,Erot);

	//Etrans
	float temp[3] = { 0, 0, eta_target[2]-eta_n[2]};
	Matrix.Multiply((float*)Atransinv, (float*)temp, motorN, 3, 1, (float*)Etrans);

	//Eroti=Eroti+Erot,Etransi=Etransi+Etrans (motorN*1)
	for (int i = 0; i < motorN; i++)
		Eroti[i] = Eroti[i] + Erot[i];
	for (int i = 0; i < motorN; i++)
		Etransi[i] = Etransi[i] + Etrans[i];

	//clear intergration saturation
	for (int i = 0; i < motorN; i++)
	{
		if (Eroti[i]>10) Eroti[i] = 0;
		if (Etransi[i]>100) Etransi[i] = 0;
	}

	for (int i = 0; i < motorN; i++)
	{
		Ttotal[i] = (pidrot[i * 3 + 0] * Erot[i] + pidrot[i * 3 + 1] * Eroti[i] + pidrot[i * 3 + 2] * (Erot[i] - Erot1[i]) / T
					+ pidtrans[i * 3 + 0] * Etrans[i] + pidtrans[i * 3 + 1] * Etransi[i] + pidtrans[i * 3 + 2] * (Etrans[i] - Etrans1[i]) / T);
		if (Ttotal[i]>0.2) Ttotal[i] = 0.2; 
		if (Ttotal[i] < -0.2) Ttotal[i] = -0.2;
	}
}

void PIDcontroller::printTtotal(float* Ttotal, int accuracy)
{
	String outputstring = "Ttotal: ";
	for (int i = 0; i < motorN; i++)
	{
		int Temp = Ttotal[i] * pow(10, accuracy);
		for (int j = accuracy; j >= 0; j--)
		{
			outputstring += (char)('0' + ceil(Temp / (int)pow(10, j)));
			Temp = Temp % ((int)pow(10, j));
		}
		outputstring += '_';
	}
	Serial.println(outputstring);
	delay(15);
}

PIDcontroller::~PIDcontroller()
{
	delete[] Atransinv;
	delete[] Arotinv;
	delete[] Erot;
	delete[] Etrans;
	delete[] Erot1;
	delete[] Etrans1;
	delete[] Eroti;
	delete[] Etransi;
	delete[] pidrot;
	delete[] pidtrans;
}

void eta2R(float* eta, float* R)
{
	float roll = eta[3];
	float pitch = eta[4];
	float yaw = eta[5];
	R[0 * 3 + 0] = cos(yaw)*cos(pitch);
	R[0 * 3 + 1] = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
	R[0 * 3 + 2] = sin(yaw)*sin(roll) + cos(yaw)*cos(roll)*sin(pitch);
	R[1 * 3 + 0] = sin(yaw)*cos(pitch);
	R[1 * 3 + 1] = cos(yaw)*cos(roll) + sin(roll)*sin(pitch)*sin(yaw);
	R[1 * 3 + 2] = sin(pitch)*sin(yaw)*cos(roll) - cos(yaw)*sin(roll);
	R[2 * 3 + 0] = -sin(pitch);
	R[2 * 3 + 1] = cos(pitch)*sin(roll);
	R[2 * 3 + 2] = cos(pitch)*cos(roll);
}

float trace(float* Rcmd)
{
	float w=0;
	for (int i = 0; i < 3; i++)
		w = w + Rcmd[i * 3 + i];
	return w;
}