// PIDcontroller.h



#ifndef _PIDCONTROLLER_h
#define _PIDCONTROLLER_h

#include "MatrixMath.h"


class PIDcontroller
{
public:
	PIDcontroller(float* Atransinvtemp, float* Arotinvtemp, int motor_n);
	void SetPID(float* pidrottemp, float* pidtranstemp);
	void controller(float* eta_target, float* eta_n, float T, float* Ttotal);
	void printTtotal(float* Ttotal, int accuracy);
	~PIDcontroller();
	int motorN;
private:
	float* Atransinv;
	float* Arotinv;
	float* Erot;
	float* Etrans;
	float* Erot1;
	float* Etrans1;
	float* Eroti;
	float* Etransi;
	float* pidrot;
	float* pidtrans;
};

void eta2R(float* eta, float* R);
float trace(float* Rcmd);
#endif

