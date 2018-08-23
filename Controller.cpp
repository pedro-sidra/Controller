#include "Controller.h"
#include <Arduino.h>

void Controller::shiftVector(double *vec,char size)
{
	for(int i=size-1; i>0 ; i--)
	{
		vec[i]= vec[i-1];
	}
}

double sat(double in, double lower, double upper)
{
	if(lower>upper)
		return in;
	if(in>upper)
		return upper;
	else if(in < lower)
		return lower;
	return in;
}



Controller::Controller(float Kp, float Ki, float Kd, float ts,float lowerbound=1,float upperbound=-1)
{
	// The expression for this discrete controller was derived using Bacwards-Euler approx.
	//
	// Consider the transfer function:
	// C(s) = Kp + Ki/s + Kd*s
	// Then we substitute s = (z-1)/(z*ts), where ts is the sample time of the controller
	// We obtain the transfer function:
	//        a*z^2 + b*z + c
	// C(z) = ---------------- 
	//            z^2 - z
	//
	// where a = Ki*ts + Kp + Kd/ts
	// 	     b = -1*(Kd/ts + Kp)
	// 	     c = Kd
	// And the difference equation for C(z) = U(z)/E(z) can be written as:
	// u[n]= a*e[n]+b*e[n-1]+c*e[n-2] + u[n-1]
	// 
	// Which is how the controller is implemented here.
	// All vector indexes correspond to the delay time
	// e.g. e[n-2] = error[2]
	// TODO: implement derivative frequency limitation (if needed)
	
	gains[0]=Ki*ts+Kp+Kd/ts;
	gains[1]=-1*(2*Kd/ts + Kp);
	gains[2]=Kd;

	error[0]=0;error[1]=0;error[2]=0;
	out[0]=0;out[1]=0;

	this-> lower = lowerbound;
	this-> upper = upperbound;
}

float Controller::update(float nPV)
{
	Controller::shiftVector(error,3);
	error[0]=_SP-nPV;
	shiftVector(out,2);
	out[0]=gains[0]*error[0]+gains[1]*error[1]+gains[2]*error[2]+out[1];
	out[0] = sat(out[0],lower,upper);	
	return out[0];
}

void Controller::reset()
{
	error[0]=0;error[1]=0;error[2]=0;
	out[0]=0;out[1]=0;
}

void Controller::setSP(float nSP)
{
	this->_SP=nSP;
	
}
double Controller::getError()
{
	return error[0];
}

double Controller::getValue()
{
	return(out[0]);
}

void Controller::setBounds(double lower, double upper)
{
	this->upper = upper;
	this->lower = lower;
}
