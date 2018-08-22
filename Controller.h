#ifndef CONTROLLER_H

#define CONTROLLER_H
class Controller
{
	static void shiftVector(double *vec);
public:

	Controller(float Kp, float Ki, float Kd,float ts,float lowerbound,float upperbound);
	float update(float PV);
	void reset();
	void setSP(float);
	double getError();
	double getValue();
	void setBounds(double lower, double upper);
	// TODO: put this elsewhere

private:
	double gains[3];
	double error[3];
	double out[2];

	float _SP;

	double lower;
	double upper;

};


#endif
