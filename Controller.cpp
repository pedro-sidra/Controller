#include "Controller.h"
#include <Arduino.h>

Controller::Controller(float Kp, float Ki, float Kd, float ts)
{
	this->_Kp = Kp;
	this->_Ki = Ki;
	this->_Kd = Kd;

  	this->_ts = ts;
  	
	this->_error = 0;
	this->_errorLast = 0;
	this->_iError = 0;
	this->_SP = 0;
}

float Controller::update(float nPV)
{
	_errorLast = _error;
	_error =_SP - nPV; 

	_dError = (_error - _errorLast)/_ts;
	_iError += _ts*(_error);
	return( _Kp*_error + _Ki*_iError + _Kd*_dError);
}

void Controller::reset()
{
	 this->_iError=0;
	 this->_errorLast= 0;
}

void Controller::setSP(float nSP)
{
	this->_SP=nSP;
	
}
double Controller::getError()
{
	return _error;
}

