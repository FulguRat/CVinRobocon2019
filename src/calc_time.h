#ifndef CALC_TIME_H_
#define CALC_TIME_H_

#include <iostream>
#include <Windows.h>

class CalcTime    
{  
public:
    CalcTime();
    virtual ~CalcTime();
	double End();
	bool Begin();
	bool Avaliable();
	
private:  
	int Initialized;  
	__int64 Frequency;  
	__int64 BeginTime;  

};  

#endif //_COMPUTE_TIME_H