#include "calc_time.h"

CalcTime::CalcTime()  
{  
	Initialized = QueryPerformanceFrequency((LARGE_INTEGER*)&Frequency);  
}  
   
CalcTime::~CalcTime()  
{  
		    
}  
   
bool CalcTime::Begin()  
{  
	if (!Initialized) { return 0; }

	return QueryPerformanceCounter((LARGE_INTEGER*)&BeginTime);  
 }
     
double CalcTime::End()
{  
	if (!Initialized) { return 0; }
    
	__int64 endtime;  
	 
	QueryPerformanceCounter((LARGE_INTEGER*)&endtime);  
		  
	__int64 elapsed = endtime - BeginTime;  
	  
	return ((double)elapsed / (double)Frequency) * 1000.0f;  //单位毫秒
}  

bool CalcTime::Avaliable()
{  
	return Initialized;  
}   
