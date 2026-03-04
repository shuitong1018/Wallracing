#ifndef __MY_FUNCTION_H
 #define __MY_FUNCTION_H
 
 typedef struct {
	 float target;
	 float actual;
	 float out;
	 
	 float Kp;
	 float Ki;
	 float Kd;
	 
	 float Error0;
	 float Error1;
	 float ErrorInt;
	 
	 float OutMax;
	 float OutMin;
 }PID_t;
	 
 
 void PID_update(PID_t *p);
 void PID_calculate();
 void system_init();
 
 
 #endif