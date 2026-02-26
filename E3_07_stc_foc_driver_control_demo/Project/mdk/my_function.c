#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#include "my_function.h"



void PID_update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->target - p->actual;
	
	if(p->Ki!=0){
		p->ErrorInt += p->Error0;
	}
	else{
		p->ErrorInt = 0;
	}
	p->out =  p->Kp*p->Error0 + p->Ki*p->ErrorInt + p->Kp*(p->Error0 - p->Error1);
	
	if(p->out > p->OutMax) {p->out = p->OutMax;}
	if(p->out < p->OutMin) {p->out = p->OutMin;}
}

void system_init()
{
tim0_irq_handler=PID_calculate;
}




