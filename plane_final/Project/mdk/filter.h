#ifndef __FILTER_H
#define __FILTER_H

#include "zf_common_typedef.h"
#include "config.h"

extern int L1_filt, L2_filt, L3_filt, L4_filt;

void Filter_Init(void);
void Filter_Process(void);

#endif