#ifndef _USERTIMER_H_
#define _USERTIMER_H_

#include "stm32g0xx.h"
#include "stm32g0xx_ll_tim.h"

typedef struct
{
	uint64_t microscnt;
    TIM_TypeDef *TIMx;

    uint64_t (*micros)(void);
}timer_t;

extern void initTicks(TIM_TypeDef *_TIMx);
extern void sysTicksTimerIRQ(void);
extern timer_t sys_ticks;

#endif
