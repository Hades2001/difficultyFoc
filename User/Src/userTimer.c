#include "userTimer.h"

timer_t sys_ticks;

uint64_t _micros(void)
{
	return sys_ticks.microscnt + sys_ticks.TIMx->CNT;
}

void initTicks(TIM_TypeDef *_TIMx)
{
    sys_ticks.TIMx = _TIMx;
    sys_ticks.micros = &_micros;
}

void sysTicksTimerIRQ(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(sys_ticks.TIMx))
	{
		LL_TIM_ClearFlag_UPDATE(sys_ticks.TIMx);	
	    sys_ticks.microscnt += 1000;
	}
}



