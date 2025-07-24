#include "sys.h"

void WFI_SET(void)
{
	__ASM volatile("wfi");
}
//关闭所有中断
void INTX_DISABLE(void)
{
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");
}
void __attribute__((naked)) MSR_MSP(uint32_t addr)
{
	__asm("MSR MSP, r0");
	__asm("BX lr");
}














