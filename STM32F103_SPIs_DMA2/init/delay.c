#include "delay.h"

int count = 0;

void delay_us(int us)
{
    if (us > US_MAX_VALUE || us == 0)
    {
        return;
    }
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // разрешить прерывания по достижении 0
    // SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // запретить прерывания по достижении 0

    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // ставим тактирование от процессора
    SysTick->LOAD = (US * us - 1);               // устанавливаем в регистр число от которого считать
    SysTick->VAL = 0;                            // обнуляем текущее значение регистра SYST_CVR
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    // запускаем счетчик

    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) // ждем установку флага COUNFLAG в регистре SYST_CSR
    {
    }

    SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk; // скидываем бит COUNTFLAG
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;    // выключаем счетчик
}

void delay_ms(int ms)
{
    for (int i = 0; i < ms; i++)
    {
        delay_us(1000);
    }
}

void delay_s(int s)
{
    for (int i = 0; i < s; i++)
    {
        delay_ms(1000);
    }
}

void SysTick_Handler(void)
{
    // count++;
}
