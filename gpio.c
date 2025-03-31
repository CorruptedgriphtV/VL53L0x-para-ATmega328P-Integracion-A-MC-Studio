#include "gpio.h"
#include <avr/io.h>
#include <stdbool.h>


void gpio_init(void)
{
    /* P1.0 GPIO and output low (mapped to PB0 on ATMega328P) */
    DDRB |= (1 << PB0); // Set PB0 as output
    PORTB &= ~(1 << PB0); // Set PB0 low

    /* P1.1 GPIO and output low (mapped to PB1 on ATMega328P) */
    DDRB |= (1 << PB1); // Set PB1 as output
    PORTB &= ~(1 << PB1); // Set PB1 low

    /* P1.2 GPIO and output low (mapped to PB2 on ATMega328P) */
    DDRB |= (1 << PB2); // Set PB2 as output
    PORTB &= ~(1 << PB2); // Set PB2 low
}

void gpio_set_output(gpio_t gpio, bool enable)
{
    uint8_t bit = 0;
    switch (gpio)
    {
    case GPIO_XSHUT_FIRST:
        bit = PB1;
        break;
    case GPIO_XSHUT_SECOND:
        bit = PB0;
        break;
    case GPIO_XSHUT_THIRD:
        bit = PB2;
        break;
    }

    if (enable)
    {
        PORTB |= (1 << bit); // Set the pin high
    }
    else
    {
        PORTB &= ~(1 << bit); // Set the pin low
    }
}
