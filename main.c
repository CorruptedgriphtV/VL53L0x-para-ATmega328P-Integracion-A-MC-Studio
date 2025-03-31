/*
 * Sensor_Laser_I2C_UART.c
 *
 * Created: 23/03/2025 06:31:01 p. m.
 * Author : Osmar Jassiel machuca Herrada
 */ 
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2c.h>
#include <vl53l0x.h>
#include <string.h>
#include <gpio.h>

#define VL53L0X
//#define VL53L0X_SECOND  //descomenta si hay otro sensor vl53l0x
//#define VL53L0X_THIRD  //descomenta si hay otro sensor vl53l0x

uint8_t var_r, bandera=0;
float distancia;




int main(void){

	i2c_init();
	sei();
	uint16_t ranges[3] = { 0 };
    while(1){
			#ifdef VL53L0X
				bool success = vl53l0x_init();

				while (success) {

					success = vl53l0x_read_range_single(VL53L0X_IDX_FIRST, &ranges[0]);
					distancia=ranges[0];
					#ifdef VL53L0X_SECOND
						success &= vl53l0x_read_range_single(VL53L0X_IDX_SECOND, &ranges[1]);
					#endif
					#ifdef VL53L0X_THIRD
						success &= vl53l0x_read_range_single(VL53L0X_IDX_THIRD, &ranges[2]);
					#endif
					_delay_ms(5);
				}
		#endif
		
    }
}

