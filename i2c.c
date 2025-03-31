#include "i2c.h"
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

uint8_t DEFAULT_SLAVE_ADDRESS=0x29;

typedef enum
{
    ADDR_SIZE_8BIT,
    ADDR_SIZE_16BIT
} addr_size_t;

typedef enum
{
    REG_SIZE_8BIT,
    REG_SIZE_16BIT,
    REG_SIZE_32BIT
} reg_size_t;
static bool Condicion_Inicio(){
	uint8_t estado;
	// Iniciar la condición de START
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))); // Esperar a que la condición de START se complete
	estado=TWSR & 0xF8;
	if (estado == 0x08 || estado == 0x10){//0x08->Condición de inicio exitoso
		return true; //0x10-> Condición de inicio repetido exitoso
	}else{
		return false;
	}
	
}
static bool enviar_8bits(uint8_t dato){
	uint8_t estado;
	TWDR=dato;//Se prepara el dato a enviar
	TWCR = (1 << TWINT)| (1 << TWEN);//Inicia el envio
	while (!(TWCR & (1 << TWINT))); // Esperar a que se envíe el dato

	// Verificar si se recibió ACK del esclavo
	estado=TWSR & 0xF8;
	if(estado == 0x18 || estado== 0x28 || estado == 0x40){//0x18->Se transmitió Dirección + W y se recibió ACK
		return true;//0x28-> Se transmitió un dato y se recibió ACK
	}else{//0x50->Se transmitió dirección + R y se recibió ACK
		return false;
	}
}
static bool start_transfer(addr_size_t addr_size, uint16_t addr) {
    uint8_t estado;
	bool success = false;
	if(!Condicion_Inicio()){
		return false;
	}
	if(!enviar_8bits((DEFAULT_SLAVE_ADDRESS<<1|0x00))){
		return false;
	}
    switch (addr_size) {
        case ADDR_SIZE_8BIT:
			success=enviar_8bits((uint8_t)addr);//Se envia dato de 8bits
			if(!success)
				return false;
			break;
        case ADDR_SIZE_16BIT:
            // Enviar el byte más significativo de la dirección de 16 bits
            success=enviar_8bits((uint8_t)(addr >> 8));
            if (success) {
                // Enviar el byte menos significativo de la dirección de 16 bits
                success=enviar_8bits((uint8_t)(addr & 0xFF));
				if(!success)
					return false;
			}else{
				return false;
			}
            break;
    }
    return success;
}

static void stop_transfer() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & 1 << TWSTO); // Esperar a que la condición de STOP se complete

}

static bool read_reg(addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint8_t *data) {
    bool success = false;
	uint8_t estado;
    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    // Enviar la condición de REPEATED START para cambiar a modo lectura
    if(!Condicion_Inicio()){
		return false;
	}

    // Enviar la dirección del esclavo (y el bit de lectura)
    if(!enviar_8bits((DEFAULT_SLAVE_ADDRESS << 1) | 0x01)){// Dirección de 7 bits + bit de lectura
		return false;
	}else{
		success=true;
	}
		
    if (success) {
        switch (reg_size) {
            case REG_SIZE_8BIT:
                TWCR = (1 << TWINT) | (1 << TWEN)|(1<<TWEA); // ACK para recibir el byte
                while (!(TWCR & (1 << TWINT)));
				estado=TWSR & 0xF8;
				if (estado==0x58 || estado==0x50){//0x58->Dato recibido y NACK enviado,0x50->Dato recibido y ACK enviado
					data[0] = TWDR;
					TWCR = (1 << TWINT) | (1 << TWEN); // NACK para el último byte
					while (!(TWCR & (1 << TWINT)));
					stop_transfer();
					success=true;
				}else{
					success=false;
				}
                break;
            case REG_SIZE_16BIT:
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // ACK para recibir el MSB
                while (!(TWCR & (1 << TWINT)));
                data[1] = TWDR;
                TWCR = (1 << TWINT) | (1 << TWEN); // NACK para el LSB
                while (!(TWCR & (1 << TWINT)));
                data[0] = TWDR;
				stop_transfer();
                break;
            case REG_SIZE_32BIT:
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // ACK para el byte 3
                while (!(TWCR & (1 << TWINT)));
                data[3] = TWDR;
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // ACK para el byte 2
                while (!(TWCR & (1 << TWINT)));
                data[2] = TWDR;
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // ACK para el byte 1
                while (!(TWCR & (1 << TWINT)));
                data[1] = TWDR;
                TWCR = (1 << TWINT) | (1 << TWEN); // NACK para el byte 0
                while (!(TWCR & (1 << TWINT)));
                data[0] = TWDR;
				stop_transfer();
                break;
        }
        
    }
    return success;
}

static bool read_reg_bytes(addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count) {
    bool success = false;
    bool transfer_stopped = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    // Enviar la condición de REPEATED START para cambiar a modo lectura
    if (!Condicion_Inicio()){
		return false;
    }

    // Enviar la dirección del esclavo (y el bit de lectura)
    if(!enviar_8bits((DEFAULT_SLAVE_ADDRESS << 1) | 0x01)){// Dirección de 7 bits + bit de lectura
	    return false;
    }else{
		success=true;
    }
    if (success) {
        for (int i = 0; i < byte_count; i++) {
            if (i + 1 == byte_count) {
                TWCR = (1 << TWINT) | (1 << TWEN); // NACK para el último byte
                while (!(TWCR & (1 << TWINT)));
                bytes[i] = TWDR;
                stop_transfer();
                transfer_stopped = true;
            } else {
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // ACK para los bytes anteriores
                while (!(TWCR & (1 << TWINT)));
                bytes[i] = TWDR;
            }
        }
    }
    if (!transfer_stopped) {
        stop_transfer();
    }
    return success;
}

bool i2c_read_addr8_data8(uint8_t addr, uint8_t *data) {
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, data);
}

bool i2c_read_addr8_data16(uint8_t addr, uint16_t *data) {
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool i2c_read_addr16_data8(uint16_t addr, uint8_t *data) {
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, data);
}

bool i2c_read_addr16_data16(uint16_t addr, uint16_t *data) {
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool i2c_read_addr8_data32(uint16_t addr, uint32_t *data) {
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool i2c_read_addr16_data32(uint16_t addr, uint32_t *data) {
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count) {
    return read_reg_bytes(ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

static bool write_reg(addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint16_t data) {
    bool success = false;
	uint8_t estado=0;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    switch (reg_size) {
        case REG_SIZE_8BIT:
			if(!enviar_8bits((uint8_t)(data & 0xFF))){
				return false;
			}else{
				success=true;
			}
            break;
        case REG_SIZE_16BIT:
            success=enviar_8bits((uint8_t)((data >> 8) & 0xFF));
            if (success) {
				success=enviar_8bits((uint8_t)(data & 0xFF));
				if(!success)
					return false;
            }else{
				return false;
            }
            break;
        case REG_SIZE_32BIT:
            return false; // No soportado
    }

    stop_transfer();
    return success;
}

static bool write_reg_bytes(addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count) {
    bool success = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    for (uint16_t i = 0; i < byte_count; i++) {
        TWDR = bytes[i];
        TWCR = (1 << TWINT) | (1 << TWEN);
        while (!(TWCR & (1 << TWINT)));
        success = ((TWSR & 0xF8) == TW_MT_DATA_ACK);
        if (!success) {
            break;
        }
    }

    stop_transfer();
    return success;
}

bool i2c_write_addr8_data8(uint8_t addr, uint8_t value) {
    return write_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, value);
}

bool i2c_write_addr8_data16(uint8_t addr, uint16_t value) {
    return write_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, value);
}

bool i2c_write_addr16_data8(uint16_t addr, uint8_t value) {
    return write_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, value);
}

bool i2c_write_addr16_data16(uint16_t addr, uint16_t value) {
    return write_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, value);
}

bool i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count) {
    return write_reg_bytes(ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

void i2c_set_slave_address(uint8_t addr) {
	DEFAULT_SLAVE_ADDRESS= addr;
    // No es necesario en ATMega328P, ya que la dirección se pasa directamente a las funciones de lectura/escritura.
}

void i2c_init() {
    // Configurar los pines SDA y SCL como entradas con resistencias pull-up
    //DDRC &= ~((1 << PC4) | (1 << PC5)); // PC4 (SDA), PC5 (SCL) como entradas
    //PORTC |= ((1 << PC4) | (1 << PC5)); // Habilitar resistencias pull-up

    // Configurar la velocidad del bus I2C (ejemplo: 100 kHz)
    TWBR = 221;// 35kHz //72; // Para 16 MHz y 100 kHz (ajusta este valor según tu frecuencia de reloj)
    TWSR =0x00; // Preescalador = 1
	TWCR=1<<TWEN;//habilita la interfaz
}
