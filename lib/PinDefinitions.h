// ------------------------------------------------------
// pin_definitions.h
// Patrick Kennedy
//
// Pin definitions for Acacia Racing AVR CAN Board Rev A
// ------------------------------------------------------

// LED Outputs
#define LED_PORT    PORTD
#define LED_PIN     PIND
#define LED_DDR     DDRD
#define LED0        PD2     // Shared with D0

// Digital Input 0
#define D0          PD2
#define D0_PORT     PORTD
#define D0_PIN      PIND
#define D0_DDR      DDRD

// Analog Input 0
#define A0         	PC0                    
#define A0_PORT     PORTC
#define A0_PIN      PINC
#define A0_DDR      DDRC

// Analog Input 1
#define A1          PC1    
#define A1_PORT     PORTC
#define A1_PIN      INC
#define A1_DDR      DDRC

// Analog Input 2
#define A2          PC2 
#define A2_PORT     PORTC
#define A2_PIN      PINC
#define A2_DDR      DDRC

// Analog Input 3
#define A3          PC3 
#define A3_PORT     PORTC
#define A3_PIN      PINC
#define A3_DDR      DDRC

//  SPI Definitions
#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB

#define SPI_SS          PB0
#define SPI_CAN_SS      PB2
#define SPI_MOSI        PB3
#define SPI_MISO        PB4
#define SPI_SCK         PB5

// I2C (Two wire) Definitions
#define I2C_SDA         PC4
#define I2C_SDA_PORT    PORTC
#define I2C_SDA_PIN     PINC
#define I2C_SDA_DDR     DDRC

#define I2C_SCL         PC5
#define I2C_SCL_PORT    PORTC
#define I2C_SCL_PIN     PINC
#define I2C_SCL_DDR    	DDRC