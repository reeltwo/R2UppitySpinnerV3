////////////////////////////////////////////
// CONTROL BOARD PIN OUT
////////////////////////////////////////////
// Only change if you using a different PCB
////////////////////////////////////////////

#ifdef USE_SDCARD

///////////////////////////////////
// --- Lifter mechanism

#define PIN_LIFTER_ENCODER_A   34
#define PIN_LIFTER_ENCODER_B   35

/* Lifter Motor */
#define PIN_LIFTER_PWM1        32
#define PIN_LIFTER_PWM2        33
#define PIN_LIFTER_DIAG        36

///////////////////////////////////
// --- Rotary mechanism

#define PIN_ROTARY_ENCODER_A   27
#define PIN_ROTARY_ENCODER_B   13

/* Rotary Motor */
#define PIN_ROTARY_PWM1        25
#define PIN_ROTARY_PWM2        26
#define PIN_ROTARY_DIAG        39

///////////////////////////////////

#define PIN_STATUSLED          5
#define PIN_PPMIN_RC           14
#define PIN_RXD2               16
#define PIN_TXD2               0 /* not used */

///////////////////////////////////

#define PIN_SD_CS              4
#define PIN_SCK                18
#define PIN_MISO               19
#define PIN_MOSI               23

///////////////////////////////////

#define PIN_SDA                21
#define PIN_SCL                22

///////////////////////////////////

#define PIN_GPIO_INTERRUPT     17
#define USE_I2C_GPIO_EXPANDER
#define GPIO_PIN_BASE           200

#define PIN_ROTARY_LIMIT        GPIO_PIN_BASE+0
#define PIN_LIGHTKIT_A          GPIO_PIN_BASE+1
#define PIN_LIGHTKIT_B          GPIO_PIN_BASE+2
#define PIN_LIGHTKIT_C          GPIO_PIN_BASE+3
#define PIN_MOTOR_EN            GPIO_PIN_BASE+4
#define PIN_MOTOR_ENB           GPIO_PIN_BASE+5
#define PIN_LIFTER_BOTLIMIT     GPIO_PIN_BASE+6
#define PIN_LIFTER_TOPLIMIT     GPIO_PIN_BASE+7

#else

///////////////////////////////////
// --- Lifter mechanism

#define PIN_LIFTER_ENCODER_A   34
#define PIN_LIFTER_ENCODER_B   35

/* Lifter Motor */
#define PIN_LIFTER_PWM1        32
#define PIN_LIFTER_PWM2        33
#define PIN_LIFTER_DIAG        36

#define PIN_LIFTER_TOPLIMIT    18
#define PIN_LIFTER_BOTLIMIT    19

///////////////////////////////////
// --- Rotary mechanism

#define PIN_ROTARY_ENCODER_A   27
#define PIN_ROTARY_ENCODER_B   13

/* Rotary Motor */
#define PIN_ROTARY_PWM1        25
#define PIN_ROTARY_PWM2        26
#define PIN_ROTARY_DIAG        39

#define PIN_ROTARY_LIMIT       23

///////////////////////////////////

#define PIN_STATUSLED          5
#define PIN_PPMIN_RC           14
#define PIN_RXD2               16
#define PIN_TXD2               0 /* not used */

///////////////////////////////////

#define PIN_SDA                21
#define PIN_SCL                22

///////////////////////////////////

#define PIN_GPIO_INTERRUPT     17
#define USE_I2C_GPIO_EXPANDER
#define GPIO_PIN_BASE           200

#define PIN_INPUT_A             GPIO_PIN_BASE+0   /* INPUT A */
#define PIN_LIGHTKIT_A          GPIO_PIN_BASE+1
#define PIN_LIGHTKIT_B          GPIO_PIN_BASE+2
#define PIN_LIGHTKIT_C          GPIO_PIN_BASE+3
#define PIN_MOTOR_EN            GPIO_PIN_BASE+4
#define PIN_MOTOR_ENB           GPIO_PIN_BASE+5
#define PIN_INPUT_B             GPIO_PIN_BASE+6   /* INPUT B */
#define PIN_INPUT_C             GPIO_PIN_BASE+7   /* INPUT B */

#endif
