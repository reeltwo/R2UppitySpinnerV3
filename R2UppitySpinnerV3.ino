#define USE_DEBUG
//#define USE_LEDLIB 0
#define USE_SDCARD
#define USE_SPIFFS
#define USE_OTA

#include "ReelTwo.h"
#include "core/SetupEvent.h"
#include "core/AnimatedEvent.h"
#include "analogWrite.h"
#include "encoder/PPMReader.h"
//#include "core/Enthropy.h"
#include "Wire.h"
#include "PCF8574.h"
#ifdef USE_OTA
#include <ArduinoOTA.h>
#endif

#ifdef USE_SPIFFS
#include "SPIFFS.h"
#define USE_FS SPIFFS
#endif
#include "FS.h"
#include "SPI.h"
#ifdef USE_SDCARD
#include "SD.h"
#endif

// #define DISABLE_ROTARY

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

// If debug is enabled the serial baud rate will be 57600
#define SERIAL_BAUD_RATE                    9600
#define CONSOLE_BUFFER_SIZE                 300
#define COMMAND_BUFFER_SIZE                 256
#define ROTARY_THROTTLE_ACCELERATION_SCALE  100
#define ROTARY_THROTTLE_DECELERATION_SCALE  20
#define ROTARY_THROTTLE_LATENCY             25
#define ROTARY_FUDGE_POSITION               5
#define EEPROM_SIZE                         4096

// Default random duration of 'M' movement mode
#define MOVEMODE_MIN_DURATION               30      // minimum 30 seconds
#define MOVEMODE_MAX_DURATION               30      // random range additional 30 seconds
#define MOVEMODE_MAX_INTERVAL               5       // default interval between random commands

///////////////////////////////////
// RISKIER CONFIGURATION OPTIONS
///////////////////////////////////

// IA-Parts lifter defaults
#define LIFTER_MINIMUM_POWER        30      // 40 out of 100. Don't bother with lower values. Won't lift reliably
#define LIFTER_SEEKBOTTTOM_POWER    30      // 30 out of 100. Lower than LIFTER_MINIMUM_POWER because we are going down
#define ROTARY_MINIMUM_POWER        20      // 20 out of 100. Don't bother with lower values. Won't rotate reliably
#define LIFTER_DISTANCE             845     // default value - lifter will calibrate

// Greg Hulette’s Periscope Lifter
//#define LIFTER_MINIMUM_POWER        65      // 65 out of 100. Don't bother with lower values. Won't lift reliably
//#define LIFTER_SEEKBOTTTOM_POWER    40      // 40 out of 100. Lower than LIFTER_MINIMUM_POWER because we are going down
//#define ROTARY_MINIMUM_POWER        40      // 40 out of 100. Don't bother with lower values. Won't rotate reliably
//#define LIFTER_DISTANCE             1200     // default value 1200 - lifter will calibrate

#define ROTARY_MINIMUM_HEIGHT       LIFTER_DISTANCE/2
#define MOTOR_TIMEOUT               2000
#define OUTPUT_LIMIT_PRESCALE       3.1
#define DISTANCE_OUTPUT_SCALE       3
#define MAX_COMMANDS                100

#define EEPROM_MAGIC                0xba5eba11
#define EEPROM_CMD_MAGIC            0xf005ba11
#define EEPROM_END_TAG              0xff

///////////////////////////////////
// This option is for builders working on their own custom lifters.
// It will leave the lifter logic intact but act as if there is no rotary.
///////////////////////////////////
//#define DISABLE_ROTARY
///////////////////////////////////

#ifdef DISABLE_ROTARY
// DO NOT ENABLE THIS UNLESS YOU REMOVE THE ROTARY UNIT
#define DISABLE_SAFETY_MANEUVER
#endif

///////////////////////////////////
// --- Lifter mechanism

#define PIN_LIFTER_ENCODER_A   34
#define PIN_LIFTER_ENCODER_B   35

/* Lifter Motor */
#define PIN_LIFTER_PWM1        32
#define PIN_LIFTER_PWM2        33
#define PIN_LIFTER_DIAG        36

// #define PIN_MOTOR_EN           2
// #define PIN_MOTOR_ENB          12

///////////////////////////////////
// --- Rotary mechanism

#define PIN_ROTARY_ENCODER_A   27
#define PIN_ROTARY_ENCODER_B   13

/* Rotary Motor */
#define PIN_ROTARY_PWM1        25
#define PIN_ROTARY_PWM2        26
#define PIN_ROTARY_DIAG        39

#define PIN_GPIO_INTERRUPT     17

// TODO GPIO EXPANDER
#define EPIN_ROTARY_LIMIT       0  //  1
#define EPIN_LIGHTKIT_A         1  //  2
#define EPIN_LIGHTKIT_B         2  //  4
#define EPIN_LIGHTKIT_C         3  //  8
#define EPIN_MOTOR_EN           4  // 10
#define EPIN_MOTOR_ENB          5  // 20
#define EPIN_LIFTER_BOTLIMIT    6  // 40
#define EPIN_LIFTER_TOPLIMIT    7  // 80

//////////////////////////////
// LIGHT KIT TRI-STATE
//////////////////////////////
//   A    B    C
// OPEN OPEN OPEN  (Switch Position 0): Full Cycle (default): This routine will randomly select the LED
//                                      color, pattern, and speed for a random period of time.
// OPEN OPEN GND   (Switch Position 1): Off: This setting turns ALL lights OFF. I added this to allow a
//                                      microcontroller to turn off the lights without having to kill
//                                      the supply power.
// OPEN GND  OPEN  (Switch Position 2): Obi Wan: The Top LED’s flash Blue, the Side LED’s are Blue,
//                                      and the Main White LED’s are Random.
// OPEN GND  GND   (Switch Position 3): Yoda: The Top LED’s and Side LED’s fade Green On and Off.
// GND  OPEN OPEN  (Switch Position 4): Sith: The Top LED’s and Side LED’s flash Red.
// GND  OPEN GND   (Switch Position 5): Search Light: All LED’s are White, the Center Bright LED is ON.
// GND  GND  OPEN  (Switch Position 6): Dagobah: This is the most screen accurate mode.
//                                      The Main White LED’s are ON, the side LED’s are White, the Lower 
//                                      Rectangular Red LED’s are All On, and the Rear LED’s are Blinking Red.
// GND GND GND     (Switch Position 7): Sparkle: All White LED’s randomly Flash

///////////////////////////////////

#define PIN_PPMIN_RC           14
#define PIN_RXD2               16
#define PIN_STATUSLED          5
#define PIN_SD_CS              4

///////////////////////////////////

#ifdef PIN_STATUSLED
#include "core/SingleStatusLED.h"
typedef SingleStatusLED<PIN_STATUSLED> StatusLED;
StatusLED statusLED;
#endif

///////////////////////////////////

#include "drive/TargetSteering.h"

///////////////////////////////////

#include "core/EEPROMSettings.h"

///////////////////////////////////
#define ENCODER_STATUS_RATE 200 // ms (10Hz)

struct OutputLimit
{
    bool valid;
    unsigned outputLimit;
};

struct LifterSettings
{
    OutputLimit fUpLimits[100/5+1];
    OutputLimit fDownLimits[sizeof(fUpLimits)/sizeof(fUpLimits[0])];
    unsigned fMinimumPower;
    unsigned fLifterDistance = LIFTER_DISTANCE;
    union
    {
        struct
        {
            bool fLifterLimitSetting:1;
            bool fRotaryLimitSetting:1;
            bool fUpLimitsCalibrated:1;
            bool fDownLimitsCalibrated:1;
            bool fSafetyManeuver:1;
            bool fDisableRotary:1;
        };
        uint8_t fFlags;
    };
    uint32_t fBaudRate = SERIAL_BAUD_RATE;

    static constexpr size_t limitCount()
    {
        return sizeof(fUpLimits)/sizeof(fUpLimits[0]);
    }
};
EEPROMSettings<LifterSettings> sSettings;

static bool sCalibrating;
static bool sSafetyManeuver;
static unsigned sRotaryCircleEncoderCount;

static unsigned sPos;
static bool sProcessing;
static bool sNextCommand;
static uint32_t sWaitNextSerialCommand;
static char sBuffer[CONSOLE_BUFFER_SIZE];
static bool sCmdNextCommand;
static char sCmdBuffer[COMMAND_BUFFER_SIZE];
static bool sRCMode;

static void runSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sProcessing = true;
}

static void resetSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

///////////////////////////////////

static bool sVSPIStarted;
static bool sSDCardMounted;

bool getSDCardMounted()
{
    return sSDCardMounted;
}

bool ensureVSPIStarted()
{
    if (!sVSPIStarted)
    {
        sVSPIStarted = true;
        SPI.begin();
        SPI.setFrequency(1000000);
    }
    return true;
}

bool mountReadOnlyFileSystem()
{
#ifdef USE_SPIFFS
    return (SPIFFS.begin(true));
#endif
    return false;
}

bool mountSDFileSystem()
{
#ifdef USE_SDCARD
    if (!ensureVSPIStarted())
        return false;
    if (SD.begin(PIN_SD_CS))
    {
        DEBUG_PRINTLN("Card Mount Success");
        sSDCardMounted = true;
        return true;
    }
    DEBUG_PRINTLN("Card Mount Failed");
#endif
    return false;
}

void unmountSDFileSystem()
{
#ifdef USE_SDCARD
    if (sSDCardMounted)
    {
        sSDCardMounted = false;
        SD.end();
    }
    if (sVSPIStarted)
    {
        SPI.end();
    }
#endif
}

void unmountFileSystems()
{
    unmountSDFileSystem();
#ifdef USE_FATFS
    FFat.end();
#endif
#ifdef USE_SPIFFS
    SPIFFS.end();
#endif
}

/////////////////////////////////////////////////////////////////////////

static volatile bool sDigitalReadAll;

static void IRAM_ATTR flagDigitalReadAll()
{
    sDigitalReadAll = true;
}

PCF8574 sGPIOExpander(0x20, PIN_GPIO_INTERRUPT, flagDigitalReadAll);
PPMReader sPPM(PIN_PPMIN_RC, 6);

void scan_i2c()
{
    unsigned nDevices = 0;
    for (byte address = 1; address < 127; address++)
    {
        String name = "<unknown>";
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (address == 0x70)
        {
            // All call address for PCA9685
            name = "PCA9685:all";
        }
        if (address == 0x40)
        {
            // Adafruit PCA9685
            name = "PCA9685";
        }
        if (address == 0x20)
        {
            name = "GPIO Expander";
        }
        if (address == 0x16)
        {
            // PSIPro
            name = "PSIPro";
        }
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.print(" ");
            Serial.println(name);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

bool getExpanderPin(uint8_t pin)
{
    return sGPIOExpander.digitalRead(pin, true);
}

static void setExpanderPin(uint8_t pinNumber, uint8_t val)
{
    sGPIOExpander.digitalWrite(pinNumber, val);
}

///////////////////////////////////

class PeriscopeLifter : public SetupEvent
{
public:
    enum
    {
        kLightKit_FullCycle = 0,
        kLightKit_Off = 1,
        kLightKit_ObiWan = 2,
        kLightKit_Yoda = 3,
        kLightKit_Sith = 4,
        kLightKit_SearchLight = 5,
        kLightKit_Dagobah = 6,
        kLightKit_Sparkle = 7
    };

    static void println()
    {
        Serial.println();
    }

    static void print(char num)
    {
        Serial.print(num);
    }

    static void print(int num)
    {
        Serial.print(num);
    }

    static void print(long num)
    {
        Serial.print(num);
    }

    static void print(unsigned int num)
    {
        Serial.print(num);
    }

    static void print(float num)
    {
        Serial.print(num);
    }

    static void print(const char* str)
    {
        Serial.print(str);
    }

    static void print(const __FlashStringHelper* str)
    {
        Serial.print(str);
    }

    static void println(char num)
    {
        Serial.println(num);
    }

    static void println(int num)
    {
        Serial.println(num);
    }

    static void println(long num)
    {
        Serial.println(num);
    }

    static void println(unsigned int num)
    {
        Serial.println(num);
    }

    static void println(float num)
    {
        Serial.println(num);
    }

    static void println(const char* str)
    {
        Serial.println(str);
    }

    static void println(const __FlashStringHelper* str)
    {
        Serial.println(str);
    }

    ///////////////////////////////////
    // Read limit switches
    ///////////////////////////////////

    static bool lifterTopLimit()
    {
        bool limit = (getExpanderPin(EPIN_LIFTER_TOPLIMIT) == sSettings.fLifterLimitSetting);
        return limit;
    }

    static bool lifterBottomLimit()
    {
        bool limit = (getExpanderPin(EPIN_LIFTER_BOTLIMIT) == sSettings.fLifterLimitSetting);
        return limit;
    }

    static long getLifterPosition()
    {
        long pos;
        cli();
        pos = encoder_lifter_ticks;
        sei();
        return pos;
    }

    ///////////////////////////////////

    static bool rotaryHomeLimit()
    {
    #ifdef DISABLE_ROTARY
        // No rotary unit. Always in home position
        return true;
    #else
        bool limit = sSettings.fDisableRotary || (getExpanderPin(EPIN_ROTARY_LIMIT) == sSettings.fRotaryLimitSetting);
        return limit;
    #endif
    }

    static long getRotaryPosition()
    {
        long pos;
        cli();
        pos = encoder_rotary_ticks;
        sei();
        return pos;
    }

    ///////////////////////////////////

    static bool lifterMotorFault()
    {
        return !digitalRead(PIN_LIFTER_DIAG);
    }

    static bool rotaryMotorFault()
    {
        return !digitalRead(PIN_ROTARY_DIAG);
    }

    static bool isIdle()
    {
        if (lifterBottomLimit())
        {
            if (!fMotorsEnabled ||
                fMotorsEnabledTime + MOTOR_TIMEOUT < millis())
            {
                return true;
            }
        }
        return false;
    }

    ///////////////////////////////////

    static bool motorsEnabled()
    {
        return fMotorsEnabled;
    }

    static void disableMotors()
    {
        setExpanderPin(EPIN_MOTOR_EN, LOW);
        setExpanderPin(EPIN_MOTOR_ENB, HIGH);

        // digitalWrite(PIN_MOTOR_EN, LOW);
        // digitalWrite(PIN_MOTOR_ENB, HIGH);
        fMotorsEnabled = false;
    }

    static void enableMotors()
    {
        // TODO ENABLE MOTORS
        setExpanderPin(EPIN_MOTOR_EN, HIGH);
        setExpanderPin(EPIN_MOTOR_ENB, LOW);

        // digitalWrite(PIN_MOTOR_EN, HIGH);
        // digitalWrite(PIN_MOTOR_ENB, LOW);
        fMotorsEnabled = true;
        fMotorsEnabledTime = millis();
    }

    static inline void dualAnalogWrite(uint8_t m1, float v1, uint8_t m2, float v2)
    {
        const auto output1{static_cast<int32_t>(abs(v1) * 255)};
        const auto output2{static_cast<int32_t>(abs(v2) * 255)};
        // if (m1 == kPWM1_Timer0 && m2 == kPWM2_Timer0)
        // {
        //     OCR0A = output1;
        //     OCR0B = output2;
        // }
        // else if (m1 == kPWM1_Timer1 && m2 == kPWM2_Timer1)
        // {
        //     const auto output1{static_cast<uint16_t>(abs(v1) * 799)};
        //     const auto output2{static_cast<uint16_t>(abs(v2) * 799)};
        //     OCR1A = output1;
        //     OCR1B = output2;
        // }
        // else if (m1 == kPWM1_Timer2 && m2 == kPWM2_Timer2)
        // {
        //     OCR2A = output1;
        //     OCR2B = output2;
        // }
        // else
        {
            // Serial.print("dualAnalogWrite "); Serial.print(output1); Serial.print(", "); Serial.println(output2);
            // TODO IMPLEMENT FOR ESP32
            ::analogWrite(m1, output1);
            ::analogWrite(m2, output2);
        }
    }

    ///////////////////////////////////
    // Move lifter motor up/down
    ///////////////////////////////////

    static void lifterMotorMove(float throttle)
    {
        bool reverse = (throttle < 0);
        throttle = min(max(abs(throttle), 0.0f), 1.0f);

        if (throttle < 0.10)
            throttle = 0;
        if (fLifterThrottle != throttle)
        {
            enableMotors();
            if (reverse)
            {
                dualAnalogWrite(PIN_LIFTER_PWM1, 0, PIN_LIFTER_PWM2, fabs(throttle));
                fLifterThrottle = -throttle;
            }
            else
            {
                dualAnalogWrite(PIN_LIFTER_PWM1, fabs(throttle), PIN_LIFTER_PWM2, 0);
                fLifterThrottle = throttle;
            }
        }
    }

    static void lifterMotorStop()
    {
        enableMotors();
        dualAnalogWrite(PIN_LIFTER_PWM1, 0, PIN_LIFTER_PWM2, 0);
        fLifterThrottle = 0;
    }

    ///////////////////////////////////

    static bool serialAbort()
    {
        if (sCalibrating && Serial.available())
        {
            DEBUG_PRINTLN("SERIAL ABORT");
            while (Serial.available())
                Serial.read();
            sCalibrating = false;
            return true;
        }
        return false;
    }

    static bool seekToPosition(float pos, float speed)
    {
        if (!ensureSafetyManeuver())
            return false;

        // ensure position is in the range of 0.0 [bottom] - 1.0 [top]
        pos = min(max(abs(pos), 0.0f), 1.0f);
        if (isRotarySpinning() || !rotaryHomeLimit())
        {
            // Cannot go below 50% if spinning or not at home position
            pos = min(max(pos, 0.5f), 1.0f);
        }
        if (speed * 100 < sSettings.fMinimumPower)
            return seekToPositionSlow(pos, speed/(sSettings.fMinimumPower/100.0));

        long maxlen = sSettings.fLifterDistance;
        long current = getLifterPosition();
        long target_ticks = pos * maxlen;

        long distance = abs(target_ticks - current);
        TargetSteering steering(target_ticks);
        steering.setSampleTime(1);
        float minpower = (1.0f - ((float)distance / (float)maxlen)) + 0.1;
        steering.setDistanceTunings(1.0, minpower, minpower);

        float limit;
        bool success = false;
        if (target_ticks > getLifterPosition())
        {
            // seek up
            if (!getUpOutputLimit(speed, limit))
                return false;
            steering.setDistanceOutputLimits(min(float(distance * DISTANCE_OUTPUT_SCALE), limit));
            bool topLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                topLimit = lifterTopLimit();
                if (topLimit || encoder_ticks == target_ticks || serialAbort())
                    break;
                steering.setCurrentDistance(encoder_ticks);
                lifterMotorMove(steering.getThrottle() * speed);

                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
            }
            success = topLimit;
        }
        else
        {
            // seek down
            if (!getDownOutputLimit(speed, limit))
                return false;
            steering.setDistanceOutputLimits(min(float(distance * DISTANCE_OUTPUT_SCALE), limit));
            bool botLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                botLimit = lifterBottomLimit();
                if (botLimit || encoder_ticks == target_ticks || serialAbort())
                    break;
                steering.setCurrentDistance(encoder_ticks);
                lifterMotorMove(steering.getThrottle() * speed);

                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
            }
            success = botLimit;
        }
        lifterMotorStop();
        return success;
    }

    ///////////////////////////////////
    // Spin rotary motor left/right
    ///////////////////////////////////

    static bool rotaryAllowed()
    {
    #ifdef DISABLE_ROTARY
        // Rotary motion not allowed
        return false;
    #else
        return !sSettings.fDisableRotary && (getLifterPosition() > ROTARY_MINIMUM_HEIGHT);
    #endif
    }

    static void rotaryMotorSpeed(float speed)
    {
        fRotarySpeed = (rotaryAllowed()) ? speed : 0;
        fRotaryEncoderLastStatus = millis();
        rotaryMotorUpdate();
    }

    static void rotaryMotorUpdate()
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        uint32_t currentMillis = millis();
        if (currentMillis - fRotaryThrottleUpdate > ROTARY_THROTTLE_LATENCY)
        {
            if (rotaryAllowed() && (fRotarySpeed != 0 || fRotaryThrottle != 0))
            {
                float throttle = 0;
                long encoder_ticks = getRotaryPosition();
                if (fRotarySpeed > fRotaryThrottle)
                {
                    float scale = ROTARY_THROTTLE_ACCELERATION_SCALE;
                    if (fRotaryThrottle < 0)
                        scale = ROTARY_THROTTLE_DECELERATION_SCALE;
                    float val = max(abs(fRotarySpeed - fRotaryThrottle) / scale, 0.01f);
                    throttle = ((int)round(min(fRotaryThrottle + val, fRotarySpeed)*100))/100.0f;
                    DEBUG_PRINTLN(throttle);
                    rotaryMotorMove(throttle);
                    fRotaryThrottle = throttle;
                }
                else if (fRotarySpeed < fRotaryThrottle)
                {
                    float scale = ROTARY_THROTTLE_ACCELERATION_SCALE;
                    if (fRotaryThrottle > 0)
                        scale = ROTARY_THROTTLE_DECELERATION_SCALE;
                    float val = abs(fRotarySpeed - fRotaryThrottle) / scale;
                    throttle = ((int)floor(max(fRotaryThrottle - val, fRotarySpeed)*100))/100.0f;
                    DEBUG_PRINTLN(throttle);
                    rotaryMotorMove(throttle);
                    fRotaryThrottle = throttle;
                }
                if (millis() - fRotaryEncoderLastStatus > ENCODER_STATUS_RATE*2)
                {
                    fRotaryEncoderLastStatus = millis();
                    if (fRotaryEncoderLastTick == encoder_ticks)
                    {
                        DEBUG_PRINTLN("ROTARY NOT MOVING - ABORT");
                        rotaryMotorStop();
                    }
                    fRotaryEncoderLastTick = encoder_ticks;
                }
                fRotaryThrottleUpdate = currentMillis;
            }
        }
    #endif
    }

    static void rotaryMotorMove(float throttle)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        bool reverse = (throttle < 0);
        throttle = min(max(abs(throttle), 0.0f), 1.0f);
        if (throttle < 0.10)
            throttle = 0;

        // Ensure lifter is higher than minimum
        if (rotaryAllowed())
        {
            if (fRotaryThrottle != throttle)
            {
                fRotaryEncoderLastStatus = millis();
                fRotaryEncoderLastTick = getRotaryPosition();
                fRotaryMoving = (throttle != 0);
                enableMotors();
                if (reverse)
                {
                    dualAnalogWrite(PIN_ROTARY_PWM1, 0, PIN_ROTARY_PWM2, fabs(throttle));
                    fRotaryThrottle = -throttle;
                }
                else
                {
                    dualAnalogWrite(PIN_ROTARY_PWM1, fabs(throttle), PIN_ROTARY_PWM2, 0);
                    fRotaryThrottle = throttle;
                }
            }
        }
        else
        {
            DEBUG_PRINT("ROTARY NOT ALLOWED: ");
            DEBUG_PRINTLN(getLifterPosition());
        }
    #endif
    }

    static bool withinArc(double p1, double p2, double p3)
    {
        return fmod(p2 - p1 + 2*360, 360) >= fmod(p3 - p1 + 2*360, 360);
    }

    static int normalize(int degrees)
    {
        degrees = fmod(degrees, 360);
        if (degrees < 0)
            degrees += 360;
        return degrees;
    }

    static int shortestDistance(int origin, int target)
    {
        int result = 0.0;
        int diff = fmod(fmod(abs(origin - target), 360), 360);

        if (diff > 180)
        {
            //There is a shorter path in opposite direction
            result = (360 - diff);
            if (target > origin)
                result *= -1;
        }
        else
        {
            result = diff;
            if (origin > target)
                result *= -1;
        }
        return result;
    }

    static bool moveScopeToTarget(int pos, int target, int fudge, float speed, float maxspeed, float &m)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return true;
        DEBUG_PRINT("MOVE raw="); DEBUG_PRINT(getRotaryPosition());
        DEBUG_PRINT(" pos="); DEBUG_PRINT(pos);
        DEBUG_PRINT(" target="); DEBUG_PRINT(target);
        if (!withinArc(target - fudge, target + fudge, pos))
        {
            int dist = shortestDistance(pos, target);
            DEBUG_PRINT(" dist="); DEBUG_PRINT(dist);
            if (maxspeed > speed && abs(dist) > fudge*2)
                speed += (maxspeed - speed) * float(abs(dist)) / 180;
            DEBUG_PRINT(" speed1="); DEBUG_PRINT(speed);
            if (speed < (ROTARY_MINIMUM_POWER/100.0))
                speed = (ROTARY_MINIMUM_POWER/100.0);
            DEBUG_PRINT(" speed2="); DEBUG_PRINT(speed);
            float nm = (dist > 0) ? -speed : speed;
            if (m != 0 && ((m < 0 && nm > 0) || (m > 0 && nm < 0)))
            {
                // Safety check: In case of problems with the rotary encoder. If our
                // direction changes from the initial direction we will just stop.
                DEBUG_PRINTLN("DIRECTION CHANGE");
                DEBUG_PRINTLN(m);
                DEBUG_PRINTLN(nm);
                return true;
            }
            m = nm;
            DEBUG_PRINT(" m="); DEBUG_PRINTLN(m);
            return false;
        }
        DEBUG_PRINTLN();
    #endif
        return true;
    }

    static unsigned rotaryMotorCurrentPosition()
    {
        return normalize(getRotaryPosition() * (360.0 / sRotaryCircleEncoderCount));
    }

    static void rotaryMotorAbsolutePosition(int degrees, float speed = 0, float maxspeed = 0)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        float m = 0;
        if (speed == 0)
            speed = (ROTARY_MINIMUM_POWER/100.0);
        if (maxspeed == 0)
            maxspeed = speed;
        // degrees = -degrees;
        RotaryStatus rotaryStatus;
        while (!moveScopeToTarget(rotaryMotorCurrentPosition(), normalize(degrees), ROTARY_FUDGE_POSITION, speed, maxspeed, m))
        {
            rotaryMotorMove(m);
            if (!rotaryStatus.isMoving())
            {
                DEBUG_PRINTLN("ABORT");
                break;
            }
        }
        rotaryMotorStop();
    #endif
    }

    static void rotaryMotorRelativePosition(int relativeDegrees)
    {
        rotaryMotorAbsolutePosition(rotaryMotorCurrentPosition() + relativeDegrees);
    }

    static void rotateHome()
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        if (shortestDistance(rotaryMotorCurrentPosition(), 0) > 0)
        {
            rotateLeftHome();
        }
        else
        {
            rotateRightHome();
        }
    #endif
    }

    static void rotateUntilHome(float speed)
    {
    #ifndef DISABLE_ROTARY
        if (sSettings.fDisableRotary)
            return;
        bool neg = (speed < 0);
        speed = (ROTARY_MINIMUM_POWER/100.0) + 0.1 * abs(speed);
        if (neg)
            speed = -speed;
        DEBUG_PRINTLN(speed);
        RotaryStatus rotaryStatus;
        // tells rotary encoder interrupt routine to stop the motor
        // when it hits the limit switch
        encoder_rotary_stop_limit = true;
        while (!rotaryHomeLimit())
        {
            rotaryMotorMove(speed);
            delay(3);
            rotaryMotorStop();
            delay(1);
            if (!rotaryStatus.isMoving())
            {
                DEBUG_PRINTLN("ABORT");
                break;
            }
        }
        rotaryMotorStop();
        encoder_rotary_stop_limit = false;
    #endif
    }

    static void rotateLeftHome()
    {
    #ifndef DISABLE_ROTARY
        // Ensure lifter is higher than minimum
        if (rotaryAllowed())
        {
            if (!rotaryHomeLimit())
            {
                rotateUntilHome(-0.1);
            }
            delay(200);
            if (!rotaryHomeLimit())
            {
                rotateUntilHome(0.1);
            }
            if (rotaryHomeLimit())
            {
                // DEBUG_PRINTLN("FOUND HOME");
                resetRotaryPosition();
            }
        }
    #endif
    }

    static void rotateRightHome()
    {
    #ifndef DISABLE_ROTARY
        // Ensure lifter is higher than minimum
        if (rotaryAllowed())
        {
            if (!rotaryHomeLimit())
            {
                rotateUntilHome(0.1);
            }
            delay(200);
            if (!rotaryHomeLimit())
            {
                rotateUntilHome(-0.1);
            }
            if (!rotaryHomeLimit())
            {
                rotateUntilHome(-0.1);
            }
            if (rotaryHomeLimit())
            {
                DEBUG_PRINTLN("FOUND HOME");
                resetRotaryPosition();
            }
            else
            {
                DEBUG_PRINTLN("NOT AT HOME");
            }
        }
    #endif
    }

    static void rotaryMotorStop()
    {
        fRotarySpeed = 0;
        enableMotors();
        dualAnalogWrite(PIN_ROTARY_PWM1, 0, PIN_ROTARY_PWM2, 0);
        fRotaryThrottle = 0;
        fRotaryMoving = false;
    }

    static bool isRotarySpinning()
    {
    #ifdef DISABLE_ROTARY
        // Rotary is not moving
        return false;
    #else
        return !sSettings.fDisableRotary && fRotaryMoving;
    #endif
    }

    static bool isRotaryAtRest()
    {
    #ifdef DISABLE_ROTARY
        return true;
    #else
        return sSettings.fDisableRotary || ((rotaryHomeLimit() || rotaryMotorCurrentPosition() == 0) && !fRotaryMoving);
    #endif
    }

    ///////////////////////////////////

    ///////////////////////////////////
    // Select show 0-7
    ///////////////////////////////////
    // 0: Full Cycle (default): This routine will randomly select the LED
    //    color, pattern, and speed for a random period of time.
    //
    // 1: Off: This setting turns ALL lights OFF. I added this to allow a
    //    microcontroller to turn off the lights without having to kill
    //    the supply power.
    //
    // 2: Obi Wan: The Top LED’s flash Blue, the Side LED’s are Blue,
    //    and the Main White LED’s are Random.
    //
    // 3: Yoda: The Top LED’s and Side LED’s fade Green On and Off.
    //
    // 4: Sith: The Top LED’s and Side LED’s flash Red.
    //
    // 5: Search Light: All LED’s are White, the Center Bright LED is ON.
    //
    // 6: Dagobah: This is the most screen accurate mode.
    //    The Main White LED’s are ON, the side LED’s are White, the Lower 
    //    Rectangular Red LED’s are All On, and the Rear LED’s are Blinking Red.
    //
    // 7: Sparkle: All White LED’s randomly Flash
    //
    static void setLightShow(unsigned show)
    {
        setExpanderPin(EPIN_LIGHTKIT_A, !((show>>2)&1));
        setExpanderPin(EPIN_LIGHTKIT_B, !((show>>1)&1));
        setExpanderPin(EPIN_LIGHTKIT_C, !((show>>0)&1));
    }

    virtual void setup() override
    {
        //////////////////////////
        // ENCODER PINS
        //////////////////////////

        pinMode(PIN_LIFTER_ENCODER_A, INPUT);
        pinMode(PIN_LIFTER_ENCODER_B, INPUT);

        pinMode(PIN_ROTARY_ENCODER_A, INPUT);
        pinMode(PIN_ROTARY_ENCODER_B, INPUT);

        attachInterrupt(
            digitalPinToInterrupt(PIN_LIFTER_ENCODER_A),
                measureLifterEncoder, CHANGE);
        attachInterrupt(
            digitalPinToInterrupt(PIN_ROTARY_ENCODER_A),
                measureRotaryEncoder, CHANGE);

        //////////////////////////
        // MOTOR DRIVER PINS
        //////////////////////////

        // LIFTER
        pinMode(PIN_LIFTER_PWM1, OUTPUT);
        pinMode(PIN_LIFTER_PWM2, OUTPUT);
        pinMode(PIN_LIFTER_DIAG, INPUT_PULLUP);

        // ROTARY
        pinMode(PIN_ROTARY_PWM1, OUTPUT);
        pinMode(PIN_ROTARY_PWM2, OUTPUT);
        pinMode(PIN_ROTARY_DIAG, INPUT_PULLUP);

        //////////////////////////
        // LIGHT KIT PINS
        //////////////////////////

        // TODO
        // pinMode(PIN_ROTARY_LIMIT, INPUT_PULLUP);
        // pinMode(PIN_LIFTER_MEN, OUTPUT);
        // pinMode(PIN_LIFTER_MENB, OUTPUT);
        // pinMode(PIN_ROTARY_MEN, OUTPUT);
        // pinMode(PIN_ROTARY_MENB, OUTPUT);
        // pinMode(PIN_LIGHTKIT_A, OUTPUT);
        // pinMode(PIN_LIGHTKIT_B, OUTPUT);
        // pinMode(PIN_LIGHTKIT_C, OUTPUT);

        //////////////////////////

        setLightShow(kLightKit_Off);

        //////////////////////////

        if (!EEPROM.begin(EEPROM_SIZE))
        {
            println("Failed to initialize EEPROM");
        }
        else if (sSettings.read())
        {
            println("Read calibration");
        }
        //println("Generating random");
        // randomSeed(Enthropy::generate());
    }

    ///////////////////////////////////

    static bool isCalibrated()
    {
        return sSettings.fUpLimitsCalibrated && sSettings.fDownLimitsCalibrated;
    }

    static void seekToBottom(bool usePID = true)
    {
        if (!isRotaryAtRest())
        {
            DEBUG_PRINTLN("ABORT: ROTARY NOT HOME");
            DEBUG_PRINTLN(rotaryMotorCurrentPosition());
            return;
        }
        if (!usePID)
        {
            DEBUG_PRINTLN("SEEK TO BOTTOM");

            // seek down
            float mpower = LIFTER_SEEKBOTTTOM_POWER / 100.0;
            bool botLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                botLimit = lifterBottomLimit();
                if (botLimit || serialAbort())
                    break;
                lifterMotorMove(-mpower);

                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
            }
            lifterMotorStop();
            delay(500);
            if (botLimit)
            {
                DEBUG_PRINTLN("BOTTOM LIMIT REACHED");
                if (sSettings.fLifterDistance == 0)
                {
                    sSettings.fLifterDistance = abs(getLifterPosition()) + 10;
                    DEBUG_PRINT("LIFTER DISTANCE: ");
                    DEBUG_PRINTLN(sSettings.fLifterDistance);
                }
                resetLifterPosition();
            }
        }
        else
        {
            float fDIn = 0;
            float fDOut = 0;
            float fDSet = 0;
            PID<float> distance(fDIn, fDOut, fDSet, 1.0, 0.01, 0.01);
            // TargetSteering steering(0);
            distance.setAutomatic(true);
            distance.setSampleTime(1);
            distance.setOutputLimits(-350, 350);

            DEBUG_PRINTLN("SEEK TO BOTTOM PID");
        #ifdef USE_DEBUG
            uint32_t start = millis();
        #endif
            bool botLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                botLimit = lifterBottomLimit();
                if (botLimit || serialAbort())
                    break;
                fDSet = encoder_ticks;
                distance.process();
                float throttle = (fDOut / 350.0);
                if (throttle > 1.0)
                    break;
                lifterMotorMove(-throttle * 0.7);

                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINT("LIFTER ABORTED AT "); DEBUG_PRINTLN(encoder_ticks);
                    break;
                }
            }
            lifterMotorStop();
        #ifdef USE_DEBUG
            uint32_t stop = millis();
        #endif
            if (botLimit)
            {
                DEBUG_PRINTLN("BOTTOM LIMIT REACHED");
                DEBUG_PRINT("DISTANCE: ");
                DEBUG_PRINTLN(getLifterPosition());
                DEBUG_PRINT("TIME: ");
                DEBUG_PRINTLN(stop - start);
                // resetLifterPosition();
            }
        }
    }

    static bool safetyManeuver()
    {
        // On startup we'll first seek to the top and make sure
        // that the rotary is in home position. Then seek back down.
        // This should safely clear any state the scope was in.
        println("SAFETY");
        if (seekToTop(0.8, false))
        {
        #ifndef DISABLE_SAFETY_MANEUVER
            if (!sSettings.fDisableRotary)
            {
                setLightShow(kLightKit_Dagobah);

                int attempt = 0;
                while (sRotaryCircleEncoderCount == 0 && attempt++ < 5)
                {
                    // Ensure rotary in home position
                    rotaryMotorMove(-(ROTARY_MINIMUM_POWER/100.0));
                    delay(200);

                    rotateLeftHome();
                    delay(100);

                    if (!rotaryHomeLimit())
                    {
                        println("NOT HOME TRY SPIN AROUND");
                        bool rotaryWasHome = true;
                        RotaryStatus rotaryStatus;
                        rotaryMotorMove(-(ROTARY_MINIMUM_POWER/100.0));
                        for (;;)
                        {
                            if (rotaryHomeLimit())
                            {
                                // DEBUG_PRINTLN("ROTARY HOME");
                                if (!rotaryWasHome)
                                {
                                    // DEBUG_PRINTLN("ROTARY FINAL HOME");
                                    break;
                                }
                            }
                            else if (rotaryWasHome)
                            {
                                // DEBUG_PRINTLN("ROTARY NO LONGER HOME");
                                rotaryWasHome = false;
                            }
                            if (!rotaryStatus.isMoving())
                            {
                                break;
                            }
                        }
                        rotaryMotorStop();
                    }
                    if (rotaryHomeLimit())
                    {
                        println("FIND ENCODER LENGTH");
                        resetRotaryPosition();
                        encoder_rotary_last_status = millis();
                        bool rotaryWasHome = true;
                        rotaryMotorMove(-((ROTARY_MINIMUM_POWER+5)/100.0));
                        delay(100);
                        RotaryStatus rotaryStatus;
                        encoder_rotary_stop_limit = true;
                        for (;;)
                        {
                            if (rotaryHomeLimit())
                            {
                                // DEBUG_PRINTLN("ROTARY HOME");
                                if (!rotaryWasHome)
                                {
                                    // DEBUG_PRINTLN("ROTARY FINAL HOME");
                                    break;
                                }
                            }
                            else if (rotaryWasHome)
                            {
                                // DEBUG_PRINTLN("ROTARY NO LONGER HOME");
                                rotaryWasHome = false;
                            }
                        #ifdef USE_DEBUG
                            long encoder_ticks = getRotaryPosition();
                        #endif
                            if (!rotaryStatus.isMoving())
                            {
                                break;
                            }
                        }
                        encoder_rotary_stop_limit = false;
                        rotaryMotorStop();
                        delay(100);
                        sRotaryCircleEncoderCount = abs(getRotaryPosition());
                        print("ROTARY ENCODER COUNT = ");
                        println(sRotaryCircleEncoderCount);
                        if (sRotaryCircleEncoderCount < 1000)
                        {
                            // BAD try again
                            sRotaryCircleEncoderCount = 0;
                        }
                    }
                    else
                    {
                        DEBUG_PRINTLN("ROTARY NOT HOME TRY AGAIN");
                    }
                }
                // Scope position won't work
                if (!sRotaryCircleEncoderCount)
                {
                    return false;
                }
            }
        #endif

            setLightShow(kLightKit_Off);
            if (!isRotaryAtRest())
            {
                println("ABORT: ROTARY NOT HOME");
                println(rotaryMotorCurrentPosition());
                return false;
            }
            // Reset fLifterDistance length
            sSettings.fLifterDistance = 0;
            resetLifterPosition();
            seekToBottom(false);
            sSafetyManeuver = lifterBottomLimit();
            return sSafetyManeuver;
        }
        else
        {
            DEBUG_PRINTLN("ABORT: FAILED SEEK TO TOP");
            return false;
        }
    }

    static bool ensureSafetyManeuver()
    {
        if (!sSafetyManeuver)
        {
            if (!safetyManeuver())
                disableMotors();
        }
        return sSafetyManeuver;
    }

    static bool calibrate()
    {
        bool success = true;
        sCalibrating = true;
        sSettings.fMinimumPower = max(int(sSettings.fMinimumPower), LIFTER_MINIMUM_POWER);
        if (!safetyManeuver())
        {
            println("ABORT: FAILED SAFETY MANEUVER");
            sCalibrating = false;
            return false;
        }
    retry:
        if (!isRotaryAtRest())
        {
            if (!safetyManeuver())
            {
                println("ABORT: FAILED SAFETY MANEUVER");
                sCalibrating = false;
                return false;
            }
        }
        // Clear all limits
        memset(sSettings.fUpLimits, '\0', sizeof(sSettings.fUpLimits));
        memset(sSettings.fDownLimits, '\0', sizeof(sSettings.fDownLimits));
        sSettings.fUpLimitsCalibrated = false;
        sSettings.fDownLimitsCalibrated = false;
        sSettings.fMinimumPower = 0;

        long homePosition = getLifterPosition();
        int topSpeed = LIFTER_MINIMUM_POWER;
        for (topSpeed = LIFTER_MINIMUM_POWER; topSpeed <= 100; topSpeed += 5)
        {
            LifterStatus lifterStatus;
            lifterMotorMove(topSpeed / 100.0);
            delay(ENCODER_STATUS_RATE*2);
            lifterMotorStop();
            if (lifterStatus.isMoving())
                break;
        }
        seekToBottom(false);

        long targetDistance = sSettings.fLifterDistance;

        println("SEEK TO TOP");
        for (; sCalibrating && topSpeed <= 100; topSpeed += 5)
        {
            unsigned tries = 0;
            if (!isRotaryAtRest())
            {
                DEBUG_PRINTLN("ABORT: ROTARY NOT AT HOME POSITION");
                DEBUG_PRINTLN(rotaryMotorCurrentPosition());
                goto retry;
            }
            long outputLimit = max(long(topSpeed * OUTPUT_LIMIT_PRESCALE), 10L);
            print("SPEED: "); println(topSpeed);
            while (outputLimit >= 0)
            {
                tries++;
                long maxEncoderVal = 0;
                seekToBottom(false);
                delay(1000);

                TargetSteering steering(targetDistance);
                steering.setDistanceOutputLimits(outputLimit);
                steering.setSampleTime(1);
            #ifdef USE_DEBUG
                uint32_t start = millis();
            #endif
                bool topLimit;
                long start_ticks = getLifterPosition();
                DEBUG_PRINT("OUTPUT LIMIT: "); DEBUG_PRINTLN(outputLimit);
                LifterStatus lifterStatus;
                for (;;)
                {
                    long encoder_ticks = getLifterPosition();
                    maxEncoderVal = max(maxEncoderVal, encoder_ticks);
                    topLimit = lifterTopLimit();
                    if (topLimit || serialAbort())
                        break;

                    steering.setCurrentDistance(encoder_ticks);
                    float throttle = 1.0;
                    if (outputLimit > 0)
                        throttle = steering.getThrottle();
                    lifterMotorMove(throttle * (topSpeed / 100.0));

                    if (!lifterStatus.isMoving())
                    {
                        println("ABORT");
                        break;
                    }
                }
                lifterMotorStop();
                if (start_ticks == getLifterPosition())
                {
                    DEBUG_PRINTLN(" SPEED TOO LOW");
                    break;
                }
            #ifdef USE_DEBUG
                uint32_t stop = millis();
            #endif
                if (topLimit)
                {
                #if 0//def USE_DEBUG
                    DEBUG_PRINT("TOP LIMIT REACHED - ");
                    DEBUG_PRINT(topSpeed);
                    DEBUG_PRINT(" ");
                    DEBUG_PRINTLN(outputLimit);

                    long encoder_ticks = getLifterPosition();
                    DEBUG_PRINT("  DISTANCE: ");
                    DEBUG_PRINT(encoder_ticks);
                    DEBUG_PRINT(" MAX DISTANCE: ");
                    DEBUG_PRINTLN(maxEncoderVal);
                    DEBUG_PRINT("TIME: ");
                    DEBUG_PRINT(stop - start);
                    DEBUG_PRINTLN();
                #endif
                    size_t index = topSpeed/5;
                    sSettings.fUpLimits[index].valid = true;
                    sSettings.fUpLimits[index].outputLimit = outputLimit;
                    if (sSettings.fMinimumPower == 0)
                        sSettings.fMinimumPower = topSpeed;
                    break;
                }
                else
                {
                    long encoder_ticks = getLifterPosition();
                    if (outputLimit == 0)
                    {
                        DEBUG_PRINTLN(" LIMIT NOT REACHED - GIVING UP");
                        break;
                    }
                    DEBUG_PRINT(encoder_ticks);
                    DEBUG_PRINT(" LIMIT NOT REACHED - RETRYING ");
                    DEBUG_PRINT(targetDistance - encoder_ticks);
                    outputLimit -= max((targetDistance - encoder_ticks)*2, 10L);
                    if (outputLimit < 0)
                        outputLimit = 0;
                    DEBUG_PRINT(" NEW LIMIT : "); DEBUG_PRINTLN(outputLimit);
                }
                delay(1000);
                if (serialAbort())
                    break;
            }
            if (!sCalibrating || serialAbort())
            {
                println("SERIAL ABORT");
                success = false;
                break;
            }
        }
        // Abort calibration
        if (!success)
        {
            println("CALIBRATION ABORTED");
            sCalibrating = false;
            return false;
        }

        sSettings.fUpLimitsCalibrated = true;
        println("SEEK TO BOTTOM");
        DEBUG_PRINT("TOP LIMIT SWITCH: ");
        DEBUG_PRINTLN(lifterTopLimit());
        DEBUG_PRINT("BOTTOM LIMIT SWITCH: ");
        DEBUG_PRINTLN(lifterBottomLimit());
        seekToBottom(false);
        delay(2000);

        for (topSpeed = sSettings.fMinimumPower; sCalibrating && topSpeed <= 100; topSpeed += 5)
        {
            float limit;
            if (!getUpOutputLimit(topSpeed/100.0f, limit))
                continue;
            long outputLimit = max(long(limit*1.5), 160L);
            while (success && outputLimit > 0)
            {
                long minEncoderVal = 0x7FFFFFFFL;
                if (!seekToTop(0.8))
                {
                    println("SEEK TO TOP FAILED - ABORT");
                    topSpeed = 200;
                    success = false;
                    break;
                }

                TargetSteering steering(homePosition);
                steering.setDistanceOutputLimits(outputLimit);
                steering.setSampleTime(1);
            #ifdef USE_DEBUG
                uint32_t start = millis();
            #endif
                bool botLimit;
                long start_ticks = getLifterPosition();
                LifterStatus lifterStatus;
                do
                {
                    long encoder_ticks = getLifterPosition();
                    minEncoderVal = min(minEncoderVal, encoder_ticks);
                    botLimit = lifterBottomLimit();
                    steering.setCurrentDistance(encoder_ticks);
                    lifterMotorMove(steering.getThrottle() * (topSpeed / 100.0));
                }
                while (!botLimit && !serialAbort() && lifterStatus.isMoving());
                lifterMotorStop();
                if (start_ticks == getLifterPosition())
                {
                    DEBUG_PRINTLN(" SPEED TOO LOW");
                    success = false;
                    break;
                }
            #ifdef USE_DEBUG
                uint32_t stop = millis();
            #endif
                if (botLimit)
                {
                    // DEBUG_PRINT("BOT LIMIT REACHED - ");
                    // DEBUG_PRINT(topSpeed);
                    // DEBUG_PRINT(" ");
                    // DEBUG_PRINTLN(outputLimit);

                    // long encoder_ticks = getLifterPosition();
                    // DEBUG_PRINT("  DISTANCE: ");
                    // DEBUG_PRINT(encoder_ticks);
                    // DEBUG_PRINT(" MAX DISTANCE: ");
                    // DEBUG_PRINTLN(minEncoderVal);
                    // DEBUG_PRINT("TIME: ");
                    // DEBUG_PRINT(stop - start);
                    // if (encoder_ticks > minEncoderVal)
                    // {
                    //     DEBUG_PRINT(" BOUNCE: ");
                    //     DEBUG_PRINT(encoder_ticks - minEncoderVal);
                    // }
                    // DEBUG_PRINTLN();
                    size_t index = topSpeed/5;
                    // DEBUG_PRINT("index: "); DEBUG_PRINTLN(index);
                    sSettings.fDownLimits[index].valid = true;
                    sSettings.fDownLimits[index].outputLimit = outputLimit;
                    break;
                }
                else
                {
                    long encoder_ticks = getLifterPosition();
                    // DEBUG_PRINT(encoder_ticks);
                    // DEBUG_PRINT(" LIMIT NOT REACHED - RETRYING ");
                    // DEBUG_PRINT(targetDistance - encoder_ticks);
                    outputLimit -= max(encoder_ticks*2, 1L);
                    if (outputLimit < 0)
                        outputLimit = 0;
                    // DEBUG_PRINT(" NEW LIMIT : "); DEBUG_PRINTLN(outputLimit);
                }
                delay(2000);
                if (serialAbort())
                    break;
            }
            if (!sCalibrating || serialAbort())
            {
                println("SERIAL ABORT");
                success = false;
                break;
            }
        }
        seekToBottom();
        if (!success)
        {
            println("CALIBRATION ABORTED");
            sSettings.fMinimumPower = max(int(sSettings.fMinimumPower), LIFTER_MINIMUM_POWER);
            sCalibrating = false;
            return false;
        }

        sSettings.fDownLimitsCalibrated = true;
    #ifndef USE_DEBUG
        writeSettingsToEEPROM();
    #endif
        println("SUCCESS");
        sCalibrating = false;
        return true;
    }

    static void animate()
    {
        bool retry = false;
        rotaryMotorUpdate();
        if (fMoveMode)
        {
            if (fMoveModeNextCmd < millis())
            {
                do
                {
                    switch (random(3))
                    {
                        case 0:
                        {
                            // random position random speed in range
                            uint8_t pos = random(100);
                            uint8_t speedpercentage = min(max(int(fMoveModeNextLifterSpeed), int(sSettings.fMinimumPower)), 100);
                            if (!seekToPosition(pos/100.0, speedpercentage/100.0))
                            {
                                // position failed lets retry once
                                retry = !retry;
                                continue;
                            }
                            // print("MOVE SEEK ");
                            // print(pos);
                            // print(", ");
                            // println(speedpercentage);
                            retry = false;
                            break;
                        }
                        case 1:
                        {
                            // rotary command but rotary not allowed raise lifter to 100%
                            if (!rotaryAllowed())
                                seekToPosition(1.0, fMoveModeNextLifterSpeed/100.0);

                            // rotate scope
                            int32_t speed = fMoveModeNextRotarySpeed;
                            if (speed == 0)
                                speed = 80;
                            speed = max(speed, ROTARY_MINIMUM_POWER);
                            speed = -speed + random(speed*2);
                            if (abs(speed) < ROTARY_MINIMUM_POWER+5)
                                speed = (speed < 0) ? -ROTARY_MINIMUM_POWER-5 : ROTARY_MINIMUM_POWER+5;
                            speed = min(max(speed, -100), 100);
                            if (speed == 0)
                            {
                                rotateHome();
                            }
                            else
                            {
                                rotaryMotorSpeed(speed / 100.0);
                            }
                            // print("MOVE ROTATE ");
                            // println(speed);
                            retry = false;
                            break;
                        }
                        case 2:
                        {
                            // rotary command but rotary not allowed raise lifter to 100%
                            if (!rotaryAllowed())
                                seekToPosition(1.0, fMoveModeNextLifterSpeed/100.0);

                            // rotate scope degree
                            uint32_t speedpercentage = max(int(fMoveModeNextRotarySpeed), ROTARY_MINIMUM_POWER);
                            speedpercentage = random(fMoveModeNextRotarySpeed - ROTARY_MINIMUM_POWER) + ROTARY_MINIMUM_POWER;
                            float speed = speedpercentage/100.0;
                            float maxspeed = speed;
                            if (speedpercentage < fMoveModeNextRotarySpeed)
                            {
                                speedpercentage = random(fMoveModeNextRotarySpeed - speedpercentage) + speedpercentage;
                                maxspeed = speedpercentage/100.0;
                            }
                            rotaryMotorAbsolutePosition(random(360), speed, maxspeed);
                            // print("MOVE DEGREES ");
                            // print(speed*100L);
                            // print(", ");
                            // println(maxspeed*100L);
                            retry = false;
                            break;
                        }
                    }
                } while (retry);
                // Time for next command
                fMoveModeNextCmd = millis() + random(fMoveModeNextIntervalMin, fMoveModeNextIntervalMax) * 1000L;
            }
        }
    }

    static void moveMode(uint8_t nextLifterSpeed, uint8_t nextRotarySpeed, uint8_t nextIntervalMin, uint8_t nextIntervalMax)
    {
        fMoveMode = true;
        fMoveModeNextLifterSpeed = nextLifterSpeed;
        fMoveModeNextRotarySpeed = nextRotarySpeed;
        fMoveModeNextIntervalMin = nextIntervalMin;
        fMoveModeNextIntervalMax = nextIntervalMax;
        fMoveModeNextCmd = millis();
    }

    static void moveModeEnd()
    {
        fMoveMode = false;
        fMoveModeNextLifterSpeed = 0;
        fMoveModeNextRotarySpeed = 0;
        fMoveModeNextIntervalMin = 0;
        fMoveModeNextIntervalMax = 0;
        fMoveModeNextCmd = 0;
    }

private:
    ///////////////////////////////////
    // Read motor encoders
    ///////////////////////////////////

    static void measureLifterEncoder()
    {
        encoder_lifter_val = digitalRead(PIN_LIFTER_ENCODER_A);
        if (encoder_lifter_pin_A_last == LOW && encoder_lifter_val == HIGH)
        {
            if (digitalRead(PIN_LIFTER_ENCODER_B) == LOW)
            {
                encoder_lifter_ticks--;
            }
            else
            {
                encoder_lifter_ticks++;
            }
            encoder_lifter_changed++;
        }
        encoder_lifter_pin_A_last = encoder_lifter_val;
    }

    static void measureRotaryEncoder()
    {
        encoder_rotary_val = digitalRead(PIN_ROTARY_ENCODER_A);
        if (encoder_rotary_pin_A_last == LOW && encoder_rotary_val == HIGH)
        {
            if (digitalRead(PIN_ROTARY_ENCODER_B) == LOW)
            {
                encoder_rotary_ticks++;
            }
            else
            {
                encoder_rotary_ticks--;
            }
            encoder_rotary_changed++;
            // if (encoder_rotary_stop_limit && rotaryHomeLimit())
            // {
            //     encoder_rotary_stop_ticks = encoder_rotary_ticks;
            //     // Stop rotary motor if limit switch was hit
            //     rotaryMotorStop();
            // }
        }
        encoder_rotary_pin_A_last = encoder_rotary_val;
    }

    ///////////////////////////////////

    static void resetLifterChangedState()
    {
        cli();
        encoder_lifter_changed = 0;
        sei();
    }

    static void resetRotaryChangedState()
    {
        cli();
        encoder_rotary_changed = 0;
        sei();
    }

    static void resetLifterPosition()
    {
        cli();
        encoder_lifter_val = 0;
        encoder_lifter_pin_A_last = 0;
        encoder_lifter_ticks = 0;    
        sei();
    }

    static void resetLifterPositionTop()
    {
        cli();
        encoder_lifter_val = 0;
        encoder_lifter_pin_A_last = 0;
        encoder_lifter_ticks = sSettings.fLifterDistance;    
        sei();
    }

    static void resetRotaryPosition()
    {
        cli();
        encoder_rotary_val = 0;
        encoder_rotary_pin_A_last = 0;
        encoder_rotary_ticks = 0;    
        sei();
    }

    ///////////////////////////////////

    static bool getDownOutputLimit(float speed, float &limit)
    {
        speed = min(max(speed, 0.0f), 1.0f);
        size_t index = size_t(speed*100/5);
        limit = 0;
        if (index < sSettings.limitCount() && sSettings.fDownLimits[index].valid)
        {
            limit = sSettings.fDownLimits[index].outputLimit;
            return true;
        }
        DEBUG_PRINT("UNCALIBRATED SPEED - ");
        DEBUG_PRINT(speed);
        DEBUG_PRINT(" - ");
        DEBUG_PRINTLN(index);
        return false;
    }

    static bool getUpOutputLimit(float speed, float &limit)
    {
        speed = min(max(speed, 0.0f), 1.0f);
        size_t index = size_t(speed*100/5);
        limit = 0;
        if (index < sSettings.limitCount() && sSettings.fUpLimits[index].valid)
        {
            limit = sSettings.fUpLimits[index].outputLimit;
            return true;
        }
        DEBUG_PRINT("UNCALIBRATED SPEED - ");
        DEBUG_PRINT(speed);
        DEBUG_PRINT(" - ");
        DEBUG_PRINTLN(index);
        return false;
    }

    ///////////////////////////////////

public:
    static bool seekToTop(float speed = 0.5, bool usePID = true)
    {
        if (!usePID)
        {
            float mpower = sSettings.fMinimumPower/100.0 + 0.05 + 0.1 * speed;
            print(" MOTOR: "); println(mpower);

            // seek up
            bool topLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                topLimit = lifterTopLimit();
                if (topLimit || serialAbort())
                    break;
                lifterMotorMove(mpower);
                delay(3);
                lifterMotorStop();
                delay(1);
                if (!lifterStatus.isMoving())
                {
                    printf("ABORT\n");
                    break;
                }
            }
            if (topLimit)
            {
                printf("TOP LIMIT REACHED\n");
                resetLifterPositionTop();
            }
            else
            {
                printf("NOT TOP LIMIT\n");
            }
            return topLimit;
        }
        if (!isRotaryAtRest())
        {
            DEBUG_PRINTLN("ABORT: ROTARY NOT HOME");
            DEBUG_PRINTLN(rotaryMotorCurrentPosition());
            return false;
        }
        speed = min(max(speed, 0.0f), 1.0f);
        TargetSteering steering(sSettings.fLifterDistance);
        steering.setSampleTime(1);
        float limit;
        if (!getUpOutputLimit(speed, limit))
            return false;
        steering.setDistanceOutputLimits(limit);
    #ifdef USE_DEBUG
        uint32_t start = millis();
    #endif
        bool topLimit;
        LifterStatus lifterStatus;
        for (;;)
        {
            long encoder_ticks = getLifterPosition();
            topLimit = lifterTopLimit();
            if (topLimit || serialAbort())
                break;
            steering.setCurrentDistance(encoder_ticks);
            lifterMotorMove(steering.getThrottle() * speed);

            if (!lifterStatus.isMoving())
            {
                DEBUG_PRINT("LIFTER ABORTED AT "); DEBUG_PRINTLN(encoder_ticks);
                break;
            }
        }
        lifterMotorStop();
    #ifdef USE_DEBUG
        uint32_t stop = millis();
    #endif
        if (topLimit)
        {
            DEBUG_PRINTLN("TOP LIMIT REACHED");
            DEBUG_PRINT("DISTANCE: ");
            DEBUG_PRINTLN(getLifterPosition());
            DEBUG_PRINT("TIME: ");
            DEBUG_PRINTLN(stop - start);
            return true;
        }
        return false;
    }

    static bool seekToPositionSlow(float pos, float speed)
    {
        DEBUG_PRINT("SLOW: "); DEBUG_PRINT(speed);
        // ensure position is in the range of 0.0 [bottom] - 1.0 [top]
        pos = min(max(abs(pos), 0.0f), 1.0f);
        if (isRotarySpinning() || !rotaryHomeLimit())
        {
            // Cannot go below 50% if spinning or not at home position
            pos = min(max(pos, 0.5f), 1.0f);
        }

        long maxlen = sSettings.fLifterDistance;
        long target_ticks = pos * maxlen;
        bool success = false;
        float mpower = sSettings.fMinimumPower/100.0 + 0.05 + 0.1 * speed;
        DEBUG_PRINT(" MOTOR: "); DEBUG_PRINTLN(mpower);
        if (target_ticks > getLifterPosition())
        {
            // seek up
            bool topLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                topLimit = lifterTopLimit();
                if (topLimit || encoder_ticks == target_ticks || serialAbort())
                    break;
                lifterMotorMove(mpower);
                delay(3);
                lifterMotorStop();
                delay(2);
                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
            }
            success = topLimit;
        }
        else
        {
            // seek down
            bool botLimit;
            LifterStatus lifterStatus;
            for (;;)
            {
                long encoder_ticks = getLifterPosition();
                botLimit = lifterBottomLimit();
                if (botLimit || encoder_ticks == target_ticks || serialAbort())
                    break;
                lifterMotorMove(-mpower);
                delay(3);
                lifterMotorStop();
                delay(1);

                if (!lifterStatus.isMoving())
                {
                    DEBUG_PRINTLN("ABORT");
                    break;
                }
            }
            success = botLimit;
        }
        lifterMotorStop();
        return success;
    }

    ///////////////////////////////////

    class RotaryStatus
    {
    public:
        RotaryStatus()
        {
            resetRotaryChangedState();
            encoder_rotary_last_status = millis();
        }

        bool isMoving()
        {
            if (millis() - encoder_rotary_last_status >= ENCODER_STATUS_RATE)
            {
                encoder_rotary_last_status = millis();
                if (encoder_rotary_changed < 20)
                    return false;
                resetRotaryChangedState();
            }
            return !rotaryMotorFault();
        }
    };

    class LifterStatus
    {
    public:
        LifterStatus()
        {
            resetLifterChangedState();
            encoder_lifter_last_status = millis();
        }

        bool isMoving()
        {
            if (millis() - encoder_lifter_last_status >= ENCODER_STATUS_RATE)
            {
                encoder_lifter_last_status = millis();
                if (encoder_lifter_changed < 20)
                {
                    printf("NO CHANGE\n");
                    return false;
                }
                resetLifterChangedState();
            }
            // printf("lifterMotorFault() : %d\n", lifterMotorFault());
            return !lifterMotorFault();
        }
    };

    ///////////////////////////////////

    static volatile long encoder_lifter_ticks;
    static long encoder_lifter_ticks_old;
    static volatile int encoder_lifter_pin_A_last;
    static volatile int encoder_lifter_val;
    static volatile uint16_t encoder_lifter_changed;
    static uint32_t encoder_lifter_last_status;

    ///////////////////////////////////

    static volatile long encoder_rotary_ticks;
    static volatile long encoder_rotary_stop_ticks;
    static long encoder_rotary_ticks_old;
    static volatile int encoder_rotary_pin_A_last;
    static volatile int encoder_rotary_val;
    static volatile uint16_t encoder_rotary_changed;
    static volatile bool encoder_rotary_stop_limit;
    static uint32_t encoder_rotary_last_status;

    static bool fMotorsEnabled;
    static uint32_t fMotorsEnabledTime;
    static float fLifterThrottle;
    static float fRotarySpeed;
    static float fRotaryThrottle;
    static uint32_t fRotaryThrottleUpdate;
    static uint32_t fRotaryEncoderLastStatus;
    static long fRotaryEncoderLastTick;
    static bool fRotaryMoving;

    static bool fMoveMode;
    static uint32_t fMoveModeNextCmd;
    static uint8_t fMoveModeNextLifterSpeed;
    static uint8_t fMoveModeNextRotarySpeed;
    static uint8_t fMoveModeNextIntervalMin;
    static uint8_t fMoveModeNextIntervalMax;
};

volatile long PeriscopeLifter::encoder_lifter_ticks;
long PeriscopeLifter::encoder_lifter_ticks_old;
volatile int PeriscopeLifter::encoder_lifter_pin_A_last;
volatile int PeriscopeLifter::encoder_lifter_val;
volatile uint16_t PeriscopeLifter::encoder_lifter_changed;
uint32_t PeriscopeLifter::encoder_lifter_last_status;

volatile long PeriscopeLifter::encoder_rotary_ticks;
volatile long PeriscopeLifter::encoder_rotary_stop_ticks;
long PeriscopeLifter::encoder_rotary_ticks_old;
volatile int PeriscopeLifter::encoder_rotary_pin_A_last;
volatile int PeriscopeLifter::encoder_rotary_val;
volatile uint16_t PeriscopeLifter::encoder_rotary_changed;
volatile bool PeriscopeLifter::encoder_rotary_stop_limit;
uint32_t PeriscopeLifter::encoder_rotary_last_status;

bool PeriscopeLifter::fMotorsEnabled;
uint32_t PeriscopeLifter::fMotorsEnabledTime;
float PeriscopeLifter::fLifterThrottle;
float PeriscopeLifter::fRotarySpeed;
float PeriscopeLifter::fRotaryThrottle;
uint32_t PeriscopeLifter::fRotaryThrottleUpdate;
uint32_t PeriscopeLifter::fRotaryEncoderLastStatus;
long PeriscopeLifter::fRotaryEncoderLastTick;
bool PeriscopeLifter::fRotaryMoving;

bool PeriscopeLifter::fMoveMode;
uint32_t PeriscopeLifter::fMoveModeNextCmd;
uint8_t PeriscopeLifter::fMoveModeNextLifterSpeed;
uint8_t PeriscopeLifter::fMoveModeNextRotarySpeed;
uint8_t PeriscopeLifter::fMoveModeNextIntervalMin;
uint8_t PeriscopeLifter::fMoveModeNextIntervalMax;

///////////////////////////////////////////////////////

PeriscopeLifter lifter;

///////////////////////////////////////////////////////

void setup()
{
    REELTWO_READY();

    mountSDFileSystem();

    delay(200);

    if (getSDCardMounted())
    {
        File binImage = SD.open("/UPPITY.BIN");
        if (binImage)
        {
            Serial.println("Firmware image found on SD card");
            Serial.print("Reflashing");
            Update.begin(binImage.size());
            uint32_t readSize = 0;
            while (binImage.available())
            {
                uint8_t buf = binImage.read();
                Update.write(&buf, 1);
                readSize++;
                if ((readSize % 102400) == 0)
                    Serial.print(".");
            }
            Serial.println("");
            binImage.close();
            // Delete the image file so we don't constantly reflash the box
            SD.remove("/UPPITY.BIN");
            if (Update.end(true))
            {
                Serial.println("Update Success: "); Serial.println(readSize);
                Serial.println("Rebooting...");
                // preferences.end();
                ESP.restart();
            }
        }
    }

    SetupEvent::ready();
    //////////////////////////

    Serial.println("READY");

    // N.B.: Must call pinMode() before begin()
    sGPIOExpander.pinMode(EPIN_ROTARY_LIMIT, INPUT_PULLUP);
    sGPIOExpander.pinMode(EPIN_LIGHTKIT_A, OUTPUT);
    sGPIOExpander.pinMode(EPIN_LIGHTKIT_B, OUTPUT);
    sGPIOExpander.pinMode(EPIN_LIGHTKIT_C, OUTPUT);
    sGPIOExpander.pinMode(EPIN_MOTOR_EN, OUTPUT);
    sGPIOExpander.pinMode(EPIN_MOTOR_ENB, OUTPUT);
    sGPIOExpander.pinMode(EPIN_LIFTER_TOPLIMIT, INPUT_PULLUP);
    sGPIOExpander.pinMode(EPIN_LIFTER_BOTLIMIT, INPUT_PULLUP);
    sGPIOExpander.begin();
    lifter.disableMotors();
    scan_i2c();
}

int atoi(const char* cmd, int numdigits)
{
    int result = 0;
    for (int i = 0; i < numdigits; i++)
        result = result*10 + (cmd[i]-'0');
    return result;
}

int32_t strtol(const char* cmd, const char** endptr)
{
    bool sign = false;
    int32_t result = 0;
    if (*cmd == '-')
    {
        cmd++;
        sign = true;
    }
    while (isdigit(*cmd))
    {
        result = result*10L + (*cmd-'0');
        cmd++;
    }
    *endptr = cmd;
    return (sign) ? -result : result;
}

uint32_t strtolu(const char* cmd, const char** endptr)
{
    uint32_t result = 0;
    while (isdigit(*cmd))
    {
        result = result*10L + (*cmd-'0');
        cmd++;
    }
    *endptr = cmd;
    return result;
}

bool startswith(const char* &cmd, const char* str)
{
    size_t len = strlen(str);
    if (strncmp(cmd, str, strlen(str)) == 0)
    {
        cmd += len;
        return true;
    }
    return false;
}

bool processLifterCommand(const char* cmd)
{
    // move mode ends on the next serial command
    switch (*cmd++)
    {
        case 'S':
        {
            // stop move mode
            lifter.moveModeEnd();

            // play sequence
            uint32_t seq = strtolu(cmd, &cmd);
            if (*cmd == '\0')
            {
                seq = min(max(int(seq), 0), MAX_COMMANDS);
                if (!sSettings.readCommand(seq, sCmdBuffer, sizeof(sCmdBuffer), ":P"))
                {
                    sCmdBuffer[0] = '\0';
                }
            }
            break;
        }
        case 'P':
        {
            // stop move mode
            lifter.moveModeEnd();

            // seek lifter to position
            float speed = 1.0;
            uint32_t pos;
            if (*cmd == 'R')
            {
                pos = random(100);
                cmd++;
            }
            else
            {
                pos = strtolu(cmd, &cmd);
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage = strtolu(cmd+1, &cmd);
                if (*cmd == '\0')
                {
                    speedpercentage = min(max(int(speedpercentage), 0), 100);
                    speed = speedpercentage / 100.0;
                }
            }
            if (*cmd == '\0' || *cmd == ':')
            {
                pos = min(max(int(pos), 0), 100);
                Serial.print("POSITION: "); Serial.print(pos);
                Serial.print(" SPEED: "); Serial.println(int(speed*100));
                lifter.seekToPosition(pos/100.0, speed);
            }
            break;
        }
        case 'M':
        {
            // stop move mode
            lifter.moveModeEnd();

            // move
            int nextLifterSpeed = sSettings.fMinimumPower+5;
            int nextRotarySpeed = ROTARY_MINIMUM_POWER+5;
            int nextIntervalMin = 1 + random(MOVEMODE_MAX_INTERVAL);
            int nextIntervalMax = nextIntervalMin + random(MOVEMODE_MAX_INTERVAL);
            if (*cmd == ',')
            {
                // command lifter speed
                nextLifterSpeed = strtolu(cmd+1, &cmd);
                nextLifterSpeed = max(nextLifterSpeed, int(sSettings.fMinimumPower));
            }
            if (*cmd == ',')
            {
                // command speed
                nextRotarySpeed = strtolu(cmd+1, &cmd);
                nextRotarySpeed = max(nextRotarySpeed, ROTARY_MINIMUM_POWER+5);
            }
            if (*cmd == ',')
            {
                // command interval
                nextIntervalMin = strtolu(cmd+1, &cmd);
                nextIntervalMin = max(nextIntervalMin, 1); // minimum duration 1 second
            }
            if (*cmd == ',')
            {
                // command interval
                nextIntervalMax = strtolu(cmd+1, &cmd);
                nextIntervalMax = max(nextIntervalMax, nextIntervalMin+1); // minimum duration + 1 second
            }
            lifter.moveMode(nextLifterSpeed, nextRotarySpeed, nextIntervalMin, nextIntervalMax);
            break;
        }
        case 'R':
        {
            // stop move mode
            lifter.moveModeEnd();

            // spin rotary speed
            int32_t speed = 0;
            if (*cmd == 'R')
            {
                speed = strtolu(cmd+1, &cmd);
                if (speed == 0)
                    speed = 80;
                speed = max(speed, ROTARY_MINIMUM_POWER);
                speed = -speed + random(speed*2);
                if (abs(speed) < ROTARY_MINIMUM_POWER)
                    speed = (speed < 0) ? -ROTARY_MINIMUM_POWER : ROTARY_MINIMUM_POWER;
            }
            else
            {
                speed = strtol(cmd, &cmd);
            }
            if (*cmd == '\0' && lifter.rotaryAllowed())
            {
                speed = min(max(speed, -100), 100);
                Serial.print("ROTARY SPEED: "); Serial.println(speed);
                if (speed == 0)
                {
                    lifter.rotateHome();
                }
                else
                {
                    lifter.rotaryMotorSpeed(speed / 100.0);
                }
            }
            break;
        }
        case 'A':
        case 'D':
        {
            bool relative = (cmd[-1] == 'D');
            // stop move mode
            lifter.moveModeEnd();

            // position absolute degree
            float speed = 0;
            float maxspeed = 0;
            int32_t degrees;
            if (*cmd == 'R' && (cmd[1] == ',' || cmd[1] == '\0'))
            {
                degrees = random(360);
                cmd++;
            }
            else
            {
                degrees = strtol(cmd, &cmd);
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage;
                if (cmd[1] == 'R' && (cmd[2] == ',' || cmd[2] == '\0'))
                {
                    speedpercentage = random(100);
                    cmd += 2;
                }
                else
                {
                    speedpercentage = strtolu(cmd+1, &cmd);
                }
                if (*cmd == ',' || *cmd == '\0')
                {
                    speedpercentage = max(min(max(int(speedpercentage), 0), 100), ROTARY_MINIMUM_POWER);
                    speed = speedpercentage / 100.0;
                }
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage;
                if (cmd[1] == 'R' && (cmd[2] == ',' || cmd[2] == '\0'))
                {
                    speedpercentage = speed * 100.0 + random(100 - speed * 100.0);
                    cmd += 2;
                }
                else
                {
                    speedpercentage = strtolu(cmd+1, &cmd);
                }
                if (*cmd == '\0')
                {
                    speedpercentage = max(min(max(int(speedpercentage), 0), 100), int(speed)*100);
                    maxspeed = speedpercentage / 100.0;
                }
            }
            if (*cmd == '\0' && lifter.rotaryAllowed())
            {
                Serial.print("ROTARY DEGREE: "); Serial.println(degrees);
                if (relative)
                {
                    lifter.rotaryMotorRelativePosition(degrees);
                }
                else if (degrees == 0)
                {
                    lifter.rotateHome();
                }
                else
                {
                    lifter.rotaryMotorAbsolutePosition(degrees, speed, maxspeed);
                }
            }
            break;
        }
        case 'W':
        {
            // wait seconds
            bool rand = false;
            uint32_t seconds;
            Serial.print("WAIT: "); Serial.println(cmd);
            if ((rand = (*cmd == 'R')))
                cmd++;
            seconds = strtolu(cmd, &cmd);
            if (rand)
            {
                Serial.println(seconds);
                if (seconds == 0)
                    seconds = 6;
                seconds = random(1, seconds);
                Serial.println(seconds);
            }
            Serial.print("WAIT SECONDS: "); Serial.println(seconds);
            if (*cmd == '\0')
            {
                sWaitNextSerialCommand = millis() + uint32_t(min(max(int(seconds), 1), 600)) * 1000L;
            }
            break;
        }
        case 'L':
        {
            // play light sequence
            uint32_t seq;
            if (*cmd == 'R' && cmd[1] == '\0')
            {
                do
                {
                    seq = random(8);
                }
                while (seq == lifter.kLightKit_Off);
                Serial.print("RAND: "); Serial.println(seq);
                cmd++;
            }
            else
            {
                seq = strtolu(cmd, &cmd);
            }
            if (*cmd == '\0')
            {
                seq = min(max(int(seq), 0), 7);
                if (seq == 0 && seq != lifter.kLightKit_Off)
                {
                    lifter.setLightShow(lifter.kLightKit_Off);
                    delay(10);
                }
                Serial.print("LIGHT: "); Serial.println(seq);
                lifter.setLightShow(seq);
            }
            break;
        }
        case 'H':
        {
            // stop move mode
            lifter.moveModeEnd();

            // return home
            lifter.ensureSafetyManeuver();
            if (!lifter.lifterBottomLimit())
            {
                uint32_t speed = sSettings.fMinimumPower;
                lifter.rotateHome();
                lifter.setLightShow(lifter.kLightKit_Off);
                lifter.rotateHome();
                if (isdigit(*cmd))
                {
                    speed = min(max(strtolu(cmd, &cmd), sSettings.fMinimumPower), 100u);
                }
                if (!lifter.seekToPosition(0, speed/100.0))
                {
                    // If we failed we try one more time
                    lifter.seekToPosition(1.0, speed/100.0);
                    lifter.rotateHome();
                    lifter.rotateHome();
                    lifter.seekToPosition(0, speed/100.0);
                }
            }
            break;
        }
        default:
            // stop move mode
            lifter.moveModeEnd();

            Serial.println("Invalid");
            return false;
    }
    return true;
}

void processConfigureCommand(const char* cmd)
{
    if (strcmp(cmd, "#PSC") == 0)
    {
        // calibrate
        if (!lifter.calibrate())
            lifter.disableMotors();
    }
    else if (strcmp(cmd, "#PRC") == 0)
    {
        sRCMode = !sRCMode;
    }
    else if (startswith(cmd, "#PZERO"))
    {
        sSettings.clearCommands();
    }
    else if (startswith(cmd, "#PL"))
    {
        sSettings.listSortedCommands(Serial);
    }
    else if (startswith(cmd, "#PD"))
    {
        if (isdigit(*cmd))
        {
            uint32_t seq = strtolu(cmd, &cmd);
            Serial.println(seq);
            if (sSettings.deleteCommand(seq))
                Serial.println("Deleted");
        }
    }
    else if (startswith(cmd, "#PBAUD"))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        if (baudrate > 1200 && sSettings.fBaudRate != baudrate)
        {
            sSettings.fBaudRate = baudrate;
            Serial.print("Reboot baud rate: "); Serial.println(sSettings.fBaudRate);
            sSettings.write();
        }
    }
    else if (startswith(cmd, "#PR"))
    {
        sSettings.fDisableRotary = !sSettings.fDisableRotary;
        sSettings.write();
        Serial.println(sSettings.fDisableRotary ? "Disabled" : "Enabled");
        void (*resetArduino)() = NULL;
        Serial.flush(); delay(1000);
        resetArduino();
    }
    else if (startswith(cmd, "#PN"))
    {
        if (cmd[1] == 'L')
        {
            Serial.println("LIFT");
            // PNCL - lifter limit normally closed
            // PNOL - lifter limit normally open
           if ((cmd[0] == 'C' && sSettings.fLifterLimitSetting) || 
                (cmd[0] == 'O' && !sSettings.fLifterLimitSetting))
            {
                Serial.println("Changed");
                sSettings.fLifterLimitSetting = !sSettings.fLifterLimitSetting;
                sSettings.write();
            }
        }
        else if (cmd[1] == 'R')
        {
            // PNCR - rotary limit normally closed
            // PNOR - rotary limit normally open
            if ((cmd[0] == 'C' && sSettings.fRotaryLimitSetting) || 
                (cmd[0] == 'O' && !sSettings.fRotaryLimitSetting))
            {
                Serial.println("Changed");
                sSettings.fRotaryLimitSetting = !sSettings.fRotaryLimitSetting;
                sSettings.write();
            }
        }
    }
    else if (startswith(cmd, "#PS"))
    {
        uint32_t storeseq = strtolu(cmd, &cmd);
        if (*cmd == ':')
        {
            storeseq = min(max(int(storeseq), 0), 100);
            const char* startcmd = ++cmd;
            while (*cmd != '\0')
            {
                switch (*cmd)
                {
                    case 'P':
                    {
                        // position
                        float speed = 1.0;
                        uint32_t pos = strtolu(cmd+1, &cmd);
                        // optional speed
                        if (*cmd == ',')
                        {
                            uint32_t speedpercentage = strtolu(cmd+1, &cmd);
                            speedpercentage = max(min(max(int(speedpercentage), 0), 100), int(sSettings.fMinimumPower));
                            speed = speedpercentage / 100.0;
                        }
                        pos = min(max(int(pos), 0), 100);
                        Serial.print("Lifter Position: ");
                        Serial.print(pos);
                        Serial.print(" Speed: "); Serial.println(int(speed*100));
                        break;
                    }
                    case 'R':
                    {
                        // speed
                        int32_t speed = strtol(cmd+1, &cmd);
                        speed = min(max(speed, -100), 100);
                        Serial.print("Rotate Scope Speed: ");
                        Serial.println(speed);
                        break;
                    }
                    case 'A':
                    case 'D':
                    {
                        bool relative = (cmd[0] == 'D');
                        bool randdegrees = false;
                        bool randspeed = false;
                        bool randmax = false;
                        uint32_t speed = 0;
                        uint32_t maxspeed = 0;
                        int32_t degrees = 0;
                        cmd += 1;
                        if (*cmd == 'R')
                        {
                            randdegrees = true;
                            cmd++;
                        }
                        else
                        {
                            degrees = strtol(cmd, &cmd);
                        }
                        Serial.println(cmd);
                        // optional speed
                        if (*cmd == ',')
                        {
                            if (cmd[1] == 'R')
                            {
                                randspeed = true;
                                cmd += 2;
                            }
                            else
                            {
                                speed = strtolu(cmd+1, &cmd);
                                speed = max(min(max(int(speed), 0), 100), ROTARY_MINIMUM_POWER);
                            }
                        }
                        // optional maxspeed
                        if (*cmd == ',')
                        {
                            if (cmd[1] == 'R')
                            {
                                randmax = true;
                                cmd += 2;
                            }
                            else
                            {
                                maxspeed = strtolu(cmd+1, &cmd);
                                maxspeed = max(min(max(int(maxspeed), 0), 100), int(speed));
                            }
                        }
                        Serial.print("Rotate ");
                        Serial.print((relative) ? "Relative" : "Absolute");
                        Serial.print(" Degrees: ");
                        if (randdegrees)
                            Serial.print("Random");
                        else
                            Serial.print(degrees);
                        if (speed != 0 || randspeed)
                        {
                            Serial.print(" Speed: ");
                            if (randspeed)
                                Serial.print("Random");
                            else
                                Serial.print(speed);
                        }
                        if (maxspeed != 0 || randmax)
                        {
                            Serial.print(" Max Speed: ");
                            if (randmax)
                                Serial.print("Random");
                            else
                                Serial.print(maxspeed);
                        }
                        Serial.println();
                        break;
                    }
                    case 'M':
                    {
                        // move
                        int nextLifterSpeed = sSettings.fMinimumPower+5;
                        int nextRotarySpeed = ROTARY_MINIMUM_POWER+5;
                        int nextIntervalMin = MOVEMODE_MAX_INTERVAL;
                        int nextIntervalMax = MOVEMODE_MAX_INTERVAL;
                        cmd++;
                        if (*cmd == ',')
                        {
                            // command lifter speed
                            nextLifterSpeed = strtolu(cmd+1, &cmd);
                            nextLifterSpeed = max(nextLifterSpeed, int(sSettings.fMinimumPower));
                        }
                        if (*cmd == ',')
                        {
                            // command speed
                            nextRotarySpeed = strtolu(cmd+1, &cmd);
                            nextRotarySpeed = max(nextRotarySpeed, ROTARY_MINIMUM_POWER+5);
                        }
                        if (*cmd == ',')
                        {
                            // command interval
                            nextIntervalMin = strtolu(cmd+1, &cmd);
                            nextIntervalMin = max(nextIntervalMin, 1); // minimum duration 1 second
                        }
                        if (*cmd == ',')
                        {
                            // command interval
                            nextIntervalMax = strtolu(cmd+1, &cmd);
                            nextIntervalMax = max(nextIntervalMax, nextIntervalMin+1); // minimum duration + 1 second
                        }
                        Serial.print("Move Continous: ");
                        Serial.print(" Lifter: ");
                        Serial.print(nextLifterSpeed);
                        Serial.print(" Rotary: ");
                        Serial.print(nextRotarySpeed);
                        Serial.print(" Min: ");
                        Serial.print(nextIntervalMin);
                        Serial.print(" Max: ");
                        Serial.println(nextIntervalMax);
                        break;
                    }
                    case 'W':
                    {
                        // seconds
                        bool randseconds = false;
                        int seconds = 0;
                        cmd++;
                        if (*cmd == 'R')
                        {
                            randseconds = true;
                            cmd++;
                        }
                        seconds = strtolu(cmd, &cmd);
                        seconds = min(max(seconds, 0), 600);
                        Serial.print("Wait Seconds: ");
                        if (randseconds)
                            Serial.println("Random");
                        else
                            Serial.println(seconds);
                        break;
                    }
                    case 'L':
                    {
                        // light kit sequence
                        int seq = 0;
                        bool randseq = false;
                        cmd++;
                        if (*cmd == 'R')
                        {
                            seq = true;
                            cmd++;
                        }
                        else
                        {
                            seq = strtolu(cmd, &cmd);
                        }
                        seq = min(max(seq, 0), 7);
                        Serial.print("Light Kit Sequence: ");
                        if (randseq)
                            Serial.println("Random");
                        else
                            Serial.println(seq);
                        break;
                    }
                    case 'H':
                    {
                        // return home
                        unsigned speed = sSettings.fMinimumPower; cmd++;
                        if (isdigit(*cmd))
                        {
                            speed = min(unsigned(max(strtolu(cmd, &cmd), sSettings.fMinimumPower)), 100u);
                        }
                        Serial.print("Return Home: Speed: "); Serial.println(speed);
                        break;
                    }
                    default:
                        cmd = nullptr;
                        break;
                }
                if (cmd != nullptr && *cmd == ':')
                {
                    cmd++;
                }
                else if (cmd == nullptr || *cmd != '\0')
                {
                    Serial.println("Invalid");
                    Serial.println(cmd);
                    cmd = nullptr;
                    break;
                }
            }
            if (cmd != nullptr && *cmd == '\0')
            {
                Serial.print("Sequence [");
                Serial.print(storeseq);
                Serial.print("]: ");
                Serial.println(startcmd);
                sCmdBuffer[0] = '\0';
                if (sSettings.writeCommand(storeseq, startcmd))
                    Serial.println("Stored");
                else
                    Serial.println("Failed");
            }
        }
        else
        {
            Serial.println("Invalid");
        }
    }
}

bool processCommand(const char* cmd, bool firstCommand)
{
    sWaitNextSerialCommand = 0;
    if (*cmd == '\0')
        return true;
    if (!firstCommand)
    {
        if (cmd[0] != ':')
        {
            Serial.println("Invalid");
            return false;
        }
        return processLifterCommand(cmd+1);
    }
    switch (cmd[0])
    {
        case ':':
            if (cmd[1] == 'P')
                return processLifterCommand(cmd+2);
            break;
        case '#':
            processConfigureCommand(cmd);
            return true;
        default:
            Serial.println("Invalid");
            break;
    }
    return false;
}

void loop()
{
    AnimatedEvent::process();
    lifter.animate();

    // append commands to command buffer
    if (Serial.available())
    {
        int ch = Serial.read();
        if (ch == 0x0A || ch == 0x0D)
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
    if (sProcessing && millis() > sWaitNextSerialCommand)
    {
        if (sCmdBuffer[0] == ':')
        {
            char* end = strchr(sCmdBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sCmdBuffer, !sCmdNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT("Unrecognized: "); DEBUG_PRINTLN(sCmdBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sCmdBuffer, end);
                DEBUG_PRINT("REMAINS: ");
                DEBUG_PRINTLN(sCmdBuffer);
                sCmdNextCommand = true;
            }
            else
            {
                sCmdBuffer[0] = '\0';
                sCmdNextCommand = false;
            }
        }
        else if (sBuffer[0] == ':')
        {
            char* end = strchr(sBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sBuffer, !sNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT("Unrecognized: "); DEBUG_PRINTLN(sBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sBuffer, end);
                sPos = strlen(sBuffer);
                DEBUG_PRINT("REMAINS: ");
                DEBUG_PRINTLN(sBuffer);
                sNextCommand = true;
            }
            else
            {
                resetSerialCommand();
                sBuffer[0] = '\0';
            }
        }
        else
        {
            processCommand(sBuffer, true);
            resetSerialCommand();
        }
    }
    if (lifter.isIdle() && lifter.motorsEnabled())
    {
        DEBUG_PRINTLN("DISABLE MOTORS");
        lifter.disableMotors();
    }
    static bool sHome;
    if (sHome != lifter.rotaryHomeLimit())
    {
        sHome = lifter.rotaryHomeLimit();
        if (sHome)
            Serial.println("HOME");
    }
    if (sDigitalReadAll)
    {
        sDigitalReadAll = false;
        static PCF8574::DigitalInput sLastFlags;
        PCF8574::DigitalInput sNowFlags;
        sNowFlags = sGPIOExpander.digitalReadAll();
        if (memcmp(&sLastFlags, &sNowFlags, sizeof(sLastFlags)) != 0)
        {
            sLastFlags = sNowFlags;
        }
    }
    if (sRCMode)
    {
        int16_t channels[6];
        for (uint16_t i = 0; i < SizeOfArray(channels); i++)
        {
            channels[i] = ((int)sPPM.latestValidChannelValue(i+1, 0) - 1000) / 10;
        }
        // if (channels[3])
        // {
        //     channels[3]
        // }
        // printf("1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u\n",
        //     channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
        static uint32_t sLastReading;
        if (sLastReading + 1000 < millis())
        {
            int height = channels[2];
            static int sLastRCHeight;
            if (height >= 0 && height <= 100 && abs(height - sLastRCHeight) > 10)
            {
                snprintf(sBuffer, sizeof(sBuffer), ":PP%d,%d", height, 50);
                runSerialCommand();
                printf("NEW HEIGHT : %d\n", height);
                sLastRCHeight = height;
            }
            sLastReading = millis();
        }
    }
    // if (sPPM.decode())
    // {
    //     uint16_t channels[6];
    //     for (uint16_t i = 0; i < SizeOfArray(channels); i++)
    //     {
    //         channels[i] = sPPM.channel(i, 0, 1024, 512);
    //     }
    //     // Throttle
    //     if (channels[3] > 0)
    //     {

    //     }
    //     // printf("1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u\n",
    //     //     channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
    // }

#ifdef USE_DEBUG
    static bool sLastTop;
    static bool sLastBot;
    static bool sLastRot;
    static long sLastLifter;
    static long sLastRotary;
    if (sLastTop != lifter.lifterTopLimit() ||
        sLastBot != lifter.lifterBottomLimit() ||
        sLastRot != lifter.rotaryHomeLimit() ||
        sLastLifter != lifter.getLifterPosition() ||
        sLastRotary != lifter.getRotaryPosition())
    {
        sLastTop = lifter.lifterTopLimit();
        sLastBot = lifter.lifterBottomLimit();
        sLastRot = lifter.rotaryHomeLimit();
        sLastLifter = lifter.getLifterPosition();
        sLastRotary = lifter.getRotaryPosition();
        printf("T: %d B: %d R: %d H: %d D: %d P: %d F: %d\n",
            int(sLastTop),
            int(sLastBot),
            int(sLastRot),
            int(sLastLifter/float(sSettings.fLifterDistance)*100.0),
            int(lifter.rotaryMotorCurrentPosition()),
            int(lifter.getRotaryPosition()),
            int(lifter.lifterMotorFault()));
    }
#endif
}
