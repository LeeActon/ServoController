/*
 * main.cpp
 *
 * Created: 9/8/2019 12:57:11 PM
 * Author : Lee
 */ 

#define ARDUINO_MAIN
#define PWM1B 0
//#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <util/atomic.h>

#include <Servo8Bit.h>
#include <ErriezLKM1638Board.h>
#include <Timer1Uint32.h>

#define SERVO_COUNT 4

// Pin 13 - PA0 - INPUT - Move servo 0 to open position
//                Pull-up resister.
//                Pull LOW to activate
//                PCINT0 enabled to wake the MCU on change.
// Pin 12 - PA1 - INPUT - Move servo 1 to open position
// Pin 11 - PA2 - INPUT - Move servo 2 to open position
// Pin 10 - PA3 - INPUT - Move servo 3 to open position
// Pin  9 - PA4 - Arduino D26 - LKM1638Board clock pin
//              - ALSO DEBUGGER ISP - USCK
// Pin  8 - PA5 - Arduino D27 - LKM1638Board - DIO pin
//              - ALSO DEBUGGER ISP - MISO
// Pin  7 - PA6 - Arduino D28 - LKM1638Board - Strobe
//              - ALSO DEBUGGER ISP- MOSI
// Pin  6 - PA7 - Available
// Pin  2 - PB0 - OUTPUT - Servo 0 PWM pin
// Pin  3 - PB1 - OUTPUT - Servo 1 PWM pin
// Pin  5 - PB2 - OUTPUT - Servo 2 PWM pin
// Pin  4 - PB3 - OUTPUT - Servo 3 PWM pin
//              - ALSO DEBUGGER RESET - ISP and debugWire

#define SERVO_MIN 0
#define SERVO_MAX 180


class ServoLimits
{
    public:
    uint8_t min;
    uint8_t max;
};
    
// EEPROM memory
#define CONFIG_EEPROM_BASE_ADDRESS ((Config *) 0)
class Config  
{
    protected:
    uint8_t initialized;
    ServoLimits servoLimits[SERVO_COUNT];

    public:
    Config()
    {
        this->initialized = 0;
    }
    void Init()
    {
        if (this->initialized == 0)
        {
            eeprom_read_block(this, CONFIG_EEPROM_BASE_ADDRESS, sizeof(Config));
            if (this->initialized != 0xA5)
            {
                this->initialized = 0xA5;
                for (int i = 0; i < SERVO_COUNT; i++)
                {
                    this->servoLimits[i].max = 80;
                    this->servoLimits[i].min = 60;
                }
                eeprom_write_block(this, CONFIG_EEPROM_BASE_ADDRESS, sizeof(Config));
            }
        }
    }
    uint8_t GetOpenedValue(int iServo)
    {
        if (iServo >= 0 && iServo < SERVO_COUNT)
           return this->servoLimits[iServo].max;
           
        return 0;
    }

    uint8_t GetClosedValue(int iServo)
    {
        if (iServo >= 0 && iServo < SERVO_COUNT)
            return this->servoLimits[iServo].min;
        
        return 0;
    }
    
    void SetOpenedValue(int iServo, uint8_t value)
    {
        this->servoLimits[iServo].max = value;
        eeprom_write_byte(&(CONFIG_EEPROM_BASE_ADDRESS->servoLimits[iServo].max), value);
    }
    
    void SetClosedValue(int iServo, uint8_t value)
    {
        this->servoLimits[iServo].min = value;
        eeprom_write_byte(&(CONFIG_EEPROM_BASE_ADDRESS->servoLimits[iServo].min), value);
    }
    void IncOpenedValue(int iServo)
    {
        uint8_t value = GetOpenedValue(iServo);
        if (value < SERVO_MAX)
            SetOpenedValue(iServo, value + 1);
    }
    void DecOpenedValue(int iServo)
    {
        uint8_t value = GetOpenedValue(iServo);
        if (value > SERVO_MIN)
            SetOpenedValue(iServo, value - 1);
    }
    void IncClosedValue(int iServo)
    {
        uint8_t value = GetClosedValue(iServo);
        if (value < SERVO_MAX)
            SetClosedValue(iServo, value + 1);
    }
    void DecClosedValue(int iServo)
    {
        uint8_t value = GetClosedValue(iServo);
        if (value > 0)
            SetClosedValue(iServo, value - 1);
    }
 };

class Servo
{
    static Config config;
    static uint8_t count;
    uint8_t index;
    uint8_t servoPin;
    uint8_t triggerPin;
    enum Position {Unknown, Closed, Opened} position;
    Servo8Bit servo;
    uint32_t servoOffTime;

    public:
    Servo()
    {
        index = count++;
        servoOffTime = 0x7FFFFFFF;
        position = Unknown;
    }
    void Init(uint8_t servoPin, uint8_t triggerPin)
    {
        this->servoPin = servoPin;
        this->triggerPin = triggerPin;
        config.Init();
        DDRA &= ~(_BV(this->triggerPin));  // Input
        PORTA |= _BV(this->triggerPin);    // Enable pull-up resistors on these input pin.

    }
    void Open()
    {
        if (!this->servo.attached())
            this->servo.attach(this->servoPin);
            
        this->servo.write(config.GetOpenedValue(this->index));
        this->servoOffTime = Timer1Uint32::Future(2*Timer1Uint32::ticksPerSecond);
        this->position = Opened;
    }
    void Close()
    {
                if (!this->servo.attached())
            this->servo.attach(this->servoPin);

        this->servo.write(config.GetClosedValue(this->index));
        this->servoOffTime = Timer1Uint32::Future(2*Timer1Uint32::ticksPerSecond);
        this->position = Closed;
    }
    
    void Check()
    {
        if (Timer1Uint32::IsPast(this->servoOffTime))
            {
            this->servo.detach();
            this->servoOffTime = 0x7FFFFFFF;    
            }
        uint8_t open = ((PINA & _BV(this->triggerPin)) == 0);
        if (open && (this->position != Opened))
        {
            this->Open();
        }
        else if (!open && (this->position != Closed))
        {
            this->Close();
        }
    }
    uint8_t GetOpenedValue()
    {
        return config.GetOpenedValue(this->index);
    }
    uint8_t GetClosededValue()
    {
        return config.GetClosedValue(this->index);
    }
    void IncOpenedValue()
    {
        config.IncOpenedValue(this->index);
    }
    void DecOpenedValue()
    {
        config.DecOpenedValue(this->index);
    }
    void IncClosedValue()
    {
        config.IncClosedValue(this->index);
    }
    void DecClosedValue()
    {
        config.DecClosedValue(this->index);
    }
};
Config Servo::config;
uint8_t Servo::count = 0;

Servo servos[SERVO_COUNT];

LKM1638Board lkm1638Board(6, 5, 4);

#define NEXT_SERVO_BUTTON   0x01
#define PREV_SERVO_BUTTON   0x02
#define INC_CLOSED_VALUE_BUTTON  0x04
#define DEC_CLOSED_VALUE_BUTTON  0x08
#define INC_OPENED_VALUE_BUTTON  0x10
#define DEC_OPENED_VALUE_BUTTTON 0x20
#define CLOSE_SERVO_BUTTON  0x40
#define OPEN_SERVO_BUTTON   0x80


void DisplayAt(uint8_t value, int iPosition)
{
    for (int i = 0; i < 4; i++)
    {
        uint8_t digit = value % 10;
        lkm1638Board.setDigit(iPosition++, digit);
        value /= 10;
    }
}

int main(void)
{
    uint32_t autoRepeatTime;
    
    servos[0].Init(PB0, PA0);
    servos[1].Init(PB1, PA1);
    servos[2].Init(PB2, PA2);
    servos[3].Init(PB3, PA3);
    
    lkm1638Board.begin();
    lkm1638Board.clear();

    Timer1Uint32::Start(CLOCK_PRESCALE_DIV_8);

#if 0
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        servos[i].Open();
    }
    _delay_ms(2000);
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        servos[i].Close();
    }
#endif

    uint8_t prevButtons = 0;
    uint8_t iServoCur = 0;
    uint8_t iServoPrev = 0xFF;
    
    uint8_t prevOpenedValue = 0xFF;
    uint8_t prevClosedValue = 0xFF;

    autoRepeatTime = 0;
        
    while (1) 
    {
        for (int iServo = 0; iServo <SERVO_COUNT; iServo++)
        {
            servos[iServo].Check();
        }
        uint8_t buttons = lkm1638Board.getButtons();
        
        if (Timer1Uint32::IsPast(autoRepeatTime))
        {
            // This will cause any buttons that are currently held down to repeat
            prevButtons = 0;
        }
            
        uint8_t changedButtons = (buttons ^ prevButtons);
        
        if (changedButtons != 0)
        {
            // If there is a change, delay the auto-repeat.
            autoRepeatTime = Timer1Uint32::Future(Timer1Uint32::ticksPerSecond/4);  // Repeat 4 times per second
        }
                
        uint8_t justPressedButtons = (buttons & changedButtons);
        prevButtons = buttons;
        if ((justPressedButtons & NEXT_SERVO_BUTTON) != 0)
        {
             iServoCur++;
             if (iServoCur >= SERVO_COUNT)
                iServoCur = 0;
        }
        if ((justPressedButtons & PREV_SERVO_BUTTON) != 0)
        {
            if (iServoCur == 0)
                iServoCur = SERVO_COUNT - 1;
            else
                iServoCur--;
        }
        if ((justPressedButtons & INC_OPENED_VALUE_BUTTON) != 0)
        {
            servos[iServoCur].IncOpenedValue();
        }
        if ((justPressedButtons & DEC_OPENED_VALUE_BUTTTON) != 0)
        {
            servos[iServoCur].DecOpenedValue();   
        }
        if ((justPressedButtons & INC_CLOSED_VALUE_BUTTON) != 0)
        {
            servos[iServoCur].IncClosedValue();
        }
        if ((justPressedButtons & DEC_CLOSED_VALUE_BUTTON) != 0)
        {
            servos[iServoCur].DecClosedValue();
        }
        if ((justPressedButtons & OPEN_SERVO_BUTTON) != 0)
        {
            servos[iServoCur].Open();
        }
        if ((justPressedButtons & CLOSE_SERVO_BUTTON) != 0)
        {
            servos[iServoCur].Close();
        }
        
        if (iServoCur != iServoPrev)
        {
            iServoPrev = iServoCur;
            if ((iServoPrev >= 0) && (iServoPrev < SERVO_COUNT))
            {
            }                
            lkm1638Board.colorLEDsOff(0x0F);
            lkm1638Board.setColorLED(3 - iServoCur, LedGreen);
        }
        
        uint8_t openedValue = servos[iServoCur].GetOpenedValue();
        if (openedValue != prevOpenedValue)
        {
            DisplayAt(openedValue, 4);
            lkm1638Board.dotOn(4);
            prevOpenedValue = openedValue;
        }

        uint8_t closedValue = servos[iServoCur].GetClosededValue();
        if (closedValue != prevClosedValue)
        {
            DisplayAt(closedValue, 0);
            prevClosedValue = closedValue;
        }
    }
}
