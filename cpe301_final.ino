// Swamp Cooler Project Code
// Restrictions: Direct register manipulation, only allowed Arduino libraries for specific components

#include <Stepper.h>
#include <DHT.h>        // For temp/humidity sensor
#include <RTClib.h>     // For RTC module
#include <LiquidCrystal.h> // For LCD display

// GPIO Mapping Macros
#define GPIO_SET_OUTPUT(port, bit) ((port) |= (1 << (bit)))
#define GPIO_SET_INPUT(port, bit)  ((port) &= ~(1 << (bit)))
#define GPIO_WRITE_HIGH(port, bit) ((port) |= (1 << (bit)))
#define GPIO_WRITE_LOW(port, bit)  ((port) &= ~(1 << (bit)))
#define GPIO_TOGGLE(port, bit)     ((port) ^= (1 << (bit)))
#define GPIO_READ(port, bit)       ((port) & (1 << (bit)))

// Pin-to-Port Mappings
// PIN 24 -> PA2
#define RED_LED_PORT PORTA
#define RED_LED_BIT 2
// PIN 25 -> PA3
#define GREEN_LED_PORT PORTA
#define GREEN_LED_BIT 3
// PIN 26 -> PA4
#define BLUE_LED_PORT PORTA
#define BLUE_LED_BIT 4
// PIN 27 -> PA5
#define YELLOW_LED_PORT PORTA
#define YELLOW_LED_BIT 5
// PIN 19 -> PD2
#define START_BUTTON_PORT PORTD
#define START_BUTTON_BIT 2
// PIN 18 -> PD3
#define STOP_BUTTON_PORT PORTD
#define STOP_BUTTON_BIT 3
// PIN 2 -> PE4
#define RESET_BUTTON_PORT PORTE
#define RESET_BUTTON_BIT 4
// PIN 36 -> PC1
#define WATER_SENSOR_POWER_PORT PORTC
#define WATER_SENSOR_POWER_BIT 1
// PIN 8 -> PH5
#define FAN_PORT PORTH
#define FAN_BIT 5

// Device Pin Definitions
#define STEPPER_IN1 9
#define STEPPER_IN2 10
#define STEPPER_IN3 11
#define STEPPER_IN4 12
#define TEMP_HUMID_PIN 13
#define POTENTIOMETER_PIN A1
#define LCD_RS 28
#define LCD_EN 30
#define LCD_D4 29
#define LCD_D5 31
#define LCD_D6 33
#define LCD_D7 35

// LED Colors
enum Color { RED, GREEN, BLUE, YELLOW };

// Stepper motor setup
const int stepsPerRevolution = 2048;
Stepper ventStepper(stepsPerRevolution, STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);

// DHT sensor setup
#define DHTTYPE DHT11
DHT dht(TEMP_HUMID_PIN, DHTTYPE);

// RTC setup
RTC_DS1307 rtc;

// LCD setup
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// System States
enum State { DISABLED, IDLE, RUNNING, ERROR };
State currentState = DISABLED;

// Variables for state management
volatile float currentTemp = 0.0;
volatile float currentHumidity = 0.0;
volatile bool waterLevelLow = false;
volatile unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 6000; // 6 seconds
const unsigned long stepInterval = 100; // Minimum time between steps (in milliseconds)


void uartInit(unsigned long baud) {
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uartTransmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uartPrint(const char* str) {
    while (*str) {
        uartTransmit(*str++);
    }
}

void adcInit() {
    // Configure ADC for single-ended input on ADC0 (Water Sensor)
    ADMUX = (1 << REFS0);            // Reference voltage: AVcc
    ADCSRA = (1 << ADEN) |           // Enable ADC
             (1 << ADPS2) | (1 << ADPS1); // Prescaler = 64 for 16 MHz clock
}

uint16_t adcRead(uint8_t channel) {
    // Select ADC channel (0-7) by setting the MUX bits in ADMUX
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));

    // Return ADC result
    return ADC;
}

volatile bool startButtonPressed = false;
volatile bool stopButtonPressed = false;
volatile bool resetButtonPressed = false;

void handleStartButtonInterrupt() {
    startButtonPressed = true;
}

void handleStopButtonInterrupt() {
    stopButtonPressed = true;
}

void handleResetButtonInterrupt() {
    resetButtonPressed = true;
}

void setup() {
    // Configure LEDs
    GPIO_SET_OUTPUT(YELLOW_LED_PORT, YELLOW_LED_BIT);
    GPIO_SET_OUTPUT(GREEN_LED_PORT, GREEN_LED_BIT);
    GPIO_SET_OUTPUT(BLUE_LED_PORT, BLUE_LED_BIT);
    GPIO_SET_OUTPUT(RED_LED_PORT, RED_LED_BIT);
    
    // Configure input buttons, with pull-up resistors
    GPIO_WRITE_HIGH(START_BUTTON_PORT, START_BUTTON_BIT);
    attachInterrupt(digitalPinToInterrupt(19), handleStartButtonInterrupt, FALLING);

    GPIO_WRITE_HIGH(STOP_BUTTON_PORT, STOP_BUTTON_BIT);
    attachInterrupt(digitalPinToInterrupt(18), handleStopButtonInterrupt, FALLING);

    GPIO_WRITE_HIGH(RESET_BUTTON_PORT, RESET_BUTTON_BIT);
    attachInterrupt(digitalPinToInterrupt(2), handleResetButtonInterrupt, FALLING);

    GPIO_SET_OUTPUT(WATER_SENSOR_POWER_PORT, WATER_SENSOR_POWER_BIT);
    GPIO_WRITE_LOW(WATER_SENSOR_POWER_PORT, WATER_SENSOR_POWER_BIT); // Ensure sensor is initialized off

    // Configure fan pin
    GPIO_SET_OUTPUT(FAN_PORT, FAN_BIT);

    lcd.begin(16, 2);
    dht.begin();

    if (!rtc.begin()) {
        lcd.clear();
        lcd.print("RTC ERROR");
        while (1); // Halt execution if RTC fails
    }
    if (!rtc.isrunning()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set RTC to compile time
    }

    uartInit(9600);
    adcInit();
}

void loop() {

    switch (currentState) {
        case DISABLED:
            handleDisabledState();
            break;
        case IDLE:
            handleIdleState();
            break;
        case RUNNING:
            handleRunningState();
            break;
        case ERROR:
            handleErrorState();
            break;
    }
}

void simulateShittyDHT() {
    // Toggle currentTemp between values above and below setpoint to simulate
    // state transition
    if (currentTemp == 0.0) {
        currentTemp = 30.0;
    } else currentTemp = 0.0;
}

void handleDisabledState() {
    if (startButtonPressed) {
        startButtonPressed = false; // Reset the flag
        currentState = IDLE;        // Transition to IDLE state
        lcd.clear();
        logStateTransition("DISABLED", "IDLE");
        return;
    }

    turnOnLED(YELLOW);
    lcd.setCursor(0, 0);
    lcd.print("System Disabled");
}

void handleIdleState() {
    if (stopButtonPressed) {
        stopButtonPressed = false;  // Reset the flag
        currentState = DISABLED;    // Transition to DISABLED state
        lcd.clear();
        logStateTransition("IDLE", "DISABLED");
        return;
    }

    turnOnLED(GREEN);
    readSensors();

    if (waterLevelLow) {
        currentState = ERROR;
        lcd.clear();
        logStateTransition("IDLE", "ERROR");
        return;
    }

    if (currentTemp > 23.0) {
        currentState = RUNNING;
        lcd.clear();
        logStateTransition("IDLE", "RUNNING");
        return;
    }

    displayTempHumidity();
    adjustVent();
}

void handleRunningState() {
    if (stopButtonPressed) {
        stopButtonPressed = false;  // Reset the flag
        currentState = DISABLED;    // Transition to DISABLED state
        GPIO_WRITE_LOW(FAN_PORT, FAN_BIT);
        lcd.clear();
        logStateTransition("RUNNING", "DISABLED");
        return;
    }

    turnOnLED(BLUE);
    readSensors();

    if (currentTemp < 21.0) {
        currentState = IDLE;
        GPIO_WRITE_LOW(FAN_PORT, FAN_BIT);
        lcd.clear();
        logStateTransition("RUNNING", "IDLE");
        return;
    }

    if (waterLevelLow) {
        currentState = ERROR;
        GPIO_WRITE_LOW(FAN_PORT, FAN_BIT);
        lcd.clear();
        logStateTransition("RUNNING", "ERROR");
        return;
    }

    GPIO_WRITE_HIGH(FAN_PORT, FAN_BIT);
    displayTempHumidity();
    adjustVent();
}

void handleErrorState() {
    if (resetButtonPressed) {
        resetButtonPressed = false; // Reset the flag
        readWaterSensor();

        if (!waterLevelLow) {
            currentState = IDLE;    // Transition to IDLE state
            lcd.clear();
            logStateTransition("ERROR", "IDLE");
            return;
        }
    }

    turnOnLED(RED);
    lcd.setCursor(0, 0);
    lcd.print("Error: Water Low");
}

void readSensors() {
    if (millis() - lastUpdateTime >= updateInterval) {
//        currentTemp = dht.readTemperature();
        // DHT sensor is shitty and does not work
        simulateShittyDHT();

        currentHumidity = dht.readHumidity();
        readWaterSensor();
        lastUpdateTime = millis();
        lcd.clear();
        
    }
}

void readWaterSensor() {
    GPIO_WRITE_HIGH(WATER_SENSOR_POWER_PORT, WATER_SENSOR_POWER_BIT);
    _delay_ms(50);
    uint16_t adcValue = adcRead(0);
    waterLevelLow = adcValue < 500;
    GPIO_WRITE_LOW(WATER_SENSOR_POWER_PORT, WATER_SENSOR_POWER_BIT);
}

void adjustVent() {
    static unsigned long lastStepTime = 0; // Track the last time a step was issued
    uint16_t potValue = adcRead(1); // Read from ADC channel 1 (Potentiometer)

    // Define midpoint and dead zone
    const uint16_t midpoint = 512;
    const uint16_t deadZone = 100;

    // Check for the dead zone
    if (potValue > (midpoint - deadZone) && potValue < (midpoint + deadZone)) {
        return; // No movement within the dead zone
    }

    // Only step if enough time has passed
    if (millis() - lastStepTime >= stepInterval) {
        // Calculate direction
        int direction = (potValue < midpoint) ? 1 : -1;

        ventStepper.setSpeed(10);
        ventStepper.step(direction * 10); // Move a small step in the chosen direction

        lastStepTime = millis(); // Update the last step time
    }
}

void displayTempHumidity() {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(currentTemp);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum:  ");
    lcd.print(currentHumidity);
    lcd.print("%");
}

void logTimestamp() {
    DateTime now = rtc.now();
    char buffer[20];
    sprintf(buffer, "[%02d:%02d:%02d] ", now.hour(), now.minute(), now.second());
    uartPrint(buffer);
}

void turnOnLED(Color color) {
    GPIO_WRITE_LOW(RED_LED_PORT, RED_LED_BIT);
    GPIO_WRITE_LOW(GREEN_LED_PORT, GREEN_LED_BIT);
    GPIO_WRITE_LOW(BLUE_LED_PORT, BLUE_LED_BIT);
    GPIO_WRITE_LOW(YELLOW_LED_PORT, YELLOW_LED_BIT);

    switch (color) {
        case RED:
            GPIO_WRITE_HIGH(RED_LED_PORT, RED_LED_BIT);
            break;
        case GREEN:
            GPIO_WRITE_HIGH(GREEN_LED_PORT, GREEN_LED_BIT);
            break;
        case BLUE:
            GPIO_WRITE_HIGH(BLUE_LED_PORT, BLUE_LED_BIT);
            break;
        case YELLOW:
            GPIO_WRITE_HIGH(YELLOW_LED_PORT, YELLOW_LED_BIT);
            break;
    }
}

void logStateTransition(const char* fromState, const char* toState) {
    logTimestamp(); // Add timestamp
    uartPrint("Transition from ");
    uartPrint(fromState);
    uartPrint(" to ");
    uartPrint(toState);
    uartPrint("\n");
}
