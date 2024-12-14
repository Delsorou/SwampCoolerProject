// Swamp Cooler Project Code
// Restrictions: Direct register manipulation, only allowed Arduino libraries for specific components

#include <Stepper.h>
#include <Wire.h>       // For RTC
#include <DHT.h>        // For temp/humidity sensor
#include <RTClib.h>     // For RTC module

// Pin Definitions
#define RED_LED_PIN 2
#define GREEN_LED_PIN 3
#define BLUE_LED_PIN 4
#define YELLOW_LED_PIN 5
#define START_BUTTON_PIN 6
#define RESET_BUTTON_PIN 7
#define WATER_SENSOR_PIN A0
#define FAN_PIN 8
#define STEPPER_IN1 9
#define STEPPER_IN2 10
#define STEPPER_IN3 11
#define STEPPER_IN4 12
#define TEMP_HUMID_PIN 13

// Stepper motor setup
const int stepsPerRevolution = 200;
Stepper ventStepper(stepsPerRevolution, STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);

// DHT sensor setup
#define DHTTYPE DHT11
DHT dht(TEMP_HUMID_PIN, DHTTYPE);

// RTC setup
RTC_DS3231 rtc;

// System States
enum State { DISABLED, IDLE, RUNNING, ERROR };
State currentState = DISABLED;

// Variables for state management
float currentTemp = 0;
float currentHumidity = 0;
bool waterLevelLow = false;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 60000; // 1 minute

void setup() {
    // Configure LEDs and buttons as output/input using direct register manipulation
    DDRD |= (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN) | (1 << YELLOW_LED_PIN); // LEDs as output
    DDRD &= ~((1 << START_BUTTON_PIN) | (1 << RESET_BUTTON_PIN)); // Buttons as input
    PORTD |= (1 << START_BUTTON_PIN) | (1 << RESET_BUTTON_PIN);   // Enable pull-up resistors for buttons

    // Initialize allowed components
    ventStepper.setSpeed(60);
    dht.begin();
    if (!rtc.begin()) {
        currentState = ERROR;
    }
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

void handleDisabledState() {
    turnOnLED(YELLOW_LED_PIN);
    if (isButtonPressed(START_BUTTON_PIN)) {
        currentState = IDLE;
        logStateTransition("DISABLED", "IDLE");
    }
}

void handleIdleState() {
    turnOnLED(GREEN_LED_PIN);
    readSensors();

    if (waterLevelLow) {
        currentState = ERROR;
        logStateTransition("IDLE", "ERROR");
        return;
    }

    if (currentTemp > 30.0) { // Replace with your temperature threshold
        currentState = RUNNING;
        logStateTransition("IDLE", "RUNNING");
        return;
    }

    if (millis() - lastUpdateTime >= updateInterval) {
        displayTempHumidity();
        lastUpdateTime = millis();
    }
}

void handleRunningState() {
    turnOnLED(BLUE_LED_PIN);
    digitalWrite(FAN_PIN, HIGH); // Turn on fan
    readSensors();

    if (currentTemp < 25.0) { // Replace with your temperature threshold
        digitalWrite(FAN_PIN, LOW); // Turn off fan
        currentState = IDLE;
        logStateTransition("RUNNING", "IDLE");
        return;
    }

    if (waterLevelLow) {
        digitalWrite(FAN_PIN, LOW); // Turn off fan
        currentState = ERROR;
        logStateTransition("RUNNING", "ERROR");
        return;
    }

    if (millis() - lastUpdateTime >= updateInterval) {
        displayTempHumidity();
        lastUpdateTime = millis();
    }
}

void handleErrorState() {
    turnOnLED(RED_LED_PIN);
    if (isButtonPressed(RESET_BUTTON_PIN)) {
        readWaterSensor();
        if (!waterLevelLow) {
            currentState = IDLE;
            logStateTransition("ERROR", "IDLE");
        }
    }
}

void readSensors() {
    currentTemp = dht.readTemperature();
    currentHumidity = dht.readHumidity();
    readWaterSensor();
}

void readWaterSensor() {
    waterLevelLow = analogRead(WATER_SENSOR_PIN) < 500; // Adjust threshold as needed
}

void displayTempHumidity() {
    Serial.print("Temp: ");
    Serial.print(currentTemp);
    Serial.print("C, Humidity: ");
    Serial.print(currentHumidity);
    Serial.println("%");
}

void logStateTransition(const char* fromState, const char* toState) {
    DateTime now = rtc.now();
    Serial.print(now.timestamp());
    Serial.print(": Transition from ");
    Serial.print(fromState);
    Serial.print(" to ");
    Serial.println(toState);
}

bool isButtonPressed(int pin) {
    return !(PIND & (1 << pin));
}

void turnOnLED(int ledPin) {
    PORTD &= ~((1 << RED_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN) | (1 << YELLOW_LED_PIN)); // Turn off all LEDs
    PORTD |= (1 << ledPin); // Turn on the specified LED
}
