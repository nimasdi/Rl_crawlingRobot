// Install the required libraries via the Library Manager in the Arduino IDE:
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>

// Pin definitions
const int triggerPin = 5;
const int echoPin = 17;
const int control_servo_down = 32;
const int control_servo_up = 33;
#define MAX_DISTANCE 200 // Maximum distance to check in cm

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servodown;
Servo servoup;
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE); // NewPing instance


float getDistance()
{
    float distance = sonar.ping_cm(); // Get distance in cm
    if (distance == 0)
    {
        Serial.println("Warning: No echo received from SRF module.");
        return -1;
    }
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
}

void healthCheck()
{
    // 1. Print Health Check on LCD
    lcd.clear();
    lcd.print("Health Check Start");
    delay(1000);

    // 2. Get distance and print it
    float distance = getDistance();
    lcd.clear();
    if (distance < 0)
    {
        lcd.print("SRF Error!");
    }
    else
    {
        lcd.print("Dist: ");
        lcd.print(distance);
        lcd.print(" cm");
    }
    delay(500);

    servodown.write(90);
    servoup.write(90);
    delay(1000);
    // 3. Move servos back to initial position
    servodown.write(0);
    servoup.write(180);
    lcd.clear();
    lcd.print("Servos Reset");
    delay(1000);

    // 4. Print completion message
    lcd.clear();
    lcd.print("Health Check Done");
    delay(1500);
    lcd.clear();
}

void moveServoSmooth(Servo &servo, int from, int to, int stepDelay = 10)
{
    if (from < to)
    {
        for (int a = from; a <= to; a += 2)
        {
            servo.write(a);
            delay(stepDelay);
        }
    }
    else
    {
        for (int a = from; a >= to; a -= 2)
        {
            servo.write(a);
            delay(stepDelay);
        }
    }
}

void doTraining()
{
    // TODO: Add your training logic here
}

void doLearnedBehavior()
{
    // TODO: Add your learned behavior logic here
}


void setup()
{
    Serial.begin(9600);

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("RL Robot Setup");
    delay(1000);

    // Attach servos with min/max pulse widths
    servodown.attach(control_servo_down, 600, 2400);
    servoup.attach(control_servo_up, 600, 2400);

    // Set servos to initial position
    servodown.write(0);
    servoup.write(180);

    // Notify setup completion
    lcd.clear();
    lcd.print("Setup Completed");
    delay(2000);
    healthCheck();
}

void loop()
{
    lcd.clear();
    lcd.print("Main Loop Running");
    delay(10000);
    // TODO: Implement your main loop logic here
}