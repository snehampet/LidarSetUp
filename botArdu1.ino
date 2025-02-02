#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <AccelStepper.h>

// Define stepper motor pins
#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define MOTOR_PIN3 10
#define MOTOR_PIN4 11

// Stepper motor settings
const int stepsPerRevolution = 2048;  // Full 360° rotation
const float degreesPerStep = 360.0 / stepsPerRevolution;  // Degrees per step

// Create motor and sensor objects
AccelStepper stepper(AccelStepper::FULL4WIRE, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

unsigned long previousMillis = 0;
const long interval = 200;  // Sensor reading every 200ms
bool clockwise = true;
int errorCount = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
 
  delay(500);
  if (!sensor.begin()) {
    Serial.println("VL53L0X initialization failed!");
    while (1);
  }
  Serial.println("VL53L0X initialized.");
  sensor.startRangeContinuous();  
  
  stepper.setMaxSpeed(300.0);  
  stepper.setAcceleration(100.0);
  stepper.moveTo(stepsPerRevolution);  // Move clockwise first
}

void loop() {
  if (stepper.distanceToGo() == 0) {
    // Change direction when reaching either end
    clockwise = !clockwise;
    if (clockwise) {
      stepper.moveTo(stepper.currentPosition() + stepsPerRevolution);
    } else {
      stepper.moveTo(stepper.currentPosition() - stepsPerRevolution);
    }
  }
  
  stepper.run();  // Keep motor moving

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    
    // Calculate current angle (0-360)
    float currentAngle = (stepper.currentPosition() % stepsPerRevolution) * degreesPerStep;
    if (currentAngle < 0) currentAngle += 360.0;
    
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter != 8191) {
      Serial.print("Angle: ");
      Serial.print(currentAngle, 1);
      Serial.print("°, Distance: ");
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
      errorCount = 0;
    } else {
      Serial.print("Angle: ");
      Serial.print(currentAngle, 1);
      Serial.println("°, Sensor ERROR!");
    }
  }
}