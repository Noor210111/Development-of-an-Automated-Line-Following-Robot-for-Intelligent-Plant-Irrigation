#include <Servo.h>

// Pin Definitions
#define ULTRASONIC_TRIG_1 2
#define ULTRASONIC_ECHO_1 3
#define ULTRASONIC_TRIG_2 4
#define ULTRASONIC_ECHO_2 5
#define IR_SENSOR_1 A0
#define IR_SENSOR_2 A1
#define MOISTURE_SENSOR A2
#define RELAY_PIN 6
#define SERVO_PIN 7
#define MOTOR_DRIVER_IN1 8
#define MOTOR_DRIVER_IN2 9
#define MOTOR_DRIVER_IN3 10
#define MOTOR_DRIVER_IN4 11

// Servo Object
Servo servo;

// Variables
int moistureThreshold = 500;
long duration, distance1, distance2;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize Ultrasonic Sensors
  pinMode(ULTRASONIC_TRIG_1, OUTPUT);
  pinMode(ULTRASONIC_ECHO_1, INPUT);
  pinMode(ULTRASONIC_TRIG_2, OUTPUT);
  pinMode(ULTRASONIC_ECHO_2, INPUT);

  // Initialize IR Sensors
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);

  // Initialize Moisture Sensor
  pinMode(MOISTURE_SENSOR, INPUT);

  // Initialize Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Initialize Servo
  servo.attach(SERVO_PIN);
  servo.write(0);

  // Initialize Motor Driver Pins
  pinMode(MOTOR_DRIVER_IN1, OUTPUT);
  pinMode(MOTOR_DRIVER_IN2, OUTPUT);
  pinMode(MOTOR_DRIVER_IN3, OUTPUT);
  pinMode(MOTOR_DRIVER_IN4, OUTPUT);
}

void loop() {
  // Read Ultrasonic Sensor 1
  distance1 = readUltrasonic(ULTRASONIC_TRIG_1, ULTRASONIC_ECHO_1);
  Serial.print("Distance 1: ");
  Serial.println(distance1);

  // Read Ultrasonic Sensor 2
  distance2 = readUltrasonic(ULTRASONIC_TRIG_2, ULTRASONIC_ECHO_2);
  Serial.print("Distance 2: ");
  Serial.println(distance2);

  // Read IR Sensors
  int ir1 = digitalRead(IR_SENSOR_1);
  int ir2 = digitalRead(IR_SENSOR_2);
  Serial.print("IR Sensor 1: ");
  Serial.println(ir1);
  Serial.print("IR Sensor 2: ");
  Serial.println(ir2);

  // Read Moisture Sensor
  int moisture = analogRead(MOISTURE_SENSOR);
  Serial.print("Moisture: ");
  Serial.println(moisture);

  // Check Soil Moisture
  if (moisture < moistureThreshold) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(5000);
    digitalWrite(RELAY_PIN, LOW);
  }

  // Control Servo
  if (distance1 < 10) {
    servo.write(90); 
    delay(1000);
    servo.write(0); 
  }

  // Motor Control
  if (distance1 < 10 || distance2 < 10) {
    stopMotors();
    delay(1000);
    moveBackward();
    delay(1000);
    turnRight();
    delay(1000);
  } else {
    moveForward();
  }

  delay(100);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(MOTOR_DRIVER_IN1, HIGH);
  digitalWrite(MOTOR_DRIVER_IN2, LOW);
  digitalWrite(MOTOR_DRIVER_IN3, HIGH);
  digitalWrite(MOTOR_DRIVER_IN4, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_DRIVER_IN1, LOW);
  digitalWrite(MOTOR_DRIVER_IN2, HIGH);
  digitalWrite(MOTOR_DRIVER_IN3, LOW);
  digitalWrite(MOTOR_DRIVER_IN4, HIGH);
}

void turnRight() {
  digitalWrite(MOTOR_DRIVER_IN1, HIGH);
  digitalWrite(MOTOR_DRIVER_IN2, LOW);
  digitalWrite(MOTOR_DRIVER_IN3, LOW);
  digitalWrite(MOTOR_DRIVER_IN4, HIGH);
}

void stopMotors() {
  digitalWrite(MOTOR_DRIVER_IN1, LOW);
  digitalWrite(MOTOR_DRIVER_IN2, LOW);
  digitalWrite(MOTOR_DRIVER_IN3, LOW);
  digitalWrite(MOTOR_DRIVER_IN4, LOW);
}
