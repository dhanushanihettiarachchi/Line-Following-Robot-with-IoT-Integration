// Motor pin definitions
int enR = 25;
int in1R = 26;
int in2R = 27;

int enL = 13;
int in1L = 12;
int in2L = 14;

// IR sensor pin definitions
int irLeft = 22;
int irCenter = 19;
int irRight = 23;

// Speed settings
int baseSpeed = 90;
int maxSpeed = 120;

// PID constants
float Kp = 25.0;
float Ki = 0.0;
float Kd = 10.0;

// PID variables
float error = 0;
float previousError = 0;
float integral = 0;

void setup(){
  Serial.begin(9600);

  // Setup motor pins
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(enR, OUTPUT);
  
  pinMode(in1L, OUTPUT);
  pinMode(in2L, OUTPUT);
  pinMode(enL, OUTPUT);

  // Setup IR sensor pins
  pinMode(irLeft, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight, INPUT);
}

void loop() {
  int left = digitalRead(irLeft);
  int center = digitalRead(irCenter);
  int right = digitalRead(irRight);

  // Print sensor readings
  Serial.print("Left: "); Serial.print(left == HIGH ? "Black" : "White");
  Serial.print(" | Center: "); Serial.print(center == HIGH ? "Black" : "White");
  Serial.print(" | Right: "); Serial.println(right == HIGH ? "Black" : "White");

  // Stop if all sensors see white (line lost)
  if (left == LOW && center == LOW && right == LOW) {
    stopMotors();
    return;
  }

  // Calculate PID error
  error = calculateError(left, center, right);
  integral += error;
  float derivative = error - previousError;

  float correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  int leftSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
  delay(10);
}

float calculateError(int left, int center, int right) {
  // Assign error values based on sensor readings
  if (left == HIGH && center == LOW && right == LOW) return 1;     // line on left
  if (left == LOW && center == HIGH && right == LOW) return 0;     // line in center
  if (left == LOW && center == LOW && right == HIGH) return -1;    // line on right
  if (left == HIGH && center == HIGH && right == LOW) return 0.5;
  if (left == LOW && center == HIGH && right == HIGH) return -0.5;
  if (left == HIGH && center == LOW && right == HIGH) return 0;    // line balanced
  return 0; // fallback
}

void setMotor(int leftSpeed, int rightSpeed) {
  // Right motor forward
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH);
  analogWrite(enR, rightSpeed);

  // Left motor forward
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  analogWrite(enL, leftSpeed);
}

void stopMotors() {
  digitalWrite(in1R, LOW); digitalWrite(in2R, LOW);
  digitalWrite(in1L, LOW); digitalWrite(in2L, LOW);
  analogWrite(enR, 0); analogWrite(enL,0);
}