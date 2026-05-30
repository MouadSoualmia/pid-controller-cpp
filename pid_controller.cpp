/*  7-IR Line Follower – No Motor Library (updated + sharp turn while) */

#define AIN1 3   // left motor
#define AIN2 4
#define BIN1 7   // right motor
#define BIN2 8
#define PWMA 5
#define PWMB 6

#define NUM_SENSORS 7

// A1 = rightmost, A4 = center, A7 = leftmost
int sensorPin[NUM_SENSORS] = {A1, A2, A3, A4, A5, A6, A7};
int weight[NUM_SENSORS]    = { 5, 4, 3, 0, -3, -4, -5};

int minVal[NUM_SENSORS];
int maxVal[NUM_SENSORS];
int threshold[NUM_SENSORS];

// PID
float Kp = 28;
float Ki = 0.02;
float Kd = 39;

int P, D;
int I = 0;
int lastError = 0;

// SPEED LIMITS
int minSpeed = 120;
int maxSpeed = 200;

// ---------------- MOTOR CONTROL ----------------
void setMotorA(int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  }
}

void setMotorB(int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
}

void stopMotors()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ---------------- SENSOR READER (NEW) ----------------
void readSensors(int sensorState[])
{
  int rawState[NUM_SENSORS];

  int blackCount = 0;
  int whiteCount = 0;

  // Read raw sensors
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    rawState[i] = (analogRead(sensorPin[i]) > threshold[i]);

    if (rawState[i]) blackCount++;
    else whiteCount++;
  }

  // Decide line color
  bool followBlack;
  if (blackCount < whiteCount)
    followBlack = true;
  else
    followBlack = false;

  // Build sensorState[]
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (followBlack)
      sensorState[i] = rawState[i];
    else
      sensorState[i] = !rawState[i];
  }
}

// ---------------- CALIBRATION ----------------
void calibrateSensors(unsigned long duration)
{
  unsigned long startTime = millis();

  while (millis() - startTime < duration)
  {
    setMotorA(80);
    setMotorB(-80);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      int val = analogRead(sensorPin[i]);
      minVal[i] = min(minVal[i], val);
      maxVal[i] = max(maxVal[i], val);
    }
  }

  stopMotors();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    threshold[i] = (minVal[i] + maxVal[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();
}

// ---------------- SETUP ----------------
void setup()
{
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  delay(3000);
  calibrateSensors(3500);
  delay(2000);
}

// ---------------- LINE FOLLOW ----------------
void lineFollow()
{
  int sensorState[NUM_SENSORS];

  // Read sensors using the new function
  readSensors(sensorState);


  // -------- PID ERROR --------
  int error = 0;
  int activeSensors = 0;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorState[i])
    {
      error += weight[i];
      activeSensors++;
    }
  }

  if (activeSensors == 0)
    error = lastError;

  P = error;
  I += error;
  I = constrain(I, -50, 50);
  D = error - lastError;

  // -------- ADAPTIVE SPEED --------
  int absError = abs(error);
  int absD = abs(D);

  int dynamicBaseSpeed;

  if (absError <= 1 && absD <= 1)
    dynamicBaseSpeed = maxSpeed;
  else if (absError <= 3)
    dynamicBaseSpeed = (maxSpeed + minSpeed) / 2;
  else
    dynamicBaseSpeed = minSpeed;

  int PID = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;

  int leftSpeed  = dynamicBaseSpeed - PID;
  int rightSpeed = dynamicBaseSpeed + PID;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotorA(leftSpeed);
  setMotorB(rightSpeed);
 
   // -------- INTERSECTION FORWARD PRIORITY --------
if (sensorState[3] && (sensorState[0] || sensorState[6]))
{
  // Middle sensor sees the line AND a side sensor sees line → intersection
  // Just move forward with normal speed
  setMotorA(maxSpeed);
  setMotorB(maxSpeed);
  lastError = 0;  // reset error
  return;
}

  // -------- SHARP TURN RIGHT -------
  
  if (sensorState[6] && !sensorState[5])
  {
    unsigned long startTime = millis();

    while (true)
    {
      readSensors(sensorState);

      setMotorA(150);
      setMotorB(-100);

      if (sensorState[3]) break;               // middle sensor sees line
      if (millis() - startTime > 700) break;   // timeout
    }

    lastError = -3;
    return;
  }

  // -------- SHARP TURN LEFT --------
  if (sensorState[0] && !sensorState[1])
  {
    unsigned long startTime = millis();

    while (true)
    {
      readSensors(sensorState);

      setMotorA(-100);
      setMotorB(150);

      if (sensorState[3]) break;
      if (millis() - startTime > 700) break;
    }

    lastError = 3;
    return;
  }

  // -------- EXTRA RECOVERY (optional) --------
  if (sensorState[0] && sensorState[1] && !sensorState[3])
  {
    setMotorA(150);
    setMotorB(-150);
    lastError = 3;
    return;
  }

  if (sensorState[6] && sensorState[5] && !sensorState[3])
  {
    setMotorA(-150);
    setMotorB(150);
    lastError = -3;
    return;
  }
}

// ---------------- LOOP ----------------
void loop()
{
  lineFollow();
}