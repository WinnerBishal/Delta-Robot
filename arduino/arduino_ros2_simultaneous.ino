int st1 = 2;
int dir1 = 3;
int st2 = 4;
int dir2 = 5;
int st3 = 6;
int dir3 = 7;

// Constants
#define STEPS_PER_REV 1600
#define VacuumSwitchPin 9
#define MaxStepperSpeed 1000
#define MaxStepperAcc 500
#define PulseWidth 4000

// CURRENT ANGLE POSITIONS
int currentAngleM1 = 0;
int currentAngleM2 = 0;
int currentAngleM3 = 0;

// NEW ANGLE POSITIONS
int newAngleM1 = 0;
int newAngleM2 = 0;
int newAngleM3 = 0;

int gripperState = 0;


// Function Declarations
void run_three_steppers(int steps1, int steps2, int steps3, int delay);
void controlGripper(int state);
int angleToSteps(int angle);
int getCommandValue(String command, String prefix);
void controlGripper(int state);

void setup() {
  Serial.begin(38400);

  pinMode(VacuumSwitchPin, OUTPUT);

  delay(1000);  // Stabilization delay
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    // Calibration command(HOMING)
    if (command.startsWith("C")) {
      int motorToCalibrate = command.charAt(1) - '0';  // Convert char to int

      switch (motorToCalibrate) {
        case 4:
          currentAngleM1 = 0;
          break;
        case 5:
          currentAngleM2 = 0;
          break;
        case 6:
          currentAngleM3 = 0;
          break;
      }
    }
    // Movement or gripper command
    else {
      if (command.indexOf("M1:") != -1) {
        newAngleM1 = getCommandValue(command, "M1:");
      }
      if (command.indexOf("M2:") != -1) {
        newAngleM2 = getCommandValue(command, "M2:");
      }
      if (command.indexOf("M3:") != -1) {
        newAngleM3 = getCommandValue(command, "M3:");
      }
      if (command.indexOf("G:") != -1) {
        gripperState = getCommandValue(command, "G:");
      }
    }
    int m1_steps = angleToSteps(newAngleM1 - currentAngleM1);
    int m2_steps = angleToSteps(newAngleM2 - currentAngleM2);
    int m3_steps = angleToSteps(newAngleM3 - currentAngleM3);

    run_three_steppers(m1_steps, m2_steps, m3_steps, PulseWidth);
    controlGripper(gripperState);
    
    currentAngleM1 = newAngleM1;
    currentAngleM2 = newAngleM2;
    currentAngleM3 = newAngleM3;
  }
}


void controlGripper(int state) {
  if (state) {
    digitalWrite(VacuumSwitchPin, HIGH);
  } else {
    digitalWrite(VacuumSwitchPin, LOW);
  }
}

int angleToSteps(int angle) {
  return (int)((angle / 360.0) * STEPS_PER_REV);
}

int getCommandValue(String command, String prefix) {
  int prefixIndex = command.indexOf(prefix) + prefix.length();
  int endIndex = command.indexOf(';', prefixIndex);
  endIndex = (endIndex == -1) ? command.length() : endIndex;  // Handle last command without semicolon
  return command.substring(prefixIndex, endIndex).toInt();
}

void run_three_steppers(int steps1, int steps2, int steps3, int delay)
{

  int stepperDelay = delay;

  if (steps1 < 0)
  {
    digitalWrite(dir1, LOW);
  }
  else
  {
    digitalWrite(dir1, HIGH);
  }
  if (steps2 < 0)
  {
    digitalWrite(dir2, LOW);
  }
  else
  {
    digitalWrite(dir2, HIGH);
  }
  if (steps3 < 0)
  {
    digitalWrite(dir3, LOW);
  }
  else
  {
    digitalWrite(dir3, HIGH);
  }

  for (int x = 0; x < max(max(abs(steps1), abs(steps2)), abs(steps3)); x++)
  {
    if (x < abs(steps1))
    {
      digitalWrite(st1, HIGH);
      delayMicroseconds(delay);
      digitalWrite(st1, LOW);
      delayMicroseconds(delay);
    }
    if (x < abs(steps2))
    {
      digitalWrite(st2, HIGH);
      delayMicroseconds(delay);
      digitalWrite(st2, LOW);
      delayMicroseconds(delay);
    }
    if (x < abs(steps3))
    {
      digitalWrite(st3, HIGH);
      delayMicroseconds(delay);
      digitalWrite(st3, LOW);
      delayMicroseconds(delay);
    }
  }
}