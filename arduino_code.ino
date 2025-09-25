#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int triggerPins[4] = {6, 4, 2, 11};
const int echoPins[4] = {7, 5, 3, 12};

const int dataLSB = 8;
const int dataMID = 9;
const int dataReady = 10;

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();
const int servoChannels[4] = {9, 12, 15, 0}; // Servo 0 is LCD flag

const int pulseStop = 1500;
const int pulseCW = 1350; //1350
const int pulseCCW = 1580; //1600

const int pulseCW0 = 1150; // Servo 0 CW (LCD flag ON) 1200
const int pulseCCW0 = 1930; // Servo 0 CCW (LCD flag OFF) 1930

const unsigned long rotationTime = 741;
const unsigned long pauseTime = 1500;

const int QUEUE_SIZE = 100;
int taskQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

int lastLSB = -1;
int lastMID = -1;

// --- NEW SENSOR DEFINITIONS ---
const int counterTriggerPin = A0;
const int counterEchoPin = A1;
const int counterOutputPin = A2;
int consecutiveValidReadings = 0;
unsigned long previousCounterMillis = 0;
const long counterInterval = 500; // 0.5 second
// --- END NEW SENSOR DEFINITIONS ---

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    pinMode(triggerPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  pinMode(dataLSB, INPUT);
  pinMode(dataMID, INPUT);
  pinMode(dataReady, INPUT);
  pinMode(13, OUTPUT);

  // New sensor pin modes
  pinMode(counterTriggerPin, OUTPUT);
  pinMode(counterEchoPin, INPUT);
  pinMode(counterOutputPin, OUTPUT);
  digitalWrite(counterOutputPin, LOW);

  srituhobby.begin();
  srituhobby.setPWMFreq(80);

  queueHead = 0;
  queueTail = 0;
}

bool isQueueEmpty() {
  return queueHead == queueTail;
}

bool enqueueTask(int sensorIndex) {
  int nextTail = (queueTail + 1) % QUEUE_SIZE;
  if (nextTail == queueHead) {
    Serial.println("Queue full! Cannot enqueue.");
    return false;
  }
  taskQueue[queueTail] = sensorIndex;
  queueTail = nextTail;
  return true;
}

int dequeueTask() {
  if (isQueueEmpty()) return -1;
  int task = taskQueue[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  return task;
}

void pollCounterSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousCounterMillis >= counterInterval) {
    previousCounterMillis = currentMillis;

    digitalWrite(counterTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(counterTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(counterTriggerPin, LOW);

    long duration = pulseIn(counterEchoPin, HIGH, 30000);
    float distance = duration * 0.0343 / 2;

    if (distance > 0 && distance < 14.0) {
      consecutiveValidReadings++;
      if (consecutiveValidReadings >= 3) {
        digitalWrite(counterOutputPin, HIGH);
        Serial.println("A2 output: HIGH (3 consecutive valid readings)");
      } else {
        digitalWrite(counterOutputPin, LOW);
        Serial.print("Consecutive valid readings: ");
        Serial.println(consecutiveValidReadings);
      }
    } else {
      consecutiveValidReadings = 0;
      digitalWrite(counterOutputPin, LOW);
      Serial.println("Object not detected or out of range. Counter reset.");
    }
  }
}

void loop() {
  pollInputs();
  pollCounterSensor();

  if (!isQueueEmpty()) {
    int currentTask = dequeueTask();
    runTask(currentTask);
  }
}

void pollInputs() {
  bool ready = digitalRead(dataReady);

  if (ready == HIGH) {
    int lsb = digitalRead(dataLSB);
    int mid = digitalRead(dataMID);

    if (lsb != lastLSB || mid != lastMID) {
      lastLSB = lsb;
      lastMID = mid;

      int sensorIndex = -1;
      String material = "Unknown";

      if (mid == 0 && lsb == 0) {
        material = "Others (100)";
        sensorIndex = 3;
      } else if (mid == 0 && lsb == 1) {
        material = "Aluminum (101)";
        sensorIndex = 0;
      } else if (mid == 1 && lsb == 0) {
        material = "Copper (110)";
        sensorIndex = 1;
      } else if (mid == 1 && lsb == 1) {
        material = "Steel (111)";
        sensorIndex = 2;
      }

      Serial.println("Received: " + material);

      srituhobby.writeMicroseconds(0, pulseCW0);
      delay(rotationTime);
      srituhobby.writeMicroseconds(0, pulseStop);
      Serial.println("Servo 0 rotated CW and stopped (LCD flag ON)");

      if (sensorIndex != -1) {
        enqueueTask(sensorIndex);
      }

      delay(50);
    }
  } else {
    lastLSB = -1;
    lastMID = -1;
  }
}

void runTask(int sensorIndex) {
  Serial.print("Running task for sensor ");
  Serial.println(sensorIndex + 1);

  float distance = -1;
  while (distance <= 0 || distance > 14.0) {
    pollInputs();
    distance = measureDistance(sensorIndex);
    delay(50);
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);

  if (sensorIndex != 3) {
    moveServoBlocking(servoChannels[sensorIndex]);
  } else {
    srituhobby.writeMicroseconds(0, pulseCCW0);
    delay(rotationTime);
    srituhobby.writeMicroseconds(0, pulseStop);
    Serial.println("Servo 0 rotated CCW and stopped for 'Others'");
  }

  Serial.println("Task complete.");
}

void moveServoBlocking(int channel) {
  unsigned long startTime;

  srituhobby.writeMicroseconds(channel, pulseCW);
  startTime = millis();
  while (millis() - startTime < rotationTime) pollInputs();

  srituhobby.writeMicroseconds(channel, pulseStop);
  startTime = millis();
  while (millis() - startTime < pauseTime) pollInputs();

  srituhobby.writeMicroseconds(channel, pulseCCW);

  if (channel == 9 || channel == 12 || channel == 15) {
    srituhobby.writeMicroseconds(0, pulseCCW0);
    delay(rotationTime);
    srituhobby.writeMicroseconds(0, pulseStop);
    Serial.println("Servo 0 rotated CCW and stopped (LCD flag OFF)");
  }

  startTime = millis();
  while (millis() - startTime < rotationTime) pollInputs();

  srituhobby.writeMicroseconds(channel, pulseStop);
}

float measureDistance(int i) {
  digitalWrite(triggerPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPins[i], LOW);

  long duration = pulseIn(echoPins[i], HIGH, 30000);
  if (duration == 0) return -1;

  return duration * 0.0343 / 2;
}