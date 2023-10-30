#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 10      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.2    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// Target Distance
#define _TARGET_LOW  180.0
#define _TARGET_HIGH 360.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
 
#define _DUTY_MIN 0 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1625 // servo neutral position (90 degree)
#define _DUTY_MAX 3150 // servo full counterclockwise position (180 degree)

// New global variables for the servo control
const int servoMinAngle = 0;   // 최소 각도 (0도)
const int servoMaxAngle = 180; // 최대 각도 (180도)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.write(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;
  
  // wait until the next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;           // Cut higher than the maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // cut lower than the minimum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In the desired range
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);       // LED ON      
  }

  // Apply EMA filter here  
  dist_ema = (1 - _EMA_ALPHA) * dist_ema + _EMA_ALPHA * dist_raw;

  // New servo control logic
if (dist_ema <= _TARGET_LOW) {
    myservo.write(_DUTY_MIN);
} else if (dist_ema >= _TARGET_HIGH) {
    myservo.write(_DUTY_MAX);
} else {
    int servoAngle;

    // Map the distance to the range 18-36cm to a 0-180 degree angle
    servoAngle = map(dist_ema, _TARGET_LOW, _TARGET_HIGH, 0, 180);
    myservo.write(servoAngle);
}

  // Output the distance to the serial port
  Serial.print("Min: ");    Serial.print(_DIST_MIN);
  Serial.print(", Low: ");   Serial.print(_TARGET_LOW);
  Serial.print(", Dist: ");  Serial.print(dist_raw);
  Serial.print(", Servo: "); Serial.print(myservo.read());  
  Serial.print(", High: ");  Serial.print(_TARGET_HIGH);
  Serial.print(", Max: ");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // Update the last sampling time
  last_sampling_time += INTERVAL;
}

// Get a distance reading from USS. Return value is in millimeters.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
