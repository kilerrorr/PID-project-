
#include <Servo.h>
Servo servo;

#define trig 2
#define echo 3

#define kp 59.2 //needs tuning 3ala hasb sor3et balance 
//#define kd 31.455
#define kd 14.19 //needs tuning 3ala hasb sor3et balance 

// Compensator parameters
#define Kc 1.0     // Compensator gain
#define T 0.02     // Zero location (seconds)
#define B 476.19      // Lag factor (<1 for lag)

// Compensator state variables
double prevPD = 0;
double prevComp = 0;

double priError = 0;


void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW); // ensure idle low
  servo.attach(5);
  Serial.begin(9600);
  servo.write(50);

}
void loop() {
//    int a = distance();
//    Serial.println(a);
  static unsigned long lastMeasMs = 0;
  static double lastDistance = 0;

  // sample distance at most every 60 ms
  if (millis() - lastMeasMs >= 60) {
    lastDistance = distance();
    lastMeasMs = millis();
    Serial.println(lastDistance, 2);
}

PD_Lag(lastDistance);
}

double distance () {
  // Trigger 10 us pulse, ensure clean low before/after
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Add timeout (e.g., 30 ms) to avoid blocking
  unsigned long t = pulseIn(echo, HIGH, 30000UL);
  if (t == 0) {
    // No echo within timeout: return last valid or a safe default
    return -1; // indicate invalid
  }
  double cm = t / 29 / 2; // HC-SR04 conversion
  // Clamp to reasonable range
  if (cm < 0) cm = 0;
  if (cm > 400) cm = 400;
  return cm;
}


void PD_Lag(double dis) {
  if (dis < 0.0) {
    // skip control update on invalid echo
    return;
  }
  
  int setP = 15; //needs tuning

  double error = dis - setP;

  // PD controller
  double Pvalue = error * kp;
  double Dvalue = (error - priError) * kd;
  double PDvalue = Pvalue + Dvalue;
  priError = error;

  // Lag compensator using Kc, T, B
  // Lag filter: compValue = alpha*prevComp + (1-alpha)*PDvalue
  double alpha = 0.9; // adjust between 0.7–0.95
  double compValue = alpha * prevComp + (1 - alpha) * PDvalue;
  prevComp = compValue;


  // Update states
  prevPD   = PDvalue;
  prevComp = compValue;

  // Servo mapping
  // Double-precision linear map
  auto linearMap = [](double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  };

  // Map PD+lag output to servo angle range [135 -> 0], then constrain to [40, 170]
  double angle = linearMap(compValue, -180, 180.0, 0.0, 180.0);
  if (angle < 70) angle =70;
  if (angle > 110) angle = 110.0;
  servo.write((int)angle);

}