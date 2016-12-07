// main.ino
#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define irPin A0
#define zetaPin A1
#define wnPin A2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *mL = AFMS.getMotor(1);
Adafruit_DCMotor *mR = AFMS.getMotor(3);

// control parameters
float u = 0, u_1 = 0, u_2 = 0; // Current control and past control values
float e = 0, e_1 = 0, e_2 = 0; // Current error and past error values
float si = 0, si_1 = 0; // Raw sensor current and past value
float so = 0, so_1 = 0; // Filtered sensor current and past value
float setDist = 0.6; // Equilibrium Distance

// IR sensor parameters
float irReading, irV, invD; // IR Sensor reading, voltage, and inverse of distance
int dist; // IR Sensor distance in centimeters

// DC motor parameters
int nDcV; // Analalog Motor voltage (0 to 255)
float zeta = 0.707; // Desired Closed Loop Damping Ratio
float wn = 3; // Desired Closed Loop Natural Frequency
float wc = 21.0; // Filter Cut-off frequency
float M = 0.1728; // Plant Parameter
float C = 16.9778;  // Plant Parameter
float k, A, B, theta, alpha, beta, gamma, phi; // controller parameters
float T = 0.01; // Sampling period
float Ce_2, Ce_1, Ce, Cu_2, Cu_1; // Control Law coefficients
float Csi, Csi_1, Cso_1; // Filter coefficients

int zetaRead, wnRead; // Analog value from Pots (0 to 1023)
float zetaPose, wnPose; // Pot Positions (0 to 100%)

void setup() {
  Serial.begin(9600);

  /* Potentiometers are used to set the desired closed loop damping ratio 
  and natural frequency. Zeta: 0.4 to 1.0; Wn: 2 to 7 rad/sec */
  zetaRead = analogRead(zetaPin);
  wnRead = analogRead(wnPin);
  zetaPose = zetaRead / 1023.0;
  wnPose = wnRead / 1023.0;
  zeta = 0.4 + zetaPose*0.6;
  wn = 2 + wnPose*5;
  /* The controller paramters A, B, and k are solved for as a function 
  of the desired closed loop damping ratio and natural frequency. */
  phi = wn*wn;
  gamma = 2.0*zeta*wn;
  alpha = 10.0*wn;
  theta = M;
  B = ( theta*(phi+alpha*gamma) - theta*(alpha+gamma)*phi*alpha/(phi+alpha*gamma) + C*theta*phi*alpha/(M*(phi+alpha*gamma)) - C*(theta*(gamma+alpha)-C)/M ) / ( M + C*C/(M*(phi+alpha*gamma)) - (alpha+gamma)*C/(phi+alpha*gamma) );
  A = (theta*(gamma+alpha) - C)/M - theta*phi*alpha/(M*(phi+alpha*gamma)) + C/(M*(phi+alpha*gamma))*B;
  beta = -phi*alpha/(phi+alpha*gamma) + C/(theta*(phi+alpha*gamma))*B;
  k = M*alpha*phi*beta;

  /* The continuous time controller is discretized using Tustin's transformation. 
  The following are coefficients associated with the error and plant input (voltage) values*/
  Ce_2 = k*T*T/(4.0 + 2.0*A*T + B*T*T);
  Ce_1 = 2.0*Ce_2;
  Ce = Ce_2;
  Cu_2 = -(4.0 - 2.0*A*T + B*T*T)/(4.0 + 2.0*A*T + B*T*T);
  Cu_1 = -(2.0*B*T*T - 8.0)/(4.0 + 2.0*A*T + B*T*T);

  /* The continuous time filter is discretized using Tustin's transformation. 
  The following are coefficients associated with the raw data and the previous filtered data*/
  Csi = wc*T/(wc*T+2.0);
  Csi_1 = Csi;
  Cso_1 = -(wc*T-2.0)/(wc*T+2.0);
  
  Timer1.initialize(10000);
  Timer1.attachInterrupt(isr);
  pinMode(irPin, INPUT);
  AFMS.begin(); // Adafruit MotorShield
}

void loop() {
  driveMotors(u, mL, mR); // u = DC Voltage; // Drive the Motors with the Control Law
  if (millis() > 25000) {
    mL->setSpeed(RELEASE);
    mR->setSpeed(RELEASE);
    while(1);
  }
}

  /* Get Distance Reads the sensor (0 to 1024 analog), converts the reading to voltage, 
 and then calculates the inverse distance based on the manufacturer calibration. 
 The distance (in cm) is then determined. */
int getDistance() {
  irReading = analogRead(irPin);
  irV = irReading / 1024.0 * 5;
  /* The calibration is done with 3 linear regions from 0.5 V to 2.75 V */
  if (irV >= 0.5 && irV < 2) {
    invD = 0.017 * irV - 0.001;
  } else if (irV >= 2 && irV < 2.55) {
    invD = 0.030303 * irV - 0.0272727;
  } else if (irV >= 2.55 && irV < 2.75) {
    invD = 0.0833333 * irV - 0.1625;
  } else {                  // if irV < 0.5 || irV > 2.75
    invD = 0.01 / setDist;  // Set the robot in equilibrium if sensor reading is out of range.
  }
  dist = int(1 / invD);
  return dist;
}


/* driveMotors writes the DC voltage to both left and right motors. 
If the voltage is out of range, the motor is saturated. */
void driveMotors(float dcV, Adafruit_DCMotor *mL, Adafruit_DCMotor *mR) {
  nDcV = int (dcV / 12 * 255); // normalized magnitude (0 to 255 analog)
  if (nDcV >= 0 && nDcV <= 255) { // If Voltage is +0 to +12 V
    mL->run(FORWARD);
    mR->run(FORWARD);
    mL->setSpeed(nDcV);
    mR->setSpeed(nDcV);
  } else if (nDcV < 0 && nDcV >= -255) {  // If Voltage is -0 to -12 V
    mL->run(BACKWARD);
    mR->run(BACKWARD);
    mL->setSpeed(-nDcV);
    mR->setSpeed(-nDcV);
  } else if (nDcV > 255) {  // If Voltage is Positive Saturated 
    mL->run(FORWARD);
    mR->run(FORWARD);
    mL->setSpeed(255);
    mR->setSpeed(255);
  } else if (nDcV < -255) { // If Voltage is Negative Saturated
    mL->run(BACKWARD);
    mR->run(BACKWARD);
    mL->setSpeed(255);
    mR->setSpeed(255);
  }
}


/* In the ISR, the sensor is filtered and the control law is determined. */
void isr(){
  si = 0.01 * getDistance(); // Get distance, convert to meters
  so = Csi*si + Csi_1*si_1 + Cso_1*so_1; // Filter Sensor Reading
  e = -(setDist - so); // Calculate Error
  u = Ce_2*e_2 + Ce_1*e_1 + Ce*e + Cu_2*u_2 + Cu_1*u_1; // Controller
  /* Book Keeping */
  so_1 = so;
  si_1 = si;
  u_2 = u_1;
  u_1 = u;
  e_2 = e_1;
  e_1 = e;
}
