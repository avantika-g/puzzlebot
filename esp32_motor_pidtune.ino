/*
 *  Build tcp connection and execute motor command
 *  Author: shayi@cs.cmu.edu
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include "Esp32Motor.h"

#define SERIAL_PRINT 1

WiFiMulti WiFiMulti;
WiFiClient client;

volatile long c[2] = { 0 };
unsigned long prev_time[2] = { 0 };
double time_diff = 0;
double cmd_v[2] = { 0 };
double wheel_v[2] = { 0 };
int pwms[2] = { 0 };

double curr_err[2] = { 0 };
double int_err[2] = { 0 };
double prev_err[2] = { 0 };
double kP = 0.05, kI = 0.01, kD = 0;  // interpolate

void setup() {
  // Wifi setup
  if (SERIAL_PRINT) {
    Serial.begin(115200);
    delay(10);
  }
  setup_wifi();
  setup_motor();
  delay(500);
}

void loop() {
  if (SERIAL_PRINT) {
    Serial.print("Connecting to ");
    Serial.print(host);
    Serial.print(", port:");
    Serial.println(port);
  }

  if (!client.connected()) {
    if (!client.connect(host, port)) {
      if (SERIAL_PRINT) {
        Serial.println("Connection failed.");
        Serial.println("Waiting 5 seconds before retrying...");
        cmd_v[0] = 0;
        cmd_v[1] = 0;
      }
      delay(5000);
    }
  }

  if (client.available() > 0) {
    //read back one line from the server
    String line = client.readStringUntil('\n');

    if (SERIAL_PRINT) {
      Serial.print("read:");
      Serial.println(line);
    }
    if (!parseInput(line)) {
      if (SERIAL_PRINT) Serial.println("Failed to parse input.");
    }
  } else {
    if (SERIAL_PRINT)
      Serial.println("client.available() timed out ");
  }

  getVelocity(c[0], prev_time[0], wheel_v[0]);
  getVelocity(c[1], prev_time[1], wheel_v[1]);

  pwms[0] = getPWM(0);
  pwms[1] = getPWM(1);
  if (SERIAL_PRINT) {
    Serial.print("cmd 0: ");
    Serial.print(cmd_v[0]);
    Serial.print(", cmd 1: ");
    Serial.println(cmd_v[1]);
    Serial.print("wheel 0: ");
    Serial.print((wheel_v[0]));
    Serial.print(", wheel 1: ");
    Serial.print((wheel_v[1]));
    Serial.print(", pwm 0: ");
    Serial.print(pwms[0]);
    Serial.print(", pwm 1: ");
    Serial.println(pwms[1]);
  }

  setVelocity();

  delay(20);
}

void setup_wifi() {

  WiFiMulti.addAP(ssid, password);

  if (SERIAL_PRINT) { Serial.println("Waiting for WiFi... "); }

  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(500);
  }

  if (SERIAL_PRINT) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  delay(500);
}

void setVelocity() {
  int pid_int = pwms[0];
  if (pid_int > 0 && pid_int < MIN_PWM) pid_int = 0;
  if (pid_int < 0 && pid_int > -MIN_PWM) pid_int = 0;
  if (pid_int >= 0) {
    analogWrite(motor0_m0, LOW);
    analogWrite(motor0_m1, (pid_int));
  } else {
    analogWrite(motor0_m1, LOW);
    analogWrite(motor0_m0, (-pid_int));
  }
    
  if (SERIAL_PRINT) {
    Serial.print(", pwm_vel0:");
    Serial.print(pid_int);
  }
  pid_int = pwms[1];
  if (pid_int >= 0) {
    analogWrite(motor1_m0, LOW);
    analogWrite(motor1_m1, (pid_int));
  } else {
    analogWrite(motor1_m1, LOW);
    analogWrite(motor1_m0, (-pid_int));
  }
  if (SERIAL_PRINT) {
    Serial.print(", pwm_vel1:");
    Serial.println(pid_int);
  }
}

void resetPID(int idx) {
  curr_err[idx] = { 0 };
  int_err[idx] = { 0 };
  prev_err[idx] = { 0 };
}

int getPWM(unsigned int idx) {
  if (time_diff == 0) return pwms[idx];
  double err = cmd_v[idx] - wheel_v[idx];
  if (abs(err) < TH) {
    resetPID(idx);
    return pwms[idx];
  }
  int_err[idx] += err * (time_diff / 1000);
  curr_err[idx] = err;
  // updatePID(cmd_v[idx]);
  double pid = kP * curr_err[idx] + kI * int_err[idx];
  if (SERIAL_PRINT) {
    Serial.print(", kp: ");
    Serial.print(kP);
    Serial.print(", ki: ");
    Serial.print(kI);
  }

  int pid_int = (int)pid + pwms[idx];
  if (pid_int > MAX_PWM) pid_int = MAX_PWM;
  if (pid_int < -MAX_PWM) pid_int = -MAX_PWM;
  return pid_int;
}

void updatePID(double err) {
  if (abs(err) > PID_UPPER) {
    kP = kP_h;
    kI = kI_h;
    return;
  } else if (abs(err) < PID_LOWER) {
    kP = kP_l;
    kI = kI_l;
    return;
  }
  kP = interpolate(abs(err), kP_l, kP_h);
  kI = interpolate(abs(err), kI_l, kI_h);
}

double interpolate(double input, double low, double high) {
  return (input - PID_LOWER) / (PID_UPPER - PID_LOWER) * (high - low) + low;
}
void getVelocity(volatile long& c, unsigned long& t, double& velocity) {
  unsigned long curr_time = millis();
  time_diff = (double)(curr_time - t);
  velocity = (double)c / ticksPerRev * 2 * PI / time_diff * 1000;  // rad/s
  c = 0;
  t = curr_time;
}

bool parseInput(String& value_str) {
  int idx_com[3];
  idx_com[0] = value_str.indexOf(',');
  idx_com[1] = value_str.indexOf(',', idx_com[0]+1);
  idx_com[2] = value_str.indexOf(',', idx_com[1]+1);
  for(int i=0; i<2; i++) {
    if (idx_com[i] < 0) return false;
  }

  String s_vec[4] = {value_str.substring(0, idx_com[0]), 
                    value_str.substring(idx_com[0]+1, idx_com[1]),
                    value_str.substring(idx_com[1]+1, idx_com[2]),
                    value_str.substring(idx_com[2]+1)};
  cmd_v[0] = s_vec[0].toDouble();
  cmd_v[1] = -s_vec[1].toDouble();  //flip motor
  kP = s_vec[2].toDouble(); kI = s_vec[3].toDouble();
  resetPID(0);
  resetPID(1);
  return true;
}

void setup_motor() {

  attachInterrupt(digitalPinToInterrupt(motor0_encoderA), updateCount0, CHANGE);
  // encoder reading pins
  pinMode(motor0_encoderA, INPUT);
  pinMode(motor0_encoderB, INPUT);
  // motor output pins
  pinMode(motor0_m0, OUTPUT);
  pinMode(motor0_m1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(motor1_encoderA), updateCount1, CHANGE);
  // encoder reading pins
  pinMode(motor1_encoderA, INPUT);
  pinMode(motor1_encoderB, INPUT);
  // motor output pins
  pinMode(motor1_m0, OUTPUT);
  pinMode(motor1_m1, OUTPUT);

  setVelocity();

  prev_time[0] = millis();
  prev_time[1] = millis();
  delay(100);
}

void updateCount0() {
  if (digitalRead(motor0_encoderA) == HIGH) {
    if (digitalRead(motor0_encoderB) == LOW)
      c[0]++;
    else
      c[0]--;
  } else {
    if (digitalRead(motor0_encoderB) == LOW)
      c[0]--;
    else
      c[0]++;
  }
}

void updateCount1() {
  if (digitalRead(motor1_encoderA) == HIGH) {
    if (digitalRead(motor1_encoderB) == LOW)
      c[1]++;
    else
      c[1]--;
  } else {
    if (digitalRead(motor1_encoderB) == LOW)
      c[1]--;
    else
      c[1]++;
  }
}