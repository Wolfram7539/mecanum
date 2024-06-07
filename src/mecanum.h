#ifndef MECANAM
#define MECANAM

#define speedPinR 9   //  Front Wheel PWM pin connect Right MODEL-X ENA 
#define RightMotorDirPin1  22    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2  24   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1  26    //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2  28   //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Right MODEL-X ENB

#define speedPinRB 11   //  Rear Wheel PWM pin connect Left MODEL-X ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B 6    //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B 7    //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2B 8  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB 12
enum motor_e {
  MOTOR_RA,
  MOTOR_LA,
  MOTOR_RB,
  MOTOR_LB
};
static const double PI_div4 = atan(1.0);
class Mecanam {
  private:
    bool dir[4];
    uint8_t out[4];
    double gain = 1.0;
  public:
    Mecanam(void);
    virtual void MotorDrive(motor_e motor, bool dir, uint8_t speed);
    virtual bool calcurate(int x, int y, double angle_now = 0);
    virtual bool dirc(motor_e motor){return dir[motor];}
    virtual uint8_t output(motor_e motor){return out[motor];}
    virtual void Allstop(void);
};
Mecanam::Mecanam(void) {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);
}
void Mecanam::MotorDrive(motor_e motor, bool dir, uint8_t speed) {
  switch (motor) {
    case 0:
      digitalWrite(RightMotorDirPin1, !dir);
      digitalWrite(RightMotorDirPin2, dir);
      analogWrite(speedPinR, speed);
      break;
    case 1:
      digitalWrite(LeftMotorDirPin1, !dir);
      digitalWrite(LeftMotorDirPin2, dir);
      analogWrite(speedPinL, speed);
      break;
    case 2:
      digitalWrite(RightMotorDirPin1B, !dir);
      digitalWrite(RightMotorDirPin2B, dir);
      analogWrite(speedPinRB, speed);
      break;
    case 3:
      digitalWrite(LeftMotorDirPin1B, !dir);
      digitalWrite(LeftMotorDirPin2B, dir);
      analogWrite(speedPinLB, speed);
      break;
  }
}
bool Mecanam::calcurate(int x, int y, double angle_now = 0) {
  double Ly = y ;
  double Lx = x ;
  double radius = sqrt((Lx * Lx) + (Ly * Ly));

  if (radius < 0) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    return 0;
  }

  double angle_ctrller = atan2(Ly, Lx);

  double Vx = Lx * 200.0 /115;
  double Vy = Ly * 200.0 /115;
  double sp = sqrt((Vx * Vx) + (Vy * Vy));
  
  /* 各ホイールに対応する出力計算 */
  int d =1;
  d = Vx >= 0 ? 1:-1;
  double V0,V1,V2,V3;
  if(Vx * Vy >=0){
    V0 = -1 * sp * cos(2 * angle_ctrller)* d;
    V1 = sp * d;
    V2 = sp * d;
    V3 = -1 * sp * cos(2 * angle_ctrller) * d;
  }else {
    V0 = -1 * sp * d;
    V1 = sp * cos(2 * angle_ctrller) * d;
    V2 = sp * cos(2 * angle_ctrller) * d;
    V3 = -1 * sp * d;
  }
  Serial.print("V0 = ");
  Serial.println(V0);
  Serial.print("V1 = ");
  Serial.println(V1);
  Serial.println();
  V0 = V0 > 200 ? 200:V0;
  V0 = V0 < -200 ? -200:V0;
  V1 = V1 > 200 ? 200:V1;
  V1 = V1 < -200 ? -200:V1;
  V2 = V2 > 200 ? 200:V2;
  V2 = V2 < -200 ? -200:V2;
  V3 = V3 > 200 ? 200:V3;
  V3 = V3 < -200 ? -200:V3;
  
  dir[0] = V0 > 0 ? 0 : 1;
  dir[1] = V1 > 0 ? 0 : 1;
  dir[2] = V2 > 0 ? 0 : 1;
  dir[3] = V3 > 0 ? 0 : 1;
  /* duty出力 */
  out[0] = (uint8_t)(fabs(V0));
  out[1] = (uint8_t)(fabs(V1));
  out[2] = (uint8_t)(fabs(V2));
  out[3] = (uint8_t)(fabs(V3));

  
  return 1;
}
void Mecanam::Allstop(void){
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
}
#endif
