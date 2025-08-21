#ifndef _MOTOR_TB_H_
#define _MOTOR_TB_H_
#include<Arduino.h>
class c_motor_TB
{

private:
int n1,n2,pwm,direction;
public:
    void setPins(int n1,int n2,int pwm,int direction);
    void motor_speed(double speed);
};






#endif // _MOTOR_TB_H

