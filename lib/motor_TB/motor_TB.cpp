#include<motor_TB.h>
void c_motor_TB ::setPins(int n1,int n2,int pwm,int direction){
    this->direction=direction;
    this->n1=n1;
    this->n2=n2;
    this->pwm=pwm;
    pinMode(n1,OUTPUT);
    pinMode(n2,OUTPUT);
    pinMode(pwm,OUTPUT);
}
void c_motor_TB ::motor_speed(double speed){
    if(direction == 1){
        digitalWrite(n1,HIGH);
        digitalWrite(n2,LOW);
        analogWrite(pwm,speed);
    }
    if(direction == 0){
        digitalWrite(n2,HIGH);
        digitalWrite(n1,LOW);
        analogWrite(pwm,speed);
    }

}