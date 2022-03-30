#ifndef JGA25Motor_h
#define JGA25Motor_h
#include <Arduino.h>

class JGA25Motor
{
    
    private:
    int dc1pin,dc2pin,pwmPin, motorCompensation, deadBand;
    double deadbandGain;
    bool motorClamped;
    void pinSetup();
    double calculateDeadBandGain(int deadBand);
    int motorPwmCorrection(double x);
    void Forward();
    void Backward();

    public:
    JGA25Motor(int direction1pin, int direction2pin, int pwm_Pin, int deadBand);
    void SetMotorCompensation(int a){motorCompensation = a;}
    bool GetClampStatus(){return motorClamped;}
    void SetSpeed(double pwm);
    void SetSpeed(double pwm, double steering);
    void Stop();
};
JGA25Motor::JGA25Motor(int direction1pin, int direction2pin, int pwm_Pin, int dead_Band){
    dc1pin = direction1pin;
    dc2pin = direction2pin;
    pwmPin = pwm_Pin;
    deadBand = dead_Band;
    deadbandGain = calculateDeadBandGain(dead_Band);
    pinSetup();
}
void JGA25Motor::Forward(){
    digitalWrite(dc1pin,0);
    digitalWrite(dc2pin,1);
}
void JGA25Motor::Backward(){
    digitalWrite(dc1pin,1);
    digitalWrite(dc2pin,0);
}
//PWM(0-255) eats over, positive forward, PWM negative backwards
void JGA25Motor::SetSpeed(double speedValue){
    speedValue > 0 ? Forward() : Backward();
    int compMotor = motorPwmCorrection(speedValue);
    analogWrite(pwmPin, compMotor);   
}
//subtracts steering from output, this means even tho max output is forced, unit will still turn.
void JGA25Motor::SetSpeed(double speedValue, double steering){
    speedValue > 0 ? Forward() : Backward();
    int compMotor = motorPwmCorrection(speedValue);
    analogWrite(pwmPin, (compMotor-steering));   
}
void JGA25Motor::Stop(){
    digitalWrite(dc1pin,0);
    digitalWrite(dc2pin,0);
    analogWrite(pwmPin,0);
}
double JGA25Motor::calculateDeadBandGain(int x){
    return 1+(x/(double)255);
}

int JGA25Motor::motorPwmCorrection(double x){
    //adds deadband
    x =(int)(deadbandGain * abs(x) + deadBand);
    //updates if motorPower is exceeded
    motorClamped = x > 255 ? true : false;
    //ensures value does not exceed -255 og 255
    x = constrain(x, 0, 255); 
    return x;
}

void JGA25Motor::pinSetup(){
    pinMode(dc1pin,1);
    pinMode(dc2pin,1);
    pinMode(pwmPin,1);
    digitalWrite(dc1pin, 0);
    digitalWrite(dc2pin, 0);
    analogWrite(pwmPin, 0);
}


#endif