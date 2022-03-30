# SegwayPID
Arduino Segway, PID regulated with antiwindup for motor and filtered derivative.<br/>
### PID
PID library contains 3 different PID's, they are selected by using the different constructors<br/>
PID(double kp, double ki, double kd, double cycletime) normal PID<br/>
PID(double kp, double ki, double kd, double cycletime, bool motorMaxPowerReached) PID with antiwindup<br/>
PID(double kp, double ki, double kd, double cycletime, bool motorMaxPowerReached, double HZcutoff) PID with antiwindup and filtered derivative<br/>

The error is filtered using a second order Butterworth filter, the filter with calculate the coefficients during the start-up sequence of the Arduino. 

### Steerable
made to work with Arduino Bluetooth Controlled Joystick App from Uncia Robotics <br/>
https://play.google.com/store/apps/details?id=uncia.robotics.joystick&hl=en&gl=US

## Final build
![finalversjon](https://user-images.githubusercontent.com/102175435/160910292-77d91f13-3ec4-4ca5-b855-1a043e2762b9.jpg)
## Wireschematic
![EL build](https://user-images.githubusercontent.com/102175435/160910330-2d03c0a2-bec7-409e-8490-9ffeafb73311.jpg)
