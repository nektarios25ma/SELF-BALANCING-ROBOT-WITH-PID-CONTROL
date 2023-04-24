
#include <MsTimer2.h>
//The speed PID control is realized by counting the speed code plate
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "Adeept_Distance.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/**********************Instantiate an object**********************/
MPU6050         mpu;                                    //Instantiate an MPU6050 object named mpu
BalanceCar      balancecar;
KalmanFilter    kalmanfilter;
Adeept_Distance Dist;

/***********************Pin definition***********************/
// The ultrasonic module controls the pins
#define TRIG            3
#define ECHO            5
// RGB Color lamp control pin
#define RPin            A0
#define GPin            A1
#define BPin            A2

// MPU6050 Gyroscope control pin
#define MPU_SCL         5
#define MPU_SDA         4
// TB6612 Chip control pins
#define TB6612_STBY     8
#define TB6612_PWMA     10
#define TB6612_PWMB     9
#define TB6612_AIN1     12
#define TB6612_AIN2     13
#define TB6612_BIN1     7
#define TB6612_BIN2     6
// Motor encoder controls pins
#define MOTOR1          2
#define MOTOR2          4
#define klaxon          11
//mode = 0:Remote control via Bluetooth mode
//mode = 1:Obstacle avoidance by ultrasonic mode
//mode = 2:Following  mode
int mode = 0;
int motorRun = 0;//0:Stop;  1:Go ahead;  2;Backwards;  3:Turn left;  4:Turn right;


// Ultrasonic detection of distance variables
int UT_distance = 0;
int detTime = 0; 

// Pulse calculation
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;

// Kalman_Filte
float Q_angle = 0.001, Q_gyro = 0.005;                      // Angular data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5;                                       // Filter sampling interval in milliseconds
float dt = timeChange * 0.001;                              // Note: Dt is the filter sampling time

// Angle data
float Q;
float Angle_ax;                                             // The Angle of tilt calculated by acceleration
float Angle_ay;
float K1 = 0.05;                                            // The weight of the accelerometer
float angle0 = -0.17;                                       // Angle of mechanical balance                float angle0 = -0.17;
int slong;

//          p:20      i:0.0     d:0.58
double kp = 23, ki = 0.0, kd = 0.48;                        // Parameters that you need to modify
//          p:5.5            i:0.1098           d:0.0
double kp_speed = 5.52, ki_speed = 0.1098, kd_speed = 0.0;  // Parameters that you need to modify
//          p:10            i:0          d:0.1
double kp_turn = 10, ki_turn = 0, kd_turn = 0.09;           // Rotary PID setting

int16_t ax, ay, az, gx, gy, gz;

int front = 0;                                              // Forward variables
int back = 0;                                               // Back variables
int turnl = 0;                                              // Turn left sign
int turnr = 0;                                              // Turn right
int spinl = 0;                                              // Rotate the left flag
int spinr = 0;                                              // Rotate the flag right

// Steering PID parameters
double setp0 = 0, dpwm = 0, dl = 0;                         // Angle balance, PWM poor, dead zone,PWM1,PWM2

// Turn and rotate parameters
int turncount = 0;                                          // Calculate the steering intervention time
float turnoutput = 0;

double Setpoint;                                            // Angle DIP set point, input, output
double Setpoints, Outputs = 0;                              // speed DIP set point, input, output

int speedcc = 0;
volatile long count_right = 0;                              // The volatile LON type is used to ensure that the external interrupt pulse meter values are valid when used in other functions
volatile long count_left = 0;
/*************************END**************************/

/*********************************************************
Function name: Pin_Init()
Function Initialize pin high/low level
Function parameters: None
The function returns: none
*********************************************************/
void Timer2Isr()
{
    sei();                                                  // Enable global variables
    countpluse();                                           // Pulse superposition subfunction
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);           // IIC obtains MPU6050 six-axis data ax ay az gx gy gz
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);                                   //Get Angle and Kaman filter
    angleout();                                             // Angle loop PD control
    speedcc++;
    if (speedcc >= 8)                                       // 40 ms into the speed loop control
    {
        Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
        speedcc = 0;
    }
    turncount++;
    if (turncount > 4)                                      // 40 ms into the rotation control
    {
        turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);                              //Rotate the subfunction
        turncount = 0;
    }
    balancecar.posture++;
    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB);                            //小车总PWM输出
    detTime++;
    if(detTime >= 4)                                        // 40 ms an ultrasound measurement
    {
        detTime = 0;
        UT_distance = Dist.getDistanceCentimeter();
    }
}


ISR(PCINT2_vect)
{
    count_right ++;
}   //Right speed dial count
void Code_left() 
{
  count_left ++;
}   //Left speed gauge count

void setup()
{
    Pin_Config();                                           // Module pin configuration
    Pin_Init();                                             // Module pin initialization
    Dist.begin(ECHO, TRIG);                                 // Add the ultrasonic module
    Wire.begin();                                           // Join the I2C bus sequence
    Serial.begin(9600);                                     // Initialize the baud rate of the serial port to 9600
    mpu.initialize();                                       // Initialize the MPU6050
    delay(1500);                                            // Wait for the system to stabilize
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    //5ms timed interrupt Settings use timer2  
    MsTimer2::set(5, Timer2Isr);
    MsTimer2::start();
}

void loop()
{
    attachInterrupt(0, Code_left, CHANGE);                  // Enable external interrupt 0               
    attachPinChangeInterrupt(MOTOR2);                       // Pin D4 is interrupted externally
  //  TX_Information(UT_distance);                            // Send ultrasonic data
   
//-----------------------------------------------------------
if(Serial.available() > 0){//Receive serial(Bluetooth) data   
       switch(Serial.read()){//Save the serial(Bluetooth) data received 
          case 'a': motorRun = 4;break;//go ahead
          case 'b': motorRun = 1;break;//turn right
          case 'c': motorRun = 2;break;//turn left
          case 'd': motorRun = 3;break;//backwards
          case 'e': mode = 0; motorRun = 0;break;
          case 'f': mode = 1; break;
          case 'g': mode = 2; break;
          case 'h': digitalWrite(klaxon, HIGH);break;
          case 'i': digitalWrite(klaxon, LOW); break;
          default:ResetCarState(); break;
       }
      }
      
      if(mode==0){//Remote control via Bluetooth mode
        switch(motorRun){
        case 0: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;  // control steering and reversing smart car
                digitalWrite(RPin, LOW);digitalWrite(GPin, HIGH);digitalWrite(BPin, HIGH);//red led
                break;
        case 1: turnr = 1; // control smart 2WD balance turn right
                digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
                break;
        case 2: turnl = 1;// control smart 2WD balance turn left
                digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
                break;
        case 3: back = 50;// control 2WD balance car backwards 
                digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
                break;
        case 4: front = -50;// control 2WD balance car go ahead
                digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
                break;
        default:ResetCarState();break;
        }
      }
       if(mode==1){//Obstacle avoidance by ultrasonic mode
        if(UT_distance<30){
          back = 50;// control 2WD balance car go ahead
           front = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;  
          digitalWrite(RPin, HIGH);digitalWrite(GPin, HIGH); digitalWrite(BPin, LOW); 
        }else if(UT_distance<60&&UT_distance>30){  
              turnl = 1; // control smart 2WD balance turn left 
              front = 0; back = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;  
            digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, LOW);
        }else{
              front = -50;// control 2WD balance car backwards
              back = 0;  turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;  
             digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
            }
      }
      if(mode==2){//Following  mode
        if(UT_distance>=30&&UT_distance<50){
          front = -50;// control 2WD balance car backwards
          digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
          }
        if(UT_distance<20&&UT_distance>5){
          back = 50;// control 2WD balance car go ahead
          digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW); digitalWrite(BPin, HIGH);
        }else{
           front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;  // control 2WD balance car stop
           digitalWrite(RPin, LOW);digitalWrite(GPin, HIGH); digitalWrite(BPin, HIGH);
        }
      }
ResetCarState();
}


void angleout()
{
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD Angle loop control
}

void countpluse()
{
    lz = count_left;
    rz = count_right;

    count_left = 0;
    count_right = 0;

    lpluse = lz;
    rpluse = rz;

    if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //When the moving direction of the trolley is judged to be backward (PWM motor voltage is negative), the pulse number is negative
    {
        rpluse = -rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))                //The number of pulses is negative when the moving direction of the trolley is judged to be forward (I.E. the voltage of the PWM motor is positive)
    {
        rpluse = rpluse;
        lpluse = lpluse;
    }
    else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                //The number of pulses is negative when the moving direction of the trolley is judged to be forward (I.E. the voltage of the PWM motor is positive)
    {
        rpluse = rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))                //The number of left rotation and right pulse is negative and the number of left pulse is positive
    {
        rpluse = -rpluse;
        lpluse = lpluse;
    }

    // Bring up the judgment
    balancecar.stopr += rpluse;
    balancecar.stopl += lpluse;

    // Every 5ms when the interrupt enters, the pulse number is superimposed
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;
}

/*********************************************************
Function name: mode1()
Function Function: Control the car direction and speed through Bluetooth
Function parameters: None
The function returns: none
*********************************************************/


/*********************************************************
Mode2 ()
Function Function: Obstacle avoidance mode
Function parameters: None
The function returns: none
*********************************************************/


/*********************************************************
Mode3 ()
Function Function: follow mode
Function parameters: None
The function returns: none
*********************************************************/




void ResetCarState()
{
    turnl = 0; 
    turnr = 0;  
    front = 0; 
    back = 0; 
    spinl = 0; 
    spinr = 0; 
    turnoutput = 0;
}



/*********************************************************
Function name: Pin_Init()
Function Initialize pin high/low level
Function parameters: None
The function returns: none
*********************************************************/
void Pin_Init()
{   
    digitalWrite(klaxon, LOW);                              // The buzzer controls the pin output low level
    
    digitalWrite(TB6612_STBY, HIGH);                        // TB6612 enable control pin output high level
    digitalWrite(TB6612_PWMA, LOW);                         // TB6612 PWMA control pin output low level
    digitalWrite(TB6612_PWMB, LOW);                         // 
    digitalWrite(TB6612_AIN1, LOW);                         // 
    digitalWrite(TB6612_AIN2, HIGH);                        // 
    digitalWrite(TB6612_BIN1, HIGH);                        // 
    digitalWrite(TB6612_BIN2, LOW);                         //
}

/*********************************************************
Function name: attachPinChangeInterrupt()
The D4 pin is set as an external interrupt
Function parameters: pin, 4
The function returns: none
*********************************************************/
void attachPinChangeInterrupt(int pin)
{
    pinMode(pin, INPUT_PULLUP);
    cli();
    PCMSK2 |= bit(PCINT20);
    PCIFR |= bit(PCIF2);
    PCICR |= bit(PCIE2); 
    sei();
}

/*********************************************************
Function name: Pin_Config()
The pin I/O function configures pin I/O mode
Function parameters: None
The function returns: none
*********************************************************/
void Pin_Config()
{
    pinMode(TRIG, OUTPUT);                                  // Ultrasonic Trig control pin configuration output
    pinMode(ECHO, INPUT);                                   // Ultrasonic Echo control pin configuration input

    pinMode(RPin, OUTPUT);                                  // RGB color lights red control pin configuration output
    pinMode(GPin, OUTPUT);                                  // RGB color light green control pin configuration output
    pinMode(BPin, OUTPUT);                                  // RGB color light blue control pin configuration output
    
    pinMode(klaxon, OUTPUT);                                // Buzzer control pin configuration output
    
    pinMode(TB6612_STBY, OUTPUT);                           // TB6612 Enable control pin configuration output
    pinMode(TB6612_PWMA, OUTPUT);                           // TB6612 PWMA control pin configuration output
    pinMode(TB6612_PWMB, OUTPUT);                           // TB6612 PWMB controls pin configuration output
    pinMode(TB6612_AIN1, OUTPUT);                           // 
    pinMode(TB6612_AIN2, OUTPUT);                           // 
    pinMode(TB6612_BIN1, OUTPUT);                           // 
    pinMode(TB6612_BIN2, OUTPUT);                           // 
    
    pinMode(MOTOR1, INPUT);                                 // Code motor 1 control pin configuration input
    pinMode(MOTOR2, INPUT);                                 // Code motor 2 control pin configuration input
}


