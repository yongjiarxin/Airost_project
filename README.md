//Controller = Rapsberry Pi Pico W
//Motor driver = Cytron Maker Pi Pico
//Communication = Bluetooth via Pico W
//Color sensor = TCS34725
//Line sensor= two LM358
//Motor 1 A : GP8
//Motor 1 B : GP9
//Motor 2 A : GP10
//Motor 2 B : GP 11
//Servo 1 : GP12
//Servo 2 : GP13
//IR Sensor 1 ： GP3
//IR Sensor 2 :  GP17
//PWM speed : use 20% * 255 =51


#include<Wire.h>
#include<Adafruit_TCS34725.h>
#include<Servo.h>
#include<SerialBT.h>

const int M1A =8;
const int M1B =9;
const int M2A =10;
const int M2B =11;

const int LEFT_IR_PIN = 3 ;
const int RIGHT_IR_PIN = 17;

const int SERVO_PIN =12;

const int MOTOR_SPEED_PERCENT =30;    //PWM is used for control the movement speed
const int MOTOR_PWM = (MOTOR_SPEED_PERCENT*255L)/100;

const int MAX_PWM = 255;

const bool INVERT_LEFT_MOTOR =false;
const bool INVERT_RIGHT_MOTOR =false;  //can change after checking the real situatiom

const int BLACK = HIGH;
const int WHITE = LOW;

Adafruit_TCS34725colorSensor(TCS34725_INTEGRATIONTIME_50MS , TCS34725_GAIN_4X);  //integration time=50milliseconds,and gain 4 x mean increase the detcted signal ,still can adjust ,depend on the real situation
bool colorSensorReady =false;

enum Color{                    //used for record the color
  UNKNOWN,
  RED,
  BLUE,
  GREEN       //based on the color given,store the taget color detected the first number
} ;   

Color targetColor=UNKNOWN;  
Color detectedColor=UNKNOWN;

Servo scoopServo;

const int SERVO_REST_ANGLE =90;
const int SERVO_SCOOP_ANGLE =30;

enum AutoState{                //used for record the status of the robot
  AUTO_START,
  READ_FIRST_MARKER,
  FOLLOW_LINE,
  SEARCH_TARGET_MARKER,
  TARGET_FOUND,
  AUTO_FINISHED,
  AUTO_ERROR
};

enum Mode{
  MANUAL , 
  AUTO
};

Mode currentMode = MANUAL;

AutoState autoState = AUTO_START;  //autoState is the variable and we defined that AUTO_START is the initial

//BLUETOOTH CONFIGURATION
//'F'=FORWARD
//'B'=BACKWARD
//'L'=LEFT
//'R'=RIGHT
//'S'=STOP
//'A'=AUTO MODE
//'M'=MANUAL MODE
//'U'=SCOOP UP
//'D'=SCOOP DOWN

char bluetoothCommand ='S';
unsigned long lastCommandTime =0;  //unsigned means no negative number,it have only 0 and positive integer
const unsigned long COMMAND_TIMEOUT =300;

const bool AUTO_TEST_ENABLED =false;

//Color calibration configuration

const uint16_t MIN_CLEAR_VALUE = 100;

const float RED_THRESHOLD =0.45;    //R/R+B+G  ,R=600,B=200,G=200
const float BLUE_THRESHOLD =0.40;
const float GREEN_THRESHOLD=0.40;

int leftIRValue = LOW;
int rightIRValue = LOW;

unsigned long lastSensorRead = 0;

const unsigned long SENSOR_INTERVAL = 50;

void setup(){
  Serial.begin(115200);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M2A,OUTPUT);
  pinMode(M2B,OUTPUT);

  stopMotors();  //to makesure the motor will not operation suddenly

  pinMode(LEFT_IR_PIN,INPUT);
  pinMode(RIGHT_IR_PIN,INPUT);

  scoopServo.attach(SERVO_PIN);
  releaseBall();

  colorSensorReady = colorSensor.begin();
  if(colorSensorReady){
    Serial.println("Color sensor is ready");
  }
  else{
    Serial.println("Color sensor is error");
  }

  SerialBT.begin("AIRO-PLAN AIROST");

  Serial.println("AIRO-PLAN system is ready");
  Serial.println("Default mode : Manual");
  stopMotor();
}

void loop(){
  readBluetooth();

  if(currentMode==MANUAL){
    manualMode();
  }
  else if (currentMode==AUTO){
    autoMode();
  }
  else {
    stopMotors();
  }
}

//motor control
void setLeftMotor(int speed){   
  speed= constrain(speed,-MAX_PWM,MAX_PWM);    //contrain is used for limit the value ,formula is likr (value,minimum,maximum)

  if(INVERT_LEFT_MOTOR){
    speed=-speed;
  }

  if(speed>0){
    analogWrite(M1A,speed);  //means we use this parameter to determine how fast we want it operate
    analogWrite(M1B,0);
  }
  else if (speed<0){
    analogWrite(M1A,0);
    analogWrite(M1B,-speed);
  }
  else {
    analogWrite(M1A,0);
    analogWrite(M1B,0);
  }

}
void setRightMotor(int speed){   
  speed= constrain(speed,-MAX_PWM,MAX_PWM);    
  if(INVERT_RIGHT_MOTOR){
    speed=-speed;
  }

  if(speed>0){
    analogWrite(M2A,speed);  //means we use this parameter to determine how fast we want it operate
    analogWrite(M2B,0);
  }
  else if (speed<0){
    analogWrite(M2A,0);
    analogWrite(M2B,-speed);
  }
  else {
    analogWrite(M2A,0);
    analogWrite(M2B,0);
  }
}

//movement control
void moveForward(){
  setLeftMotor(MOTOR_PWM);
  setRightMotor(MOTOR_PWM);
}

void moveBackward(){
  setLeftMotor(-MOTOR_PWM);
  setRightMotor(-MOTOR_PWM);
}

void turnLeft(){
  setLeftMotor(-MOTOR_PWM);
  setRightMotor(MOTOR_PWM);
}

void turnRight(){
  setLeftMotor(MOTOR_PWM);
  setRightMotor(-MOTOR_PWM);
}

void stopMotors(){
  setLeftMotor(0);
  setRightMotor(0);
}

void scoopBall(){
  scoopServo.write(SERVO_SCOOP_ANGLE);
}

void releaseBall(){
  scoopServo.write(SERVO_REST_ANGLE);
}

//Bluetooth receiving 
//Forward = F
//Backward = B
//Stop = S
//Left = L
//Right = R
//Auto mode = A
//Manual mode = M

void readBluetooth(){                //determine the manual and auto mode
  while(SerialBT.available() >0 ){  //SerialBT is the communication connected for the Pico W
    char command = SerialBT.read();
    if(command =='\n' || command =='\r'){         //'\n' is the new line ,'/r'is the carriage return
      continue;
    }
    bluetoothCommand =command;
    lastCommandTime =millis();

//auto mode
    if(command =='A'){
      stopMotors();        //safety percautions

      autoState =AUTO_START;
      targetColor =UNKNOWN;        //gonna change the color

      currentMode =AUTO;
      bluetoothCommand ='S';       //clear the previous imfo
    }
    else if(command =='M'){
      stopMotors();

      currentMode =MANUAL;
      bluetoothCommand ='S';
    }
  }
}

void manualMode(){
  //stop if bluetooth command times out
  if(millis()-lastCommandTime > COMMAND_TIMEOUT){   //>300milliseconds
    stopMotors(); 
    bluetoothCommand = 'S';
    return;
  }
  switch(bluetoothCommand){     //choose choices
    case'F': moveForward();
    break;

    case'B' :moveBackward();
    break;


    case'L' :turnLeft();
    break;

    case'R' :turnRight();
    break;

    default:
    stopMotors();
    break;
  }
}

//read the IR SENSORS

void readLineSensors(){
  leftIRValue = digitalRead(LEFT_IR_PIN);
  rightIRValue = digitalRead(RIGHT_IR_PIN);
}

void printIRValues(){
  Serial.print("LEFT");
  Serial.print(leftIRValue);

  Serial.print("RIGHT");
  Serial.print(rightIRValue);
}

Color readDetectedColor(){
  if(!colorSensorReady){
    return UNKNOWN;
  }
  uint16_t r,g,b,c   //unsigned integer 16 bit,r=red,g=green,b=blue,c=clear

  colorSensor.getRawData(&r,&g,&b,&c);   //start to read the data

  Serial.print("R:");
  Serial.print(r);

  Serial.print("G:");
  Serial.print(g);

  Serial.print("B:");
  Serial.print(b);

  if (c<MIN_CLEAR_VALUE){
    return UNKNOWN;
  }


float total=(float)r +g +b;

  if (total <=0){            //to determine the light is enough or not
    return UNKNOWN;
  }

   float redRatio = r / total;
   float greenRatio = g / total;
   float blueRatio = b / total;

   if (redRatio > RED_THRESHOLD &&
      redRatio > greenRatio &&
      redRatio > blueRatio)
   {
    return RED;
   }

   if (blueRatio > BLUE_THRESHOLD &&
      blueRatio > redRatio &&
      blueRatio > greenRatio)
   {
    return BLUE;
   }

   if (greenRatio > GREEN_THRESHOLD &&
      greenRatio > redRatio &&
      greenRatio > blueRatio)
   {
    return GREEN;
   }

  return UNKNOWN;
}

void printColor(Color color){
  switch (color){
    case RED: 
    Serial.println("RED");
    break;

    case BLUE: 
    Serial.println("BLUE");
    break;

    case GREEN: 
    Serial.println("GREEN");
    break;

    default:
    Serial.println("UNKNOWN");
    break;
  }
}

void storeTargetColor(){
  Color newColor =readDetectedColor();

  if (newColor != UNKNOWN){
    targetColor = newColor;
    Serial.print("Target Stored:");

    if (targetColor == RED) {
      Serial.println("RED");
    }
    else if (targetColor == BLUE) {
      Serial.println("BLUE");
    }
    else if (targetColor == GREEN) {
      Serial.println("GREEN");
    }

    autoState=FOLLOW_LINE;
  }
}

bool checkTargetColor(){
  Color currentColor = readDetectedColor();
  if(currentColor==UNKNOWN){
    return false;
  }
  if(currentColor ==targetColor){
    Serial.println("MATCHING COLOR FOUND");
    return true;
  }
  else{
    Serial.println("NOT MATCHING");
    return false;
  }
}

void followLine(){
  readLineSensors();

  if(leftIRValue == BLACK && rightIRValue ==BLACK){
    moveForward();
  }
  else if(leftIRValue == BLACK &&rightIRValue ==WHITE){
    turnLeft();
  }
  else if(leftIRValue ==WHITE &&rightIRValue ==BLACK){
    turnRight();
  }
  else{
    stopMotors();
  }
}

void autoMode() {

  switch (autoState) {

    case AUTO_START:
      stopMotors();
      targetColor = UNKNOWN;

      if (!colorSensorReady) {
        autoState = AUTO_ERROR;
        break;
      }

      Serial.println("AUTO MODE STARTED");
      autoState = READ_FIRST_MARKER;
      break;


    case READ_FIRST_MARKER:
      stopMotors();
      storeTargetColor();
      break;


    case FOLLOW_LINE:
      followLine();

      // This state currently only follows the line.
      // The transition to SEARCH_TARGET_MARKER
      // still needs to be designed and tested.
      break;


    case SEARCH_TARGET_MARKER:
      if (checkTargetColor()) {
        stopMotors();
        autoState = TARGET_FOUND;
      }
      else {
        followLine();
      }
      break;


    case TARGET_FOUND:
      stopMotors();
      Serial.println("TARGET AREA FOUND");

      currentMode = MANUAL;
      bluetoothCommand = 'S';
      break;


    case AUTO_FINISHED:
      stopMotors();
      break;


    case AUTO_ERROR:
      stopMotors();
      Serial.println("AUTO MODE ERROR");
      break;
  }
}
