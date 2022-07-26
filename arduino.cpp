#include <Rotary.h>
#include <Servo.h>

Servo servoRight;
Servo servoLeft;

int16_t encoder = 0;
int16_t counter_set = 369;

uint8_t takeState, dropState, calState;
uint8_t limitState;

Rotary rotary = Rotary(11, 12);

// motor pins
#define InA1                    5                  // INA motor pin
#define InB1                    6                  // INB motor pin
#define PWM1                    7                   // PWM motor pin
#define limitSwitch             2                   // PWM motor pin

void setup() {
  
  servoRight.attach(9);
  servoLeft.attach(10);
  servoLeft.write(0);
  servoRight.write(0);
  
  pinMode(limitSwitch, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
 
  Serial.begin(115200);   

  //Startup Delay
  delay(1000);
}

void loop() {
  Serial.println("Menu");
  bool servoState = false, fn_isFinish = false;
  bool downState = false, fnState = false;
  readState();

  //*********************************************************
  //****************** Dropper Calibration ******************
  //*********************************************************
  
  while(!takeState == LOW && !dropState == LOW && !calState == HIGH){
    Serial.println("Calibration Mode");

    if(fn_isFinish == true){
      readState();
    }
    
    while(fnState == false && fn_isFinish == false){
      limitState = digitalRead(limitSwitch);
    Serial.println(!limitState);

      if(!limitState == 0){
        //Chamber goes Up
        MotorBackward();
      }
      //Check if Chamber has fully pulled
      else if(!limitState == 1){
        MotorStop();
        delay(2000);
        fnState = true;
        encoder = 0;
        break;
      }
    }
    
    while(fnState == true){
      //Get Encoder & Limit Status
      encoder = getEncoder(encoder);
      if(encoder != counter_set){
        MotorForward();
      }
      //Check if Chamber has fully pulled down
      else{
        MotorStop();
        fnState = false;
        fn_isFinish = true;
        break;
      }
    }
  }
  
  //*********************************************************
  //************** Drone go to Water Reservoir **************
  //*********************************************************
  
  //Check if Drone has arrived
  while(!takeState == HIGH && !dropState == LOW){
    Serial.println("Take Mode");
    
    if(fn_isFinish == true){
      readState();
    }
    
    while(fnState == false && fn_isFinish == false){
     //Dropping Chamber using Servo
      Serial.println("Dropping chamber...");
      servoRight.write(180);
      servoLeft.write(180);
      delay(2500);
      Serial.println("Finished...");

      fnState = true;
      break;
    }

    while(fnState == true){
      //Get Limit Status
      limitState = digitalRead(limitSwitch);
      
      if(!limitState == 0){
        //Chamber goes Up
        MotorBackward();
      }
      
      //Check if Chamber has fully pulled
      else if(!limitState == 1){
        MotorStop();
        fnState = false;
        fn_isFinish = true;
        break;
      }
    }
  } 

  
  //********************************************************
  //***************** Drone go to Dropzone *****************
  //********************************************************

  //Check if Drone has arrived
  while(!dropState == HIGH && !takeState == LOW){
    Serial.println("Drop Mode");
    encoder = 0;
    
    if(fn_isFinish == true){
      readState();
    }
    
    //Dropping Chamber
    while(fnState == false && fn_isFinish == false){
      //Get Encoder Status
      encoder = getEncoder(encoder);
      Serial.print("encoder: ");
      Serial.println(encoder);
      
      //Chamber goes down
      if(encoder != counter_set){
        MotorForward();
      }
      //Check if Chamber has fully pulled down
      else{
        MotorStop();
        delay(1000);
        fnState = true;
        break;
      }
    }

    while(fnState == true){
      limitState = digitalRead(limitSwitch);
      Serial.print("limitState: ");
      Serial.println(limitState);
      
      if(!limitState == 0){
        //Chamber goes Up
        MotorBackward();
      }
      //Check if Chamber has fully pulled
      else if(!limitState == 1){
        MotorStop();
        fnState = false;
        fn_isFinish = true;
        break;
      }
    }
  }
}


//********************************************************
//*********************** Function ***********************
//********************************************************

int16_t getEncoder(int16_t counter){
  //Start Encoder
  unsigned char result = rotary.process();
  if (result == DIR_CW) {
    counter++;
    Serial.println(counter);
  } 
  else if (result == DIR_CCW) {
    counter--;
    Serial.println(counter);
  }
  return counter;
}

void MotorForward(){
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, 255); 
}

void MotorBackward(){
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    analogWrite(PWM1, 255); 
}
void MotorStop(){
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, 0); 
}

void readState(){
  takeState = digitalRead(A2);
  dropState = digitalRead(A3);
  calState = digitalRead(A4);
}
