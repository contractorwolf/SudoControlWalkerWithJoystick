
/*
FORWARD DIRECTION, max speed ~180

//send 4 step movements
//forward walking, 4 steps  
//                       elbo   leg    elbo   leg    elbo   leg    elbo   leg
SendServoCommandList("#1P1541#2P1579#3P1192#4P1119#5P1582#6P1149#7P1227#8P1547T250");//step 1
SendServoCommandList("#1P1541#2P1079#3P1192#4P1619#5P1582#6P1649#7P1227#8P1047T250");//step 2
SendServoCommandList("#1P1191#2P1079#3P1542#4P1619#5P1232#6P1649#7P1577#8P1047T250");//step 3
SendServoCommandList("#1P1191#2P1579#3P1542#4P1119#5P1232#6P1149#7P1577#8P1547T250");//step 4

0-1024 pot values
530-2340 max servo values

*/

int pot1pin = A0;  // analog pin used to connect the potentiometer
int pot2pin = A1;  // analog pin used to connect the potentiometer

int forwardPin = 2; //joystick pin for forward
int reversePin = 3; //joystick pin for reverse

//potentiometer values
int val;  
int val2; 

//legit centers
int centers[] = {1366,1329,1367,1369,1407,1399,1402,1297};

int vertical_sway = 200;
int horizontal_sway = 175;
int time_delay = 300;


boolean walkingForward = false;
boolean walkingReverse = false;
boolean changedDirection = false;


long lastStep;
int currentStep = 0;
String lastCommand; 




String steps[] = {"#1P1541#2P1579#3P1192#4P1119#5P1582#6P1149#7P1227#8P1547T250",
                  "#1P1541#2P1079#3P1192#4P1619#5P1582#6P1649#7P1227#8P1047T250",
                  "#1P1191#2P1079#3P1542#4P1619#5P1232#6P1649#7P1577#8P1047T250",
                  "#1P1191#2P1579#3P1542#4P1119#5P1232#6P1149#7P1577#8P1547T250"}; 

void setup() {    
  pinMode(forwardPin, INPUT);  
  pinMode(reversePin, INPUT);    
  
  pinMode(pot1pin, INPUT);  // backbone servos controller
  pinMode(pot2pin, INPUT);  // leg servo controller

  Serial.begin(115200);// COM7 USB
  Serial1.begin(9600);// USC32 Servo Control Board
  
  Serial.println("MilliWalker v2 Started");

  delay(3000);
}


// the loop routine runs over and over again forever:
void loop() {
  
  // reads the value of the potentiometer (value between 0 and 1023)
  val = analogRead(pot1pin);             
  val2 = analogRead(pot2pin);

  // show analog values
  Serial.print("analog1 value: ");  
  Serial.println(val); 

  Serial.print("analog2 value: ");  
  Serial.println(val2);
  
  
  //********WALKING************
  
  
  //get joystick values from pins
  walkingForward = digitalRead(forwardPin);
  walkingReverse = digitalRead(reversePin);
  
  if(walkingForward){
      
    //has it been at least 300 millis
    if((millis()-lastStep)>300){  
        //walk forward
        incrementSteps();
        SendCommand(steps[currentStep]);
        Serial.println("step forward: ");
        Serial.println(currentStep);
        lastStep = millis();  
    }else{
        //announce waiting
        Serial.println("waiting on last step timeout");
    }   
  }else if(walkingReverse){
      if((millis()-lastStep)>300){  
        //go in reverse direction
        decrementSteps();   
        SendCommand(steps[currentStep]);
        Serial.println("step reverse: ");
        Serial.println(currentStep);
        lastStep = millis();    
      }else{
        //announce waiting
        Serial.println("waiting on last step timeout");
      }       
    }else if(!walkingReverse&&!walkingForward){
      //center all servos
      SendServoCommandList(centers[0],centers[1],centers[2],centers[3],centers[4],centers[5],centers[6],centers[7],val);     
      Serial.println("center all");
    }
}


void incrementSteps(){
  if(currentStep<3){
    currentStep++;
  }else{
    currentStep = 0;
  }
}

void decrementSteps(){
  if(currentStep>0){
    currentStep--;
  }else{
    currentStep = 3;
  }
}

void SendCommand(String command){
  Serial1.println(command);
  Serial.println(command);
}

void SendServoCommandList(int servo0,int servo1,int servo2,int servo3,int servo4,int servo5,int servo6,int servo7,int time_delay){

  //send to USC32 servo
  Serial1.print("#1P");
  Serial1.print(servo0);
  Serial1.print("#2P");
  Serial1.print(servo1);
  Serial1.print("#3P");
  Serial1.print(servo2);
  Serial1.print("#4P");
  Serial1.print(servo3);
  Serial1.print("#5P");
  Serial1.print(servo4);
  Serial1.print("#6P");
  Serial1.print(servo5);
  Serial1.print("#7P");
  Serial1.print(servo6);
  Serial1.print("#8P");
  Serial1.print(servo7);
  Serial1.print("T");
  Serial1.println(time_delay);

  //output to USB
  Serial.print("#1P");
  Serial.print(servo0);
  Serial.print("#2P");
  Serial.print(servo1);
  Serial.print("#3P");
  Serial.print(servo2);
  Serial.print("#4P");
  Serial.print(servo3);
  Serial.print("#5P");
  Serial.print(servo4);
  Serial.print("#6P");
  Serial.print(servo5);
  Serial.print("#7P");
  Serial.print(servo6);
  Serial.print("#8P");
  Serial.print(servo7);
  Serial.print("T");
  Serial.println(time_delay);

  delay(time_delay); 
}

void SendServoCommand(int pot_angle, int pot_angle2, int time_delay){
  //first segment responding to potentiometers
  int servo0 = map(pot_angle, 0, 1023, centers[0] + horizontal_sway, centers[0] - horizontal_sway); //horizontal_angle
  int servo1 = map(pot_angle2,0, 1023, centers[1] + vertical_sway, centers[1] - vertical_sway); //vertical_angle


  int servo2 = map(pot_angle, 0, 1023, centers[2] - horizontal_sway, centers[2] + horizontal_sway); //reverse_horizontal_angle
  int servo3 = map(pot_angle2,0, 1023, centers[3] - vertical_sway, centers[3] + vertical_sway); //reverse_vertical_angle

  int servo4 = map(pot_angle, 0, 1023, centers[4] + horizontal_sway, centers[4] - horizontal_sway); //horizontal_angle
  int servo5 = map(pot_angle2,0, 1023, centers[5] + vertical_sway, centers[5] - vertical_sway); //vertical_angle

  int servo6 = map(pot_angle, 0, 1023, centers[6] - horizontal_sway, centers[6] + horizontal_sway); //reverse_horizontal_angle
  int servo7 = map(pot_angle2,0, 1023, centers[7] - vertical_sway, centers[7] + vertical_sway); //reverse_vertical_angle

  //send to USC32 servo
  Serial1.print("#1P");
  Serial1.print(servo0);
  Serial1.print("#2P");
  Serial1.print(servo1);
  Serial1.print("#3P");
  Serial1.print(servo2);
  Serial1.print("#4P");
  Serial1.print(servo3);
  Serial1.print("#5P");
  Serial1.print(servo4);
  Serial1.print("#6P");
  Serial1.print(servo5);
  Serial1.print("#7P");
  Serial1.print(servo6);
  Serial1.print("#8P");
  Serial1.print(servo7);
  Serial1.print("T");
  Serial1.println(time_delay);

  //output to USB
  Serial.print("#1P");
  Serial.print(servo0);
  Serial.print("#2P");
  Serial.print(servo1);
  Serial.print("#3P");
  Serial.print(servo2);
  Serial.print("#4P");
  Serial.print(servo3);
  Serial.print("#5P");
  Serial.print(servo4);
  Serial.print("#6P");
  Serial.print(servo5);
  Serial.print("#7P");
  Serial.print(servo6);
  Serial.print("#8P");
  Serial.print(servo7);
  Serial.print("T");
  Serial.println(time_delay);

  delay(time_delay); 
}

