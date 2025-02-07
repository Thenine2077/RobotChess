int sw1=2, sw2=3, sw3=4, sw4=5;
int pul1 =6, pul2=8, pul3 =10, pul4=12;
int dir1 =7, dir2=9, dir3 =11, dir4=13;
int m1 =32, m2 =32, m3 =32, m4 =32;; //Micro step
int ppr1 = m1*200, ppr2 = m2*200, ppr3 = m3*200, ppr4 = m4*200;  //Pulses per revolution
//float ig1=1, ig2=1, ig3=1, ig4=1; //gear ratio

float ig1 = 20.72, ig2 = 27.33, ig3 = 17.25, ig4 = 4.5; // Gear ratio
float theta1, theta2, theta3, theta4;
float targetPos1=0, targetPos2=0, targetPos3=0, targetPos4=0;
float currentPos1=0,currentPos2=0, currentPos3=0,currentPos4=0;
float sq[4][4], sq1[4][4]={{0,0,0,0}, {90,-135,-90,45}, {90,0,0,0},{180,-135,-90,45}};

float t=8;
bool moveMotors=0,home=0;

//Delay time (us/pulse); for TB5600, m =32 min dt=1 us ,
// m=16 min dt =5 us,m =8 min dt =50, m=4 min dt =150
//m =2 mint dt=350, m=1 min dt =700
float dt1_=500, dt2_=500, dt3_=500, dt4_=800;
float dt1, dt2, dt3, dt4;

bool direction1, direction2, direction3, direction4;
char command;
unsigned long previousTime1, previousTime2, previousTime3, previousTime4;
unsigned long currentTime1, currentTime2,currentTime3, currentTime4;

void setup() {
 Serial.begin(9600);

  pinMode(pul1,OUTPUT);
  pinMode(dir1,OUTPUT);
  pinMode(pul2,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(pul3,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(pul4,OUTPUT);
  pinMode(dir4,OUTPUT);

  //magnetic sw
  pinMode(sw1,INPUT);
  pinMode(sw2,INPUT);
  pinMode(sw3,INPUT);
  pinMode(sw4,INPUT);

}

void loop() {
  runMotor();
  checkSerial();
}

void checkSerial(){
  if (Serial.available()==true){

    command=Serial.read(); //Read the first charractor.

    if (command=='f'){ 
      moveMotors=1;
      theta1=Serial.parseFloat();
      theta2=Serial.parseFloat();
      theta3=Serial.parseFloat();
      theta4=Serial.parseFloat();
      runMotor1();
      runMotor2();
      runMotor3();
      runMotor4();

    }

    if (command=='1'){ 
      moveMotors=0;
      theta1=Serial.parseFloat();
      runMotor1();
    }

    if (command=='2'){ 
      moveMotors=0;
      theta2=Serial.parseFloat();
      runMotor2();
    }

    if (command=='3'){ 
      moveMotors=0;
      theta3=Serial.parseFloat();
      runMotor3();
    }

    if (command=='4'){ 
      moveMotors=0;
      theta4=Serial.parseFloat();
      runMotor4();
    }

    if (command=='t'){ //Change the time taken to move.
      t=Serial.parseFloat();
      Serial.print("The time taken to move = ");
      Serial.print(t);
      Serial.println(" seconds");
    }

     if (command=='d'){ //Change time delay.
      dt1_ = Serial.parseFloat();
      dt2_ = Serial.parseFloat();
      dt3_ = Serial.parseFloat();
      dt4_ = Serial.parseFloat();

      Serial.print("dt = [");
      Serial.print(dt1_);
      Serial.print(" , ");
      Serial.print(dt2_);
      Serial.print(" , ");
      Serial.print(dt3_);
      Serial.print(" , ");
      Serial.print(dt4_);
      Serial.println("]");
     }

    if (command=='i'){ //chage gear ratio
      ig1=Serial.parseFloat();
      ig2=Serial.parseFloat();
      ig3=Serial.parseFloat();
      ig4=Serial.parseFloat();

      Serial.print("gear ratio ig = [");
      Serial.print(ig1);
      Serial.print(" , ");
      Serial.print(ig2);
      Serial.print(" , ");
      Serial.print(ig3);
      Serial.print(" , ");
      Serial.print(ig4);
      Serial.println("]");

    }

    if (command=='s'){ //stop
      targetPos1 = 0;
      targetPos2 = 0;
      targetPos3 = 0;
      targetPos4 = 0;
    }

    if (command=='h'){ //setcurrent postition to zero position
    
      homing1();
      delay(500);
      homing2();
      delay(500);
      homing3();
      delay(500);
      homing4();
      delay(500);
    }
/*
     if (command=='x'){ //show case
      for (int i=0; i<4;i++){
        for (int j=0;j<4; j++){
          moveMotors=1;

          theta1=sq[i][j]
        }
      }
    
 
      Serial.println("Set current postition to zero position");
    }
  
*/
    if (command=='m'){ //setcurrent postition to zero position
      m1=Serial.parseInt();
      m2=Serial.parseInt();
      m3=Serial.parseInt();
      m4=Serial.parseInt();

      ppr1=m1*200;
      ppr2=m2*200;
      ppr3=m3*200;
      ppr4=m4*200;

      Serial.print("micro step m = [");
      Serial.print(m1);
      Serial.print(" , ");
      Serial.print(m2);
      Serial.print(" , ");
      Serial.print(m3);
      Serial.print(" , ");
      Serial.print(m4);
      Serial.println("]");
    }
    
  }

}

void homing1(){
  
   digitalWrite(dir1, 0);
  while (digitalRead(sw1)==1){  
    digitalWrite(pul1,1);
    delayMicroseconds(dt1_);
    digitalWrite(pul1,0);
  }

  delay(1000);
  float theta1Cal=-40; //Angle at magnetaic switch 1
  currentPos1=ig1*theta1Cal*ppr1/360.00;
  theta1=0;
  
  targetPos1=ig1*(theta1*ppr1/360)-currentPos1;
  direction1=(targetPos1<0) ? 0: 1;
  targetPos1=abs(targetPos1);
  digitalWrite(dir1, direction1);

  previousTime1=micros(); //The start time of a pulse.

  while (targetPos1>0){

    currentTime1=micros();

    digitalWrite(pul1,1);

    if (currentTime1-previousTime1>=dt1_){
      digitalWrite(pul1,0);
      previousTime1=currentTime1;
      currentPos1=(direction1==1) ? currentPos1+1 : currentPos1-1;
      targetPos1--;
    }
    
  }

}

void homing2(){
  digitalWrite(dir2, 0);

  while (digitalRead(sw2)==1){
    
    digitalWrite(pul2,1);
    delayMicroseconds(dt2_);
    digitalWrite(pul2,0);
  }

  delay(1000);
  float theta2Cal=52; //Angle at magnetaic switch 255
  currentPos2=ig2*theta2Cal*ppr2/360.00;
  theta2=90;
  
  targetPos2=ig2*(theta2*ppr2/360)-currentPos2;
  direction2=(targetPos2<0) ? 0: 1;
  targetPos2=abs(targetPos2);
  digitalWrite(dir2, direction2);

  previousTime2=micros(); //The start time of a pulse.

  while (targetPos2>0){

    currentTime2=micros();

    digitalWrite(pul2,1);

    if (currentTime2-previousTime2>=dt2_){
      digitalWrite(pul2,0);
      previousTime2=currentTime2;
      currentPos2=(direction2==1) ? currentPos2+1 : currentPos2-1;
      targetPos2--;
    }
    
  }

}

void homing3(){
  digitalWrite(dir3, 1);

  while (digitalRead(sw3)==1){
    
    digitalWrite(pul3,1);
    delayMicroseconds(dt3_);
    digitalWrite(pul3,0);
  }

  delay(1000);
  float theta3Cal=-4; //Angle at magnetaic switch 2
  currentPos3=ig3*theta3Cal*ppr3/360.00;
  theta3=0;
  
  targetPos3=ig3*(theta3*ppr3/360)-currentPos3;
  direction3=(targetPos3<0) ? 0: 1;
  targetPos3=abs(targetPos3);
  digitalWrite(dir3, direction3);

  previousTime3=micros(); //The start time of a pulse.

  while (targetPos3>0){

    currentTime3=micros();

    digitalWrite(pul3,1);

    if (currentTime3-previousTime3>=dt3_){
      digitalWrite(pul3,0);
      previousTime3=currentTime3;
      currentPos3=(direction3==1) ? currentPos3+1 : currentPos3-1;
      targetPos3--;
    }
    
  }

}

void homing4(){
  digitalWrite(dir4, 1);

  while (digitalRead(sw4)==1){
    
    digitalWrite(pul4,1);
    delayMicroseconds(dt4_);
    digitalWrite(pul4,0);
  }

  delay(1000);
  float theta4Cal=-9; //Angle at magnetaic switch 2
  currentPos4=ig4*theta4Cal*ppr4/360.00;
  theta4=0;
  
  targetPos4=ig4*(theta4*ppr4/360)-currentPos4;
  direction4=(targetPos4<0) ? 0: 1;
  targetPos4=abs(targetPos4);
  digitalWrite(dir4, direction4);

  previousTime4=micros(); //The start time of a pulse.

  while (targetPos4>0){

    currentTime4=micros();

    digitalWrite(pul4,1);

    if (currentTime4-previousTime4>=dt4_){
      digitalWrite(pul4,0);
      previousTime4=currentTime4;
      currentPos4=(direction4==1) ? currentPos4+1 : currentPos4-1;
      targetPos4--;
    }
    
  }

}

void runMotor1(){
  previousTime1=micros(); //The start time of a pulse.
  targetPos1=ig1*(theta1*ppr1/360)-currentPos1;
  direction1=(targetPos1<0) ? 0: 1;
  targetPos1=abs(targetPos1);
  dt1= (moveMotors==1) ? t*1e6/targetPos1 : dt1_;
  digitalWrite(dir1, direction1);
}

void runMotor2(){
  previousTime2=micros(); 
  targetPos2=ig2*(theta2*ppr2/360)-currentPos2;
  direction2=(targetPos2<0) ? 0: 1;
  targetPos2=abs(targetPos2);
  dt2= (moveMotors==1) ? t*1e6/targetPos2 : dt2_;
  digitalWrite(dir2, direction2);
}

void runMotor3(){
  previousTime3=micros(); 
  targetPos3=ig3*(theta3*ppr3/360)-currentPos3;
  direction3=(targetPos3<0) ? 0: 1;
  targetPos3=abs(targetPos3);
  dt3= (moveMotors==1) ? t*1e6/targetPos3 : dt3_;
  digitalWrite(dir3, direction3);
}

void runMotor4(){
  previousTime4=micros(); 
  targetPos4=ig4*(theta4*ppr4/360)-currentPos4;
  direction4=(targetPos4<0) ? 0: 1;
  targetPos4=abs(targetPos4);
  dt4= (moveMotors==1) ? t*1e6/targetPos4 : dt4_;
  digitalWrite(dir4, direction4);
}

void runMotor(){

  if (targetPos1<=0 && targetPos2<=0 && targetPos3<=0 && targetPos4<=0 ){
    moveMotors=0;
    return;
  }
 
  if (targetPos1>0){

    currentTime1=micros();

    digitalWrite(pul1,1);

    if (currentTime1-previousTime1>=dt1){
      digitalWrite(pul1,0);
      previousTime1=currentTime1;
      currentPos1=(direction1==1) ? currentPos1+1 : currentPos1-1;
      targetPos1--;

    }
    
  }

  if (targetPos2>0){

    currentTime2=micros();

    digitalWrite(pul2,1);

    if (currentTime2-previousTime2>=dt2){
      digitalWrite(pul2,0);
      previousTime2=currentTime2;
      currentPos2=(direction2==1) ? currentPos2+1 : currentPos2-1;
      targetPos2--;

    }
    
  }

  if (targetPos3>0){

    currentTime3=micros();

    digitalWrite(pul3,1);

    if (currentTime3-previousTime3>=dt3){
      digitalWrite(pul3,0);
      previousTime3=currentTime3;
      currentPos3=(direction3==1) ? currentPos3+1 : currentPos3-1;
      targetPos3--;

    }
    
  }

  if (targetPos4>0){

    currentTime4=micros();

    digitalWrite(pul4,1);

    if (currentTime4-previousTime4>=dt4){
      digitalWrite(pul4,0);
      previousTime4=currentTime4;
      currentPos4=(direction4==1) ? currentPos4+1 : currentPos4-1;
      targetPos4--;

    }
    
  }
}

