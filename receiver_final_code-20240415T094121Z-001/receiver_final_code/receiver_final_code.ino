char direct=0;
int motorIn1=9;
int motorIn2=10;
int motorIn3=11;
int motorIn4=12;

void setup() {
  Serial.begin(9600);      
  pinMode(motorIn1,OUTPUT);
  pinMode(motorIn2,OUTPUT);
  pinMode(motorIn3,OUTPUT);
  pinMode(motorIn4,OUTPUT); 
 digitalWrite(motorIn1,LOW);
 digitalWrite(motorIn2,LOW);
 digitalWrite(motorIn3,LOW);
 digitalWrite(motorIn4,LOW);
}
void forward(){
  digitalWrite(motorIn1,LOW);//LEFT WHEEL
  digitalWrite(motorIn2,HIGH);
  digitalWrite(motorIn3,LOW);//RIGHT WHEEL
  digitalWrite(motorIn4,HIGH);
}
void backward(){
  digitalWrite(motorIn1,HIGH);//LEFT WHEEL
  digitalWrite(motorIn2,LOW);
  digitalWrite(motorIn3,HIGH);//RIGHT WHEEL
  digitalWrite(motorIn4,LOW);
}
void leftTurn(){
  digitalWrite(motorIn1,HIGH);//LEFT WHEEL
  digitalWrite(motorIn2,LOW);
  digitalWrite(motorIn3,LOW);//RIGHT WHEEL
  digitalWrite(motorIn4,HIGH);
}
void rightTurn(){
  digitalWrite(motorIn1,LOW);//LEFT WHEEL
  digitalWrite(motorIn2,HIGH);
  digitalWrite(motorIn3,HIGH);//RIGHT WHEEL
  digitalWrite(motorIn4,LOW);
}
void stops(){
  digitalWrite(motorIn1,LOW);
  digitalWrite(motorIn2,LOW);
  digitalWrite(motorIn3,LOW);
  digitalWrite(motorIn4,LOW);

}
void  loop() {
  if(Serial.available()>0){  
   direct=Serial.read();
    
  if(direct == 'B'){
    Serial.println("Backward");
    backward();
    }
  else if (direct=='F'){
    Serial.println("Forward");
    forward();
    }
  else if(direct == 'R'){
    Serial.println("Right");
    rightTurn();
    }
  else if(direct == 'L'){
    Serial.println("left");
    leftTurn();
    }
  else if(direct=='S'){
    Serial.println("Stop");
    stops();
    }
  else{
    direct=0;
    }
  }
}
    
  


 
