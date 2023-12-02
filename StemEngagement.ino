int AI1 = 15; // physical pins correspond to integer values in arduino
int AI2 = 16;
int BI1 = 17;
int BI2 = 4;

int PWMA = 11; // Pulse Width Modulation (PWM) pins on 11 and 3
int PWMB = 3;

int userInput = 0; // variable to store serial input
int motorSpeed = 0; // stores current motor speed

void setup() {
  pinMode(AI1, OUTPUT);       // set pin I/O for PWM Control
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);       
  pinMode(BI2, OUTPUT);
  pinMode(PWMA, OUTPUT);       
  pinMode(PWMB, OUTPUT);
  analogWrite(PWMA, motorSpeed); // motors start off
  analogWrite(PWMB, motorSpeed);
  Serial.begin(115200);    // setup serial monitor to read keyboard input


}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) { // check for input from the serial monitor
    userInput = Serial.read();  // store received charcacter 
    
    if(userInput == 'w'){ // set H-bridge inputs AI1-BI2 to drive forward 
      digitalWrite(AI1, HIGH);
      digitalWrite(BI1, HIGH);
      digitalWrite(AI2, LOW);
      digitalWrite(BI2, LOW);
      Serial.print("Forward\r\n"); // text formatting \r carriage return and \n newline
    }
    else if(userInput == 'a'){ // set H-bridge inputs AI1-BI2 to turn CCW 
      digitalWrite(AI1, LOW);
      digitalWrite(BI1, HIGH);
      digitalWrite(AI2, HIGH);
      digitalWrite(BI2, LOW);
      Serial.print("CCW\r\n");
    }
    else if(userInput == 's'){ // set H-bridge inputs AI1-BI2 to drive forward 
      digitalWrite(AI1, LOW);
      digitalWrite(BI1, LOW);
      digitalWrite(AI2, HIGH);
      digitalWrite(BI2, HIGH);
      Serial.print("Reverse\r\n");
    }
    else if(userInput == 'd'){ // set H-bridge inputs AI1-BI2 to drive CW 
      digitalWrite(AI1, HIGH);
      digitalWrite(BI1, LOW);
      digitalWrite(AI2, LOW);
      digitalWrite(BI2, HIGH);
      Serial.print("CW\r\n");
    }
    else if(userInput == 'r'){
      if(motorSpeed < 255){
        motorSpeed++;
        analogWrite(PWMA, motorSpeed);
        analogWrite(PWMB, motorSpeed);
        Serial.print("+\r\n");
      }
    }
    else if(userInput == 'e'){
      if(motorSpeed > 0){
        motorSpeed--;
        analogWrite(PWMA, motorSpeed);
        analogWrite(PWMB, motorSpeed);
        Serial.print("-\r\n");
      }
    }
    else{ // all other inputs turn off the motor
      digitalWrite(AI1, LOW);
      digitalWrite(BI1, LOW);
      digitalWrite(AI2, LOW);
      digitalWrite(BI2, LOW);
      Serial.print("Stop\r\n");
    }
  }
}
