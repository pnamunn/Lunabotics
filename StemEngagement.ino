int AI1 = 15;
int AI2 = 16;
int BI1 = 17;
int BI2 = 4;

int PWMA = 11; 
int PWMB = 3;

int userInput = 0;

void setup() {
  pinMode(AI1, OUTPUT);       // set pin I/O for PWM Control
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);       
  pinMode(BI2, OUTPUT);
  pinMode(PWMA, OUTPUT);       
  pinMode(PWMB, OUTPUT);


  Serial.begin(115200);    // setup serial monitor to read keyboard input
  analogWrite(AI1, 255);
  analogWrite(AI2, 255);
  analogWrite(BI1, 255);
  analogWrite(BI2, 255);
  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) { // read the incoming byte:
    userInput = Serial.read();  // say what you got:
    if(userInput == 'w'){
      analogWrite(AI1, 0);
    }
    else if(userInput == 'a'){
      analogWrite(AI2, 0);
    }
    else if(userInput == 's'){
      analogWrite(BI1, 0);
    }
    else if(userInput == 'd'){
      analogWrite(BI2, 0);
    }
  }
}
