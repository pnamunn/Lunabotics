int enA = 10;  // PWM (Pulse Width Modulation) for motor A
int in1 = 9; 
int in2 = 8;

int enB = 6;   // PWM for motor B
int in3 = 5;
int in4 = 4;

int serial_input = 'q';   // variable to store serial input
int motor_speed = 128;    // current motor speed

void setup() {    // Setup code runs once at the beginning

  pinMode(enA, OUTPUT);       
  pinMode(enB, OUTPUT);

  pinMode(in1, OUTPUT);    
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);       
  pinMode(in4, OUTPUT);

  analogWrite(enA, motor_speed);
  analogWrite(enB, motor_speed);

  Serial.begin(9600);    // Serial Baud rate is 9600 bits/sec

}

void loop() {   // Loop code keeps running

  if (Serial.available() > 0) { // if there's serial data coming in
    serial_input = Serial.read();  // read the received data 
    
    if(serial_input == 'w'){         // drive forward 
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);

      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      Serial.print("Forward\r\n");  // print "Forward"
    }

    else if(serial_input == 'a'){    // turn left 
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);

      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      Serial.print("Left\r\n");
    }

    else if(serial_input == 's'){    // drive backward 
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      Serial.print("Reverse\r\n");
    }

    else if(serial_input == 'd'){    // turn right
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      Serial.print("Right\r\n");
    }

    else if (serial_input == 'q') {   // stop
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);

      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      Serial.print("Stop\r\n");
    }
    
    else if(serial_input == 'e'){    // increase speed
      if(motor_speed < 255){
        motor_speed = motor_speed + 1;
        analogWrite(enA, motor_speed);
        analogWrite(enB, motor_speed);
        Serial.print("+ speed\r\n");
      }
    }
    else if(serial_input == 'r'){    // decrease speed
      if(motor_speed > 0){
        motor_speed = motor_speed - 1;
        analogWrite(enA, motor_speed);
        analogWrite(enB, motor_speed);
        Serial.print("- speed\r\n");
      }
    }

  }
}
