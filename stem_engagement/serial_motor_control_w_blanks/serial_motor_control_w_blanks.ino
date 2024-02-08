int enA = 10;  // PWM (Pulse Width Modulation) for motor A
int in1 = 9; 
int in2 = 8;

int enB = 6;   // PWM for motor B
int in3 = 5;
int in4 = 4;

int serial_input = 'q';   // variable to store serial input
int motor_speed = 128;    // current motor speed

void setup() {    // Setup code runs once at the beginning

  pinMode(enA, __);       
  pinMode(enB, __);

  pinMode(in1, __);    
  pinMode(in2, __);
  pinMode(in3, __);       
  pinMode(in4, __);

  analogWrite(enA, motor_speed);
  analogWrite(enB, motor_speed);

  Serial.begin(9600);    // Serial Baud rate is 9600 bits/sec

}

void loop() {   // Loop code keeps running

  if (Serial.available() > 0) { // if there's serial data coming in
    serial_input = Serial.read();  // read the received data 
    
    if(serial_input == 'w'){         // drive forward 
      digitalWrite(in1, __);
      digitalWrite(in2, __);
      digitalWrite(in3, __);
      digitalWrite(in4, __);
      Serial.print("Forward\r\n");  // print "Forward"
    }

    else if(serial_input == 's'){    // drive backward 
      digitalWrite(in1, __);
      digitalWrite(in2, __);

      digitalWrite(in3, __);
      digitalWrite(in4, __);
      Serial.print("Reverse\r\n");
    }

    else if(serial_input == 'a'){    // turn left 
      digitalWrite(in1, __);
      digitalWrite(in2, __);

      digitalWrite(in3, __);
      digitalWrite(in4, __);
      Serial.print("Left\r\n");
    }

    else if(serial_input == 'd'){    // turn right
      digitalWrite(in1, __);
      digitalWrite(in2, __);

      digitalWrite(in3, __);
      digitalWrite(in4, __);
      Serial.print("Right\r\n");
    }
    
    else if (serial_input == 'q') {   // stop
      digitalWrite(in1, __);
      digitalWrite(in2, __);

      digitalWrite(in3, __);
      digitalWrite(in4, __);
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
