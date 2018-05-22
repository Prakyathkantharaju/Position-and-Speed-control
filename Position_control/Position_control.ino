// Initialising all the parameters
int count = 0;
int last_count;
float theta = 0;
int outputA, outputB;
float theta_desired = -720;
float Kp = 0.4;
float Kd = 0.25;
float Ki = 0.001;
float last_error = 0;
float error;
float total_error = 0;
float P_control, I_control, D_control;
float U_control;
float Motor_Power;
int state;
int last_state;

// Inital setup
void setup()
{
  Serial.begin(9600);
  // Rotary encoder pins
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), Encoder_Reading_A, CHANGE);
  // Motor PWM pins
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

// Loop
void loop()
{
  Encoder_Reading_A();
  Angle();
  Controller();
  Motor_Control();
  Print_Data();
}

// Controller
void Controller()
{
  error = theta_desired - theta;
  P_control = Kp * error;
  D_control = Kd * (error - last_error);
  total_error = total_error + error;
  if(total_error > 255)
  {
   // AntiWind Up control
    total_error = 0;
  }
  I_control = Ki * total_error;
  U_control = P_control + I_control + D_control;
  U_control = constrain(U_control, -255, 255);
  last_error = error;
}

// Calculation of Angle using encoder reading /needs to be scaled/
void Angle()
{
  // Rise in reading means one blade has cut the opto isolator 
  // once hence the blade has moved 45 degrees
  theta = theta + ((count - last_count) * 45);
}

// Motor Control
void Motor_Control()
{
  if(U_control >= 0)
  {
    // Indicates error is positive 
    Motor_Power = floor(U_control);
    analogWrite(5, Motor_Power);
    analogWrite(6, 0);
  }
  else
  {
    // Indicates error is negative i.e. we over shoot
    Motor_Power = -1*floor(U_control);
    analogWrite(5, 0);
    analogWrite(6, Motor_Power);
  }
}

// Encoder Reading
void Encoder_Reading_A()
{
  last_count = count;
  outputA = digitalRead(3);
  outputB = digitalRead(4);
  if(outputA == HIGH && outputB == HIGH)
  {
    state = 1;
  }
  if(outputA == HIGH && outputB == LOW)
  {
    state = 2;
  }
  if(outputA == LOW && outputB == HIGH)
  {
    state = 3;
  }
  if(outputA == LOW && outputB == LOW)
  {
    state = 4;
  }
  switch(state)
  {
    case 1:
    {
      if(last_state == 3)
      {
        count++;
      }
      if(last_state == 2)
      {
        count--;
      }
      break;
    }
    case 2:
    {
      if(last_state == 1)
      {
        count++;
      }
      if(last_state == 4)
      {
        count--;
      }
      break;
    }
    case 3:
    {
      if(last_state == 4)
      {
        count++;
      }
      if(last_state == 1)
      {
        count--;
      }
      break;
    }
    case 4:
    {
      if(last_state == 2)
      {
        count++;
      }
      if(last_state == 3)
      {
        count--;
      }
      break;
    } 
  }
  last_state = state;
}

// Print all data
void Print_Data()
{
  //Serial.println("Angle \t");
 //Serial.println(theta);
//  Serial.println("\t P_control \t");
//  Serial.println(P_control);
  
//  Serial.println("D_control \t");
//  Serial.println(D_control);
//  Serial.println("I_control \t");
  // Serial.println(I_control);

//  Serial.println("U_control \t");
//Serial.println(U_control);
 // Serial.println("Error \t");
  Serial.println(error);
//
//  Serial.println("Count \t");
//Serial.println(count);
//  Serial.println("Motor Power \t");
//  Serial.println(Motor_Power); 
}

