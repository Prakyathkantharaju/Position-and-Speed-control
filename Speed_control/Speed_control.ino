// Initialising all the parameters
float old_time_countangle ;
int count = 0;
int i =0;
int last_count;
float theta = 0;
int outputA, outputB;
float angular_vel_desired = 250;
float Kp = 1;
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
float delta_time;
float t;
float last_theta = 0;
float angular_vel;
float countangle;
float counttime;
#define LOOPTIME 1000
float last_t = 0;
float old_time_counttime;
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
   if((millis()-last_t) >= LOOPTIME)   
  {                                                                                 // enter timed loop
    last_t = millis();
    Angular_Velocity();                                                          // calculate speed                                        
  }
  //Angular_Velocity();
  Controller();
  Motor_Control();
  Print_Data();
}

// Controller
void Controller()
{
  error = angular_vel_desired - angular_vel;
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
void Angular_Velocity()
{
  // Rise in reading means one blade has cut the opto isolator 
  // once hence the blade has moved 45 degrees
//  delta_time = millis() - t;
//  t = t + delta_time;
  angular_vel = ((count - last_count)*(60*(1000/LOOPTIME)));
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
  //Serial.println("Angular Velocity ");
 // Serial.println(angular_vel);
 // Serial.println("Delta time");
 // Serial.println(delta_time);
 // Serial.println("Theta ");
 // Serial.println(theta);
//  Serial.println("\t P_control \t");
//  Serial.println(P_control);
  
//  Serial.println("D_control \t");
//  Serial.println(D_control);
//  Serial.println("\t I_control \t");
//  Serial.println(I_control);

//  Serial.print("\t U_control \t");
//  Serial.print(U_control);
//  Serial.println("Error \t");
  Serial.println(error);
//
//  Serial.println("Count \t");
//  Serial.println(count);
//  Serial.println("Motor Power \t");
//  Serial.println(Motor_Power); 
}

