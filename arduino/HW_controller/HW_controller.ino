
    /////////////simple elevator////////////////
   // BY : Abdelraouf Hawash                 //
  // DATE : 27/11/2022                      //
 // GMAIL : abdelraouf.hawash@gmail.come   //
////////////////////////////////////////////



///// initializing hardware variables
const int pwm_motor_pin = 10; // motor pins
const int inA_motor_pin = 8 ;  
const int inB_motor_pin = 9 ;

int ultra_echoPin = 2;        //ultrasonic HC-SR04
int ultra_trigPin = 3;

const int servo_pin = 4;      // servo pin (we will control servo manualy to use pin 10 as PWM)


//// variables
//motor
int max_pwm = 0;          //maximum pwm command for motors
double max_elevator_pos = 0.70;    // maximum position for elevator
double min_elevator_pos = 0.12;    // maximum position for elevator
//servo
double target_servo_pos = 90;   // servo goal position
int curent_servo_pos = 90;      // curent elevator position
const int max_servo_pos = 178;  // maximum position for servo
const int min_servo_pos = 2;    // maximum position for servo
// time variables
const int loop_time = 100 ;     //Looptime in millisecond
unsigned long lastMilli = 0;    //time history
// initializing PID
double Kp = 0;    // PID parameters initial value is zero
double Ki = 0;
double Kd = 0;
unsigned long currentTime, previousTime;
double elapsedTime, error, lastError, cumError, rateError;
double curent_elevator_pos;       //curent elevator position
double target_elevator_pos = 0.3; // goal position
double speed_pwm = 0 ;            //PWM commond for the motor


void setup() {

  //initialization serial for communication
  Serial.begin(115200);
  Serial.setTimeout(10);

  
  // motor pins mode
  pinMode (pwm_motor_pin, OUTPUT); // motor
  pinMode (inA_motor_pin, OUTPUT); 
  pinMode (inB_motor_pin, OUTPUT);
  
  //ultrasonic pins
  pinMode(ultra_trigPin, OUTPUT);
  pinMode(ultra_echoPin, INPUT);

  //servo pins
  pinMode(servo_pin,OUTPUT);
  servo_write(target_servo_pos,servo_pin);
  
}


void loop() {

  // read and write data
  if (Serial.available()){
    
    // read
    String Data_cmd = Serial.readString();
    
    // update cmd
    update_cmd(Data_cmd);
    
    //write
    String Data = String(curent_elevator_pos) + "," + String(curent_servo_pos) + "," + String(round(speed_pwm));
    Serial.print(Data);
    
  }

  if((millis()-lastMilli) >= loop_time){

    lastMilli = millis();
    
    // update elevator position
    curent_elevator_pos = ultrasonic_distance();
    
    // motor action
    computePID();
    set_motor(round(speed_pwm),pwm_motor_pin,inA_motor_pin,inB_motor_pin);


    // servo action
    if(abs(curent_elevator_pos - target_elevator_pos) <= 0.01 ){
      
      servo_write(target_servo_pos,servo_pin);
      curent_servo_pos = target_servo_pos;
    }
    
  } // end of timed loop
} // end of main loop



//update commands received like requested elevator position and other
void update_cmd(String input){
  
  // input should be : target_elevator_pos,target_servo_pos
  char separator = ',';
  int maxIndex = input.length() - 1;
  int cmd_index = 0;

  String Value = "";
  
  for(int i= 0 ; i <= maxIndex ; i++){
    
    Value.concat(input[i]);
    
    if (input[i] == separator || i == maxIndex)
    {
        Value.trim();

        if (cmd_index == 0) target_elevator_pos = constrain(Value.toDouble(), min_elevator_pos , max_elevator_pos); 
        if (cmd_index == 1) target_servo_pos = constrain(Value.toDouble(), min_servo_pos , max_servo_pos);
        if (cmd_index == 2) max_pwm = constrain(Value.toInt(), 0 , 250);
        if (cmd_index == 3) Kp = Value.toDouble();
        if (cmd_index == 4) Ki = Value.toDouble();
        if (cmd_index == 5) Kd = Value.toDouble();
        
        cmd_index++;
        Value = "";
    }
  }
}

// PID control function
void computePID(){

  currentTime = millis();                             //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation
  error = target_elevator_pos - curent_elevator_pos;  // determine error
  cumError += error * elapsedTime;                    // compute integral
  rateError = (error - lastError)/elapsedTime;        //compute derivative
  double output = Kp*error + Ki*cumError + Kd*rateError;     //PID output
  speed_pwm = constrain(output,-max_pwm,max_pwm);
  
  previousTime = currentTime; //remember current time
  lastError = error;          //remember current error
  
}

// setting motors pwm
void set_motor( int PWM_vel ,int pwm_pin ,int inpA ,int inpB ){

  PWM_vel = constrain(PWM_vel,-max_pwm,max_pwm);
  
  if(PWM_vel > 0 ){
    analogWrite(pwm_pin , abs(PWM_vel) );
    digitalWrite(inpA , HIGH);
    digitalWrite(inpB , LOW);
  }
  else{
    analogWrite(pwm_pin , abs(PWM_vel) );
    digitalWrite(inpA , LOW);
    digitalWrite(inpB , HIGH);
  }
}


// calculate distance from ultrasonic sensor
double ultrasonic_distance(){
  
  digitalWrite( ultra_trigPin , LOW); 
  delayMicroseconds(2);
  digitalWrite(ultra_trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(ultra_trigPin, LOW);
  
  double duration = pulseIn( ultra_echoPin, HIGH); // duration in microseconds
  
  return duration / 2910.0 / 2.0 ;                 //distance in m

}

// Custom servo motor contorl function
void servo_write(int degree, int pin){

  degree = constrain(degree,min_servo_pos,max_servo_pos);
  int val = (degree*10.25)+500;
  int cycles = 5;
  
  for(int i=0; i< cycles; i++){
    digitalWrite(pin,HIGH);
    delayMicroseconds(val);
    digitalWrite(pin,LOW);
    delay(10);
  }
}
