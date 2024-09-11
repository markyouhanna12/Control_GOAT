
double kp = 2.0; //Proportional gain
double ki = 1.0; // Integral gain
double kd = 0.7; // Derivative gain

double setPoint = 50.0; //speed we want to move with (target_speed)
double input = 0;  //current speed 
double output = 0; //output of the motor

double integral = 0;
double previousError = 0;
unsigned long lastTime = 0;


// Exponential Smoothing Filter Parameters
// equation is y(n) = alpha * y(n-1) + (1-alpha)* x(n)
double alpha = 0.99;
double smoothedOutput = 0; // x(n)

const int motorPin = 8; //use the  Motor control pin (PWM)

void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  double elapsedTime = (double)(currentTime - lastTime); // elapsedTime is the (dt)
  
  // Read motor speed
  input = readMotorSpeed(); //this function we recalled it to return the the current speed of the motor
  
  //apply PID on the output 
  double error = setPoint - input;
  integral = integral + (error * elapsedTime); // integral is the integration of the error in elapsedTime (dt)
  double derivative = (error - previousError) / elapsedTime; // derivative is the difference between the error and the previous divided by the elapsedTime (dt) ( de(t)/d(t) )
  // the equation of the PID 
  output = (kp * error) + (ki * integral) + (kd * derivative);


  // Apply Exponential Smoothing Filter for soft start
  smoothedOutput = (alpha * output) + (1 - alpha) * smoothedOutput;

  // Update motor speed
  analogWrite(motorPin, constrain(smoothedOutput, 0, 255));

  // Store for next iteration to begin a new point to filter
  previousError = error;
  lastTime = currentTime;
  
  Serial.print("Setpoint: ");
  Serial.print(setPoint);
  Serial.print(" Input: ");
  Serial.print(input);
  Serial.print(" Output: ");
  Serial.println(smoothedOutput);

  delay(100);  // Loop delay to simulate processing time
}

  double readMotorSpeed() {
  // Return the current speed of the motor
  return analogRead(A0); // reading from analogPin on the microcontroller
}
