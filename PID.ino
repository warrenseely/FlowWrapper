void feederPID(void) //calculate PID control for the feeder chain speed
{
  long proportional = 0, integraltemp = 0, derivative = 0, intround = 0;

  //ensure we round the correct direction
  if(feeder_error >= 0)
    intround = 0.5;
  else
    intround = -0.5;

  //reset if we cross zero
  if(((lastfeeder_error < 0) && (feeder_error > 0)) || ((lastfeeder_error > 0) && (feeder_error < 0)))
  {
    integral = 0; //reset to current error
  }
    
  //proportional
  proportional = kp * feeder_error; //proportional value * error in the encoder counts

//  //integral
  integral = integral + (feeder_error); //sum of the errors for all time
  integraltemp = ki * integral; //integral scalar

  //derivative
  derivative =  kd * ((feeder_error - lastfeeder_error)); //change in error over time
  lastfeeder_error = feeder_error; //update
  
  float temp = (proportional + integraltemp + derivative) + intround; //new feeder speed. add 0.5 so feeder_speed truncates to nearest integer
  
  feeder_speed += temp; //add it to the current feeder speed

  //out of range check
  if(feeder_speed > 200)
    feeder_speed = 200;
  else if(feeder_speed < 3)
    feeder_speed = 3;
}

//void JawPID(void) //calculate PID control for the feeder chain speed
//{
//  unsigned long int proportional = 0, integral = 0, derivative = 0;
//  
//  //proportional
//  proportional = kp * jawTime; //new jaw speed = proportional value * error in the times
//
////  //integral
////  integral += chaintime; //sum of the errors
////  integral *= ko; //integral scalar
//
//  //derivative
//  //derivative = (lastfeeder_speed - feeder_speed) * kd; //change in speed
//
//  float temp = proportional + integral + derivative; //new feeder speed in microseconds
//  temp = map(temp, 0, maxchainTime, 15, 155); //
//  jawpwm = constrain(temp, 15, 155); //0 = 0v, 255 = 10v = full speed!
//}

