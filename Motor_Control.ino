void motorDrive(int state) 
  { 
    if(state == stopAll)
    {
      //send a motor stop command here
      digitalWrite(JawEnable_PIN, 0); //shut off vfd for jaws
      digitalWrite(PaperEnable_PIN, 0); //shut off vfd for paper
      digitalWrite(FeederEnable_PIN, 0); //shut off vfd for feeder chain
      analogWrite(feederPWM, 0); //feeder chain
      analogWrite(paperPWM, 0); //paper feed
      analogWrite(jawPWM, 0); //jaws
      //Serial.println("STOP BUTTON PRESSED");
      writeLCD("STOP BUTTON PRESSED");
    }
  }

  void startupProcedure(void)
  {
    switch(startupFlag) {
        case 1:    //align jaw
          if(!homeflag) //only want to write this once
          {
            homeflag = 1; //set flag
            jaw_speed = 50;//10;
            jawhomeflag = -1; //make sure this is set to -1 for startup
            analogWrite(jawPWM, jaw_speed); //write the speed of the film during cut(hardcode for homing)
            PORTC |= (1 << PORTC3); //enable the jaws
            //Serial.println("homing jaws");  
            writeLCD("homing jaws");          
          }
          if(jawhomeflag == 1) //have the jaws shut off via interrupt?
          {
            startupFlag = 2; //we saw the jaw reach home, now we can home the paper
            jawhomeflag = 0; //reset
            homeflag = 0; //reset for next state
            //Serial.println("finished homing jaws");
            writeLCD("finished homing jaws");
          }
        break;
        case 2: //align paper
          if(!homeflag) //only want to write this once
          {
            homeflag = 1; //set flag
            paperHome = 1; //so paper shuts off in ISR
            paperhomeflag = 0; //make sure this is set to zero
            analogWrite(paperPWM, 10); //write the speed during homing(hardcode for homing)
            PORTC |= (1 << PORTC4); //enable paper vfd
            //Serial.println("homing paper");
            writeLCD("homing paper");
          }
          if(paperhomeflag) //have we seen a register mark?
          {
            startupFlag = 3; //we saw the paper reach home, now we can home the feeder chain
            paperhomeflag = 0; //reset
            homeflag = 0; //reset for next state
            //Serial.println("finished homing paper");
            writeLCD("finished homing paper");
          }
        break;
        case 3: //align feeder chain
          if(!homeflag) //only want to write this once
          {
            homeflag = 1; //set flag for state change
            fingerHomeFlag = 1; //set flag for finger homing
            analogWrite(feederPWM, 60); //15 write the speed during homing(hardcode for homing)
            PORTC |= (1 << PORTC5); //enable feeder vfd, shut off by pin change interrupt on A0 when finger is seen
            //Serial.println("homing feeder");
             writeLCD("homing and priming feeder");
          }
          if((PIND & bit(PattySensor_PIN))) //pin goes HIGH when we see a patty
          {
            homeflag = 0; //reset flag
            startupFlag = 4; //we saw a patty, now we can home the feeder chain
          }
       break;
       case 4:
          if(!fingerHomeFlag) //have we seen a feeder finger??
          {
            homeflag = 0; //reset flag
            startupFlag = 5; //we saw the feeder reach home, now we are done homing machine
            //Serial.println("finished homing feeder");
             writeLCD("finished homing feeder");
            startTime = millis(); //preserve for startup delay
          }
       break; 
       case 5: //startup delay
          unsigned long currentTime = millis();
         if((unsigned long)(currentTime - startTime) >= startupDelay) //we have aligned all axis now we wait for two seconds(timed loop = 100ms X 20 = 2000ms = 2 seconds)
         {
            //Serial.println("Starting");
            writeLCD("Starting");
            //startupDelay = 0; //reset
            startupFlag = 0; //set flag so no longer enter this subroutine
            PCICR &= ~(1 << PCIE0); //disable pin change interrupts for port B
            PCICR &= ~(1 << PCIE2);    // disable pin change interrupts for D0 to D7 //disable interrupts
            FeederCount = 0; //reset to 0 for startup
            PaperCount = 0; //reset to 0 for startup
            PCIFR |= bit (PCIF0); //clear any outstanding interrupts
            PCIFR |= bit (PCIF2);    // clear any outstanding interrupts
            PCICR |= bit (PCIE0); //enable pin change interrupts for port B //reenable interrupts
            PCICR |= bit (PCIE2);    // enable pin change interrupts for D0 to D7
            feeder_count = 0; //reset to 0 for startup
            paper_count = 0; //reset to 0 for startup
            
            //get the starting speeds
            jaw_speed = (paper_speed * jawScalar) + 0.5; //start at scaled speed of paper, add 0.5 so it is rounded to nearest integer by truncation
            feeder_speed = (paper_speed * feederScalar) + 0.5; //start at scaled speed of paper, add 0.5 so it is rounded to nearest integer by truncation
            //now we set the starting PWMs
            analogWrite(jawPWM, jaw_speed); //write the speed of the film for run;23
            analogWrite(paperPWM, paper_speed); //write the speed for run;10; constant
            analogWrite(feederPWM, feeder_speed); //write the speed for run;52; this is only start speed, the PID takes control after this
            //Serial.print("test");
            //now we activate the vfds
            PORTC |= (1 << PORTC3); //enable the jaws vfd
            PORTC |= (1 << PORTC4); //enable paper vfd
            PORTC |= (1 << PORTC5); //enable feeder vfd
         }            
        break;
      }
  }


  void getSettings()
  {
    int dataloop = 0; //flag for data entry
    do
    {
      printData(6); //print prompt and get value
         
      if((setpoint < 48) || (setpoint > 505)) //out of range check
      {
        dataloop = 1;
        //Serial.println("Setpoint out of range! Please enter packages per minute between 48 and 505");
        writeLCD("Setpoint out of range! Please enter packages per minute between 48 and 505");
      }
      else //valid number
      {
        dataloop = 0;
      }
    }while(dataloop == 1);  

    paper_speed = (3.2998 * setpoint - 0.4401) * 0.1; //this equation obtained by graphing pwm vs jaw rpm; paper_speed is an int to truncate decimals. Multiply by 0.1 to scale pwm
    offset = 300 * (10 / paper_speed); //timing offset between finger and register; 300ms offset when paper_speed is 10(PWM). 
                                       //If paper_speed changes this should appropriately modify offset to match the new speed

    //map(setpoint, 48, 505, 5, 155); //convert the setpoint to pwm
    
    setBtnFlag = 0; //reset so do not enter subroutine again unless button pressed again
  }

