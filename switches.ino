void button_results()
{
  if(stopBtn.wasReleased() && currently_running && StartBtnState && !StopBtnState) //stop button pressed stop the machine
  {
      motorDrive(stopAll); //stop outputs
      StartBtnState = 0;
      StopBtnState = 1;
      currently_running = 0; //we are not currently running for subsequent loops
      pkgcount = 0; //reset
      homeflag = 0; //reset flag used for startup
      PCICR &= ~(1 << PCIE0); //disable pin change interrupts for port B
      PCICR &= ~(1 << PCIE2);    // disable pin change interrupts for D0 to D7
      PCICR &= ~(1 << PCIE1); //disable pin changei nterrupts for port C
      //Serial.println("STOP BUTTON PRESSED");
       writeLCD("STOP BUTTON PRESSED");
  }
  if(startBtn.wasReleased() && !currently_running && !StartBtnState && StopBtnState) //we are reading a press and release; if we have not been here prior
  {
    startupFlag = 1; 
    StartBtnState = 1;
    StopBtnState = 0;
    currently_running = 1;
    PCIFR |= bit (PCIF0); //clear any outstanding interrupts
    PCICR |= bit (PCIE0); //enable pin change interrupts for port B
    PCIFR |= bit (PCIF2);    // clear any outstanding interrupts
    PCICR |= bit (PCIE2);    // enable pin change interrupts for D0 to D7
    
    //enable pin change interrupts for feeder chain
    //other pin change interrupts enabled at end of startupProcedure()
    PCICR |= bit (PCIF1); //clear any outstanding interrupts for port C
    PCICR |= bit (PCIE1); //enable pin change interrupts for port C
  }
  else if(settingsBtn.wasReleased())
  {
    setBtnFlag = 1;
  }
}

