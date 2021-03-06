void printData(int data) //pass in an integer to determine values to print
{
  switch(data){
      case 1: //print all data for troubleshooting
//        Serial.print("Paper setpoint pkg/min: "); //user given
//        Serial.println(setpoint); //send out on usb
//        Serial.print("Paper setpoint pkg/min actual: "); //calculated during runtime
//        Serial.print(integral);
        Serial.print(" "); //send out on usb
        Serial.print(feeder_speed); //sent out from PID
        Serial.print(" ");//timingScalarcopy: "); //real time count
        Serial.print(timingScalarcopy, 3); //send out on usb with 3 decimal points precision   
        Serial.print(" ");//papercount: "); 
//        Serial.print(paper_count);   
//        Serial.print(" ");//feedercount: ");
//        Serial.println(feeder_count);
//        Serial.print(" ");
        Serial.println(timeDifference);//
      break;
      case 2:
        Serial.print("Paper actual pkg/min: "); //calculated during runtime
        Serial.println(pkgPerMinute); //send out on usb
      break;
      case 3:
        Serial.print("Paper setpoint pkg/min: "); //user given
        Serial.println(setpoint); //send out on usb
      break;
      case 4:
        Serial.print("Pkg since last start: "); //real time count
        Serial.println(pkgcount); //send out on usb
      break;
      case 5:
        Serial.print("Time Difference: ");
        Serial.println(timeDifference); //the time difference between register and finger
      break;
      case 6:
        Serial.println("Enter desired packages per minute and press Enter: ");
        int flag = 1, i = 0;
        byte temp;
        char val[10];
      
        while(flag) //make sure we have data
        {
          if(Serial.available())
          {
              temp = Serial.read(); //sent as a char value
              val[i++] = temp; //move to array to process
              val[i] = '\0'; //terminate it
              setpoint = atoi(val); //convert to int and store in setpoint
              flag = 0;
          }
        }
      break;      
  }
}

//write a string to the LCD; Utilizes function overloading to determine input data type
void writeLCD(char *string, int Hcursor) //takes a string or int and writes to the LCD display. Set unused type to "0" or 0 respectively
{
  const unsigned char home_cursor[] = {27, '[', 'j', '\0'};
  //const unsigned char cursor_L2[] = {27, '[', '<' '\0'};

  if(Hcursor == 0) //0 means we are printing after nothing else and can home the cursor
  {
    for(int i = 0; i < 3; i++) //loop and send the home sequence
      Serial.write(home_cursor[i]);
  }
  else if(Hcursor == 1) //1 means we are printing after something so we want to start on next line
  {
    
  }
    
  Serial.print(string);
}

//write an int to the LCD; Utilizes function overloading to determine input data type
void writeLCD(int data, int Hcursor) //takes an int and writes to the LCD display. 
{
  const unsigned char home_cursor[] = {27, '[', 'j', '\0'};

  if(Hcursor == 0) //0 means we are printing nothing else and can home the cursor
  {
    for(int i = 0; i < 3; i++) //loop and send the home sequence
      Serial.write(home_cursor[i]);
  }
  else if(Hcursor == 1) //1 means we are printing after something so we want to start on next line
  {
    
  }
    
  Serial.print(data);
}

//write a float to the LCD; Utilizes function overloading to determine input data type
void writeLCD(float data, int Hcursor) //takes an int and writes to the LCD display. 
{
  const unsigned char home_cursor[] = {27, '[', 'j', '\0'};

  if(Hcursor == 0) //0 means we are printing nothing else and can home the cursor
  {
    for(int i = 0; i < 3; i++) //loop and send the home sequence
      Serial.write(home_cursor[i]);
  }
  else if(Hcursor == 1) //1 means we are printing after something so we want to start on next line
  {
    
  }
    
  Serial.print(data);
}

//write an int to the LCD; Utilizes function overloading to determine input data type
void writeLCD(long int data, int Hcursor) //takes an int and writes to the LCD display. 
{
  const unsigned char home_cursor[] = {27, '[', 'j', '\0'};

  if(Hcursor == 0) //0 means we are printing nothing else and can home the cursor
  {
    for(int i = 0; i < 3; i++) //loop and send the home sequence
      Serial.write(home_cursor[i]);
  }
  else if(Hcursor == 1) //1 means we are printing after something so we want to start on next line
  {
    
  }
    
  Serial.print(data);
}

void livePIDTune(void)
{
    if(Serial.available() > 0)
    {
      char incoming = Serial.read();
      if(incoming == 'd')
        kd+=0.1;
      else if((incoming == 'c') && (kd > 0))
        kd-=0.1;
      else if(incoming == 'i')
        ki+=0.0001;
      else if((incoming == 'k') && (ki > 0))
        ki-=0.0001;
      else if(incoming == 'p')
        kp+=0.001;
      else if((incoming == ';') && (kp > 0))
        kp-=0.001;

      //settings changed, print them out
      Serial.print(kp);
      Serial.print(", ");
      Serial.print(ki, 4); //print 4 decimal points precision
      Serial.print(", ");
      Serial.println(kd);
    }//end pid and feeder drive settings if statement
}

