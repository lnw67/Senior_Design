/*robotdiagnostics V1.1
 * Made to control the robot via a wireless serial connection
 * 
 * Brandon Malone
 * 1/31/2016
 *Version history... 1.1 added commands 6 and 7
 *Version history... 1.2 reset center to 1860
                         added a incremental change to turn rate
                         and universal variable iS2Current
                     1.3 added stepper and IR sensor dependant commands
                         as well as fnIRRead function
                         doubled the turn slowness
                     1.4 removed turn slowing
                     1.5 accounted for reversed motors following post amp
                         added RLYPostAmp
                         deleted "#define laser 8 "
                         added #12 and #13 commands
                     1.6 REMOVED
                     1.7 added fnGyroTurn and added overturn code for left90 to center 
                         correction
 */




//***************************************************************************************
//#define servotran A0       //analog read for the servo transistor  
////temp//#define midtran   A1       //analog read for the dead center photo transistor
//#define servo     3        //PWM signal for servo mounted phototransistor
#define S1        DAC1       //foward motion on sabertooth
#define S2        DAC0       //turn motion on sabertooth
//#define laser     8        //high for laser on 
//#define redled    9        //high for red led on
//#define yelled    10       //high for yellow led on 
//#define bluled    11       //high for blue led on
//#define mtrdrive  12       //switch motor drive pin to LOW for UNO control
////***************************************************************************************
//relays
#define RLYCap    7        //high to "ground out" integrating cap
#define RLYMotor  5        //low to enable the motor driver, high to disable
#define RLYPostAmp 8       //post amplifier RLY, high for low sensitivity, low for high sensitivity         1.5

#define bkIRRead  A1 

//***************************************************************************************
//nav
//***************************************************************************************

int right90        = 1050;      //12 bit analogwrite value to command a 90 degree right turn
int left90         = 3000;
int center         = 2048;

//Review for later***********************************************************************
int FWD            =2025;
int REV            =2250; 
int STP            =2100;
int Turndelay      =4000;
int TurnDelay     =4000;

//***************************************************************************************
//stepper/IR SENSOR specific variables and definitions (from Nstepper1_3.ino)

#define stpdir  24
#define stp  22
#define Switch  3
#define SIRsense  A2

#define stpM3 26
#define stpM2 28
#define stpM1 30
#define stpEN 32
#define stpSLP 34

int iSMCorrect = 10; //Correction from the switch to 90 deg right
int iSMPos = 0; //Current stepper position
int iSMCMD = 0;



//***************************************************************************************
//variables
int iCMD;
int iVAL;
int iS2Current;
int iTemp5;
int iTemp6;
int iTemp1;

//Review Later***************************************************************************

//***************************************************************************************
//diagnostic spicific variables
boolean bRLYCap = false;
boolean bRLYPostAmp = true;




void setup()
{
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(RLYCap, OUTPUT);
  pinMode(RLYMotor, OUTPUT);
  pinMode(RLYPostAmp, OUTPUT);
  pinMode(3, OUTPUT);   // templine
  
  analogWriteResolution(12);
  analogReadResolution(12);
  digitalWrite(RLYMotor,HIGH); // RESET INTEGRATOR WITH MOTOR DISABLED
  digitalWrite(RLYPostAmp,HIGH); //POST AMP RELAY SET TO LOW SENSITIVITY MODE
  digitalWrite(3, HIGH); //templine
  analogWrite(S1, 2100);
  analogWrite(S2, center);
  iS2Current = center;
  digitalWrite(RLYCap, HIGH);
  delay(5000);
  digitalWrite(RLYCap, LOW);
  digitalWrite(RLYMotor, LOW); // ENABLE MOTOR
  Serial.begin(9600);
  
  
  // ****** DROP from Nstepper1_3.ino
  pinMode(stpdir, OUTPUT);
  pinMode(stp , OUTPUT);
  pinMode(Switch, INPUT);
  pinMode(stpM3, OUTPUT);               
  pinMode(stpM2, OUTPUT);
  pinMode(stpM1, OUTPUT);
  pinMode(stpEN, OUTPUT);
  pinMode(stpSLP, OUTPUT);
  
  digitalWrite(stpEN, LOW);
  digitalWrite(stpM1, LOW);
  digitalWrite(stpM2, HIGH);
  digitalWrite(stpM3, LOW);
  digitalWrite(stpSLP, LOW);
  delay(1000);
  digitalWrite(stpSLP, HIGH);
  
  
  digitalWrite(stpdir, LOW);
  
  /*this portion of the setup calibrates the stepper motor*/
  
  do
  {
    digitalWrite(stp , HIGH);   //drive right until you trigger the LH switch
    delay(1);
    digitalWrite(stp , LOW);
    delay(1);
  }while(digitalRead(Switch)==HIGH);
  
  digitalWrite(stpdir, HIGH);
  
  for(int i = iSMCorrect; i > 0; i--) //drive left until you reach the correction factor
  {
    digitalWrite(stp , HIGH);
    delay(1);
    digitalWrite(stp , LOW);
    delay(1);
  }

  /*this portion of the setup moves robot from a-b and resets gyro to north*/

  analogWrite(S2, 1900);
  delay(200);
  analogWrite(S2, center);
  delay(200);

  analogWrite(S1, FWD);
  do
  {
    delay(20);
    iTemp1 = analogRead(bkIRRead);
  }while(iTemp1 > 1100); //VARIABLE 
  analogWrite(S1, STP);
  delay(1000);

  fnGyroTurn(left90);
  delay(2000);

  analogWrite(S1, FWD);
  do
  {
    delay(20);
    iTemp1 = analogRead(bkIRRead);
    Serial.println(iTemp1);
  }while(iTemp1 > 1100); //VARIABLE 
  analogWrite(S1, STP);

  analogWrite(S1, REV);
  do
  {
    delay(20);
    iTemp1 = analogRead(bkIRRead);
  }while(iTemp1 < 3000); //VARIABLE 
  analogWrite(S1, STP);
  delay(2000);


  
  Serial.print("Resetting RLYcap for 1 sec, S2 set to ");
  Serial.println(center);
  analogWrite(S2, center);
  digitalWrite(RLYMotor, HIGH);
  digitalWrite(RLYCap, HIGH);
  delay(5000);
  digitalWrite(RLYCap, LOW);
  delay(100);
  digitalWrite(RLYMotor, LOW);
  
  bRLYCap = false;
  
  iS2Current = center;

  analogWrite(S2, 1900);
  delay(200);
  analogWrite(S2, center);
  delay(200);


// point b - e
  analogWrite(S1,FWD);  
  iTemp5 = 0;   
  do                    // Move forward until the IR sensor reads 
  {
   iTemp5 = fnIRRead(100);
  }while(iTemp5 < 2800);
  
  analogWrite(S1, STP);
  delay(TurnDelay);
 //point e to f
 fnGyroTurn(right90);
 delay(TurnDelay);
 analogWrite(S1,FWD);
 int iTemp5 = 4000;
 do
 {
   iTemp5 = fnIRRead(200);
 }while(iTemp5 > 2000);

 iTemp5 =0;
 do
{
   iTemp5= fnIRRead(180);
} while(iTemp5 < 2000);
analogWrite(S1,STP);
delay(TurnDelay);
fnGyroTurn(center);
delay(TurnDelay);
  //********* END  
  

}

void loop()
{
  
  
/*This is a diagnostic program to help everyone understand how our robot functions 
 * and how to make it navigate. I will add more sensor functionality in later versions,
 * but this one will allow you to move to robot around by sending values through the serial 
 * monitor (under the tools menu up top)...
 * 
 * following the setup, the robot should be stationary, the motor driver is enabled, and the 
 * gyro integrator is active...
 * 
 * To review, our robot is driven by a rate gyro. The output of that gyro is fed to an integrating
 * op amp circuit. This integrator can be reset (the capacitor grounded out), by the "RLYcap" pin. 
 * 
 * Triggering RLYCap and opening it back up resets what the whole robot consideres "center"
 * 
 * In order to turn our robot, we analogWrite a value to "S2". This pin is an on board Digital to Analog converter
 * However, that output isn't actually fed to our motor driver. Instead, it goes to an instrumentation
 * amplifier (nominal 2.5v). The gyro output is also fed to the instrumentation amp (5.0V when the voltage on 
 * the integration amp is 0v) 
 * 
 * So, at this point, the inst. amp is saying 5.0V minus 2.5V and this output is sent to S2 on the motor controller
 * which tells the motor controller to drive both treads at the same time. So, if you power up the robot,
 * at this point in the code, you could pick up the robot, turn it, and the robot will "correct" its self
 * and point back in the same direction you started. That is because when you turn it, the capacitor charges
 * sending a voltage other than 2.5V to the motor controller differential pin (S2).
 * 
 * If we command a different value, other than 2000 (about 2000 = "center") to the S2 pin, it will again
 * make an output other than 2.5 at the inst. amplifier meaning the motor driver will command to turn, 
 * a charge will then accumulate on the integration capacitor until the output of the inst. amplifer is again, 2.5v
 * 
 * So say we have made a turn, but then we do digitalWrite(RLYcap, HIGH). The robot would start turning but 
 * it would never stop... please don't do that...
 * 
 * S1 (FWD/BACKWARD pin) is pretty simple. It does feed directly to the motor driver, however it is pretty sensitive.
 * a value of about 2000 stops it, 1900 makes it go one way pretty quck, 2100 makes it go the other way pretty quick.
 * 
 * The other relay is RLYMotor when you do digitalWrite(RLYMotor, HIGH) it DISABLES the motor driver. When the driver is 
 * ENABLED, the Red LED on the board will be on...
 */
  

  //prompt
  Serial.print("Awaiting command, enter '1' to see a list of commands...");
  
  //wait for command
  do
  {
    delay(100);
  }while(Serial.available() == false);

  //recieve command
  iCMD = Serial.parseInt();
  Serial.println();
  
//***************************************************************************************  
  //Menu
  if (iCMD == 1)
  {
    Serial.println(" 1 = Menu ");
    Serial.println(" 2 = Reset RLY cap and center S2");
    Serial.println(" 3 = Set turn S2");
    Serial.println(" 4 = Drive FWD ");
    Serial.println(" 5 = Drive REV ");
    Serial.println(" 6 = DIABLE motor");
    Serial.println(" 7 = ENABLE motor");
    Serial.println(" 8 = Change center value");
    Serial.println(" 9 = Read IR sensor ");
    Serial.println(" 10 = Drive FWD until IR goes low ");
    Serial.println(" 11 = Drive FWD until IR goes high");
    Serial.println(" 12 = Toggle RLY cap");
    Serial.println(" 13 = Toggle RLY post amp");
  }
  
//***************************************************************************************
  //Reset relay cap
  else if (iCMD == 2)
  {
    Serial.print("Resetting RLYcap for 1 sec, S2 set to ");
    Serial.println(center);
    analogWrite(S2, center);
    digitalWrite(RLYMotor, HIGH);
    digitalWrite(RLYCap, HIGH);
    delay(5000);
    digitalWrite(RLYCap, LOW);
    delay(100);
    digitalWrite(RLYMotor, LOW);
    
    bRLYCap = false;
    
    iS2Current = center;
    
  }
//***************************************************************************************
  //set turn (S2 variable)
  else if (iCMD == 3)
  {
    int iVAL2;
    Serial.print("Enter commanded value...");
    do
    {
      delay(100);
    }while(Serial.available() == false);
  
    //recieve command
    iVAL = Serial.parseInt();
    Serial.print("Writing ");
    Serial.println(iVAL2);
    fnGyroTurn(iVAL2);
    
//    int i;
//   /* ANTI JERKAGE CODE!!!! */   
//    if(iVAL <= iS2Current)                      //if turning right
//    {
//      do
//      {
//        i = iS2Current - iVAL;
//        if(i > 100)                             //if diff between current S2 and commanded > 100
//        {
//          iS2Current = iS2Current - 100;        //incrimentally update S2 
//          analogWrite(S2, iS2Current);
//          delay(75);                           //delay long enough for robot to move
//        }
//      }while(i > 100);                          //while that difference is greater that 100
//    }
//    
//    
//    else                                        //if robot turning left same thing
//    {
//      do
//      {
//        i = iVAL - iS2Current;
//        if(i > 100)
//        {
//          iS2Current = iS2Current + 100;
//          analogWrite(S2, iS2Current);
//          delay(75);
//        }
//      }while(i > 100);
//    }
//    
//    iS2Current = iVAL;                          //final update
//    analogWrite(S2, iVAL);                      //writing remainder
    
    
  }
  
  
  
  
//***************************************************************************************
  //drive fwd
  else if (iCMD == 4)
  {
    Serial.print("For how long, in mS...");
    do
    {
      delay(100);
    }while(Serial.available() == false);
  
    //recieve command
    iVAL = Serial.parseInt();
    Serial.println();

    analogWrite(S1, FWD);
    delay(iVAL);           
    analogWrite(S1, STP);
  }



//***************************************************************************************  
  //drive backward
  else if (iCMD == 5)
  {
    Serial.print("For how long, in mS...");
    do
    {
      delay(100);
    }while(Serial.available() == false);
  
    //recieve command
    iVAL = Serial.parseInt();
    Serial.println();

    analogWrite(S1, REV);
    delay(iVAL);           
    analogWrite(S1, STP);
  }



//***************************************************************************************
  //disable motor
  else if (iCMD == 6)
  {
    digitalWrite(RLYMotor, HIGH);
    Serial.println("Motor disabled");
  }
  
  
  
  
//***************************************************************************************  
  //enable motor
  else if (iCMD == 7)
   {
     digitalWrite(RLYMotor,LOW);
     Serial.println("Motor Enabled");
   }
   
//***************************************************************************************
  //change "center"
   else if (iCMD == 8)
   {
     Serial.print("To what bearing...");
    do
    {
      delay(100);
    }while(Serial.available() == false);
  
    //recieve command
    iVAL = Serial.parseInt();
    Serial.println(iVAL);
    
    center = iVAL;
   }
   
   
   
//***************************************************************************************
   //read IR 
   else if (iCMD == 9)
   {
    Serial.print("what bearing...");
    do
    {
      delay(100);
    }while(Serial.available() == false); 
    
    //recieve command
    iVAL = Serial.parseInt();
    Serial.println(iVAL);
    
    iTemp5 = fnIRRead(iVAL);
    Serial.print("At bearing ");
    Serial.print(iVAL);
    Serial.print(" Range is: ");
    Serial.println(iTemp5);   
   } 

   
   
   
//***************************************************************************************
   //Drive FWD to opening   
   
   
   else if (iCMD == 10)

   {
    Serial.println("Drive FWD until read an opening");
    Serial.print("what angle...");
    do
    {
      delay(100);
    }while(Serial.available() == false); 
    
    //recieve command
    iVAL = Serial.parseInt();
    Serial.println(iVAL);
    
    
    Serial.print("at what threshold...");
    do
    {
      delay(100);
    }while(Serial.available() == false); 
    
    //recieve command
    iTemp6 = Serial.parseInt();
    Serial.println(iTemp6);
    
    analogWrite(S1, FWD);    
    do
    {
      iTemp5 = fnIRRead(iVAL);
      Serial.print("At bearing ");
      Serial.print(iVAL);
      Serial.print(" Range is: ");
      Serial.println(iTemp5);
    }while(iTemp5 >iTemp6);
            
    analogWrite(S1, STP);
    
    
   }
   
       
//***************************************************************************************
   //Drive FWD to wall
        
   else if (iCMD == 11)

   {
    Serial.println("Drive FWD until read a wall");
    Serial.print("what angle...");
    do
    {
      delay(100);
    }while(Serial.available() == false); 
    
    //recieve command
    iVAL = Serial.parseInt();
    Serial.println(iVAL);
    
    
    Serial.print("at what threshold...");
    do
    {
      delay(100);
    }while(Serial.available() == false); 
    
    //recieve command
    iTemp6 = Serial.parseInt();
    Serial.println(iTemp6);
    
    analogWrite(S1, FWD);    
    do
    {
      iTemp5 = fnIRRead(iVAL);
      Serial.print("At bearing ");
      Serial.print(iVAL);
      Serial.print(" Range is: ");
      Serial.println(iTemp5);
    }while(iTemp5 < iTemp6);
            
    analogWrite(S1, STP);
    
    
   }  
  
//***************************************************************************************
   //Toggle RLY Cap
        
   else if (iCMD == 12)
   {
     if (bRLYCap == true)
     {
       Serial.println("Setting RLYCap to LOW");
       digitalWrite(RLYCap, LOW);
       bRLYCap = false;
     }
     else
     {
       Serial.println("Setting RLYCap to HGIH");
       digitalWrite(RLYCap, HIGH);
       bRLYCap = true;
     }
   }
   
   
//***************************************************************************************
   //Toggle RLY PostAmp
        
   else if (iCMD == 13)
   {
     if (bRLYPostAmp == true)
     {
       Serial.println("Setting RLYPostAmp to LOW");
       digitalWrite(RLYPostAmp, LOW);
       bRLYPostAmp = false;
     }
     else
     {
       Serial.println("Setting RLYPostAmp to HGIH");
       digitalWrite(RLYPostAmp, HIGH);
       bRLYPostAmp = true;
     }
   }
   
  
//***************************************************************************************  
   
  else
  {
    Serial.println("invalid entry dummy");
  }
  
  

  

}





//***************************************************************************************
//                 IR (top sensor) READING FUNCTION. PASS IN THE BEARING
//                WITH 0 BEING RIGHT, 100 BEING CENTER, AND 200 BEING LEFT
//                  RETURNS A DISTANCE MEASUREMENT (HIGH VALUE = CLOSER)
//                  PULLED FROM PROGRAM Nstepper1_3.ino 3/8/2016
//                                                  -Brandon
//***************************************************************************************
int fnIRRead(int iSMCMD)
{
  
  if(iSMCMD >= 210) //protection, if an overstep is commanded the stepper should not move. (1.3)
  {
    iSMCMD = iSMPos;
  }
  
  
  int iTemp1; //Review for later iterations
  int iTemp4;

  if ( iSMCMD < iSMPos)
  {
    digitalWrite(stpdir,LOW); //Clock wise motion
    iTemp1 = iSMPos - iSMCMD;
    for ( int i = iTemp1;i > 0 ; i--)
    {
      digitalWrite(stp ,HIGH);
      delay(1);
      digitalWrite(stp ,LOW);
      delay(1);
    }
    iSMPos = iSMCMD; 

  } 
  if (iSMCMD > iSMPos)
  {
    digitalWrite(stpdir,HIGH); //Counter Clock wise motion
    iTemp1 = iSMCMD - iSMPos; 
    for ( int i = iTemp1; i > 0 ; i-- )
    {
      digitalWrite(stp ,HIGH);
      delay(1);
      digitalWrite(stp ,LOW);
      delay(1);
    }
    iSMPos = iSMCMD; 
  }
  
  
  delay(25);  //delay added for accuracy (1.3)

  iTemp4 = analogRead(A2);
  return iTemp4;
  
}

void fnGyroTurn(int iVAL)   //1.7
{
  int i;
  bool bOverTurn = false;
  int iVALOverTurn;
  if(iS2Current > 2100)
  {
    bOverTurn = true;
    iVALOverTurn = iVAL - 300;
  }
 /* ANTI JERKAGE CODE!!!! */   
  if(iVAL <= iS2Current)                      //if turning right
  {
    if(bOverTurn == false)
    {
      do
      {
        i = iS2Current - iVAL;
        if(i > 100)                             //if diff between current S2 and commanded > 100
        {
          iS2Current = iS2Current - 100;        //incrimentally update S2 
          analogWrite(S2, iS2Current);
          delay(75);                           //delay long enough for robot to move
        }
      }while(i > 100);                          //while that difference is greater that 100
    }
    else
    {
      do
      {
        i = iS2Current - iVALOverTurn;
        if(i > 100)                             //if diff between current S2 and commanded > 100
        {
          iS2Current = iS2Current - 100;        //incrimentally update S2 
          analogWrite(S2, iS2Current);
          delay(75);                           //delay long enough for robot to move
        }
      }while(i > 100);                          //while that difference is greater that 100
    }
  }
  
  
  else                                        //if robot turning left same thing
  {
    do
    {
      i = iVAL - iS2Current;
      if(i > 100)
      {
        iS2Current = iS2Current + 100;
        analogWrite(S2, iS2Current);
        delay(75);
      }
    }while(i > 100);
  }
  
  iS2Current = iVAL;                          //final update
  analogWrite(S2, iVAL);                      //writing remainder

}









