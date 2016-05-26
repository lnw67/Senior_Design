//Robot Drive Program
//William Leverette & Muath Alomair
//April 8, 2016
//Version 1.8

/*Updates:
    Diagnostics 1.7
  Addition of FrontSensor functions
*/

//From Diagnogstics 1.8:

//***************************************************************************************
//#define servotran A0       //analog read for the servo transistor  
////temp//#define midtran   A1       //analog read for the dead center photo transistor
//#define servo     3        //PWM signal for servo mounted phototransistor
#define S1        DAC1       //foward motion on sabertooth
#define S2        DAC0       //turn motion on sabertooth
// Define for pins that communicate with Color sensor arduino
#define RedVictim 51        //RedVictim pin
#define YellowVictim 49     //YellowVictim pin
#define ContoNav 47     //
#define NavtoCon 45
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

int right90        = 1100;      //12 bit analogwrite value to command a 90 degree right turn
int left90         = 3000;
int center         = 2048;

//Review for later***********************************************************************
int FWD            =2025;
int REV            =2250; 
int STP            =2100;
int Turndelay      =4000;
int TurnDelay      =1000;

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
int iTemp1;
int iTemp5;
int iTemp6;

//Review Later***************************************************************************

//***************************************************************************************
//diagnostic spicific variables
boolean bRLYCap = false;
boolean bRLYPostAmp = true;


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
  
  
  delay(50);  //delay added for accuracy (1.3)

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

// Point e to b
  void fnGoeb()
  {
    analogWrite(S1,REV);
    do
    {
     iTemp6 = analogRead(bkIRRead);
     delay(2000);
    }
    while(iTemp6 < 3000);
    }
  

// Point b to e

  void fnGobe()
  {
    
       analogWrite(S1,FWD);    
 
    // Move forward until the IR sensor reads 
     
     
     
     do
     {
       iTemp5 = fnIRRead(100);
     }
     while(iTemp5 < 3000);
     
     analogWrite(S1, STP);
     
  }
  
  
// Point b to c
  void fnGobc() //VERIFIED
  {
    fnGyroTurn(right90);
    delay(TurnDelay);
    analogWrite(S1,FWD);
    
     // Move forward until the IR sensor reads 
     
     
     do
     {
       iTemp5 = fnIRRead(100);
     }
     while(iTemp5 < 2500);
     
     analogWrite(S1, STP);
     delay(TurnDelay);
     
  }



  

  
// Point c to b
 void fnGocb()
  {
     analogWrite(S1,REV);
    
        do
     {
       iTemp6 = analogRead(bkIRRead);   
       delay(2000);
     }
     while(iTemp6 < 2000);
     
     analogWrite(S1, STP);
     
     fnGyroTurn(left90);
     delay(TurnDelay);
  
  }
  
  //Point D to F
  
  void fnGodf()
  {
     analogWrite(S1,REV);
     do
     {
       iTemp5 = fnIRRead(200);
     }
     while(iTemp5 > 15000);
     
     analogWrite(S1,STP);

  //   fnGyroTurn(left90);
     delay(TurnDelay);
  }
  
  //Point E to G
  
  void fnGoeg() //VERIFIED
  {
    
    
    fnGyroTurn(right90);
    delay(TurnDelay);
    analogWrite(S1,FWD);
    iTemp5 = 0;
    
    do     // Move forward until the IR sensor reads 
    {
      iTemp5 = fnIRRead(100);
    }while(iTemp5 < 1500);
    
    analogWrite(S1, STP);
    delay(TurnDelay);

     
  }

  //Point E to D   
  void fnGoed()  //verified
  {
    fnGyroTurn(left90);
    delay(TurnDelay);
    analogWrite(S1,FWD);
    delay(1000);
    analogWrite(S1,STP);
    delay(TurnDelay);
  }

  //Point D to E
  void fnGode() //verified
  {
  analogWrite(S1,REV);
  iTemp6=0;
  do
  {
    iTemp6 = fnIRRead(100);
  }while(iTemp6 > 1100);
   
   analogWrite(S1,STP);
   fnGyroTurn(right90);
   
  }
 
  
  // Point E to F

  void fnGoef()
  {
    
       fnGyroTurn(right90);
       delay(TurnDelay);
       analogWrite(S1,FWD);    
    
      do
     {
       iTemp6 = fnIRRead(75);   
     }
     while(iTemp6 > 1500);
     
     analogWrite(S1, STP);
     fnGyroTurn(left90);
     delay(TurnDelay);
     
  }
  
  
// Point F to H
  
  void fnGofh()
  {
    
    analogWrite(S1,FWD);
    
     
     do
     {
       iTemp6 = fnIRRead(75);
     }
     while(iTemp6 > 2000);
     
     analogWrite(S1, STP);
     fnGyroTurn(left90);
     delay(TurnDelay);
     
     analogWrite(S1,FWD);
     
     do
     {
       iTemp5 = fnIRRead(100);
     }
     while(iTemp5 < 3000);
     
     analogWrite(S1, STP);
     
  }
  
  
// Point F to D

  void fnGofd()
  {
    
    fnGyroTurn(left90);
    delay(TurnDelay);
    analogWrite(S1,FWD);
    
        do
     {
       iTemp5 = fnIRRead(100);   //Change to Back sensor
     }
     while(iTemp5 < 3000);
     
     analogWrite(S1, STP);
  
  
  }
  
  
// Point F to E
 void fnGofe()
  {
     fnGyroTurn(center);
     delay(TurnDelay);
    analogWrite(S1,FWD);
    
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 < 1500);
     
     analogWrite(S1, STP);
     
     fnGyroTurn(right90);
     delay(TurnDelay);
  
  }
 //point F to K
  
  void fnGofk()
  {
     analogWrite(S1,FWD);
    
        do
     {
       iTemp5 = fnIRRead(125);   
     }
     while(iTemp5 > 1000);
     
     analogWrite(S1, FWD);
     do
     {
       iTemp5 = fnIRRead(75);
     }while (iTemp5 < 3000);
     analogWrite(S1,STP);
  
  }


  // point G to E
  void fnGoge()
  {
     analogWrite(S1,REV);
    
        do
     {
       iTemp6 = analogRead(bkIRRead);
       delay(2000);   
     }
     while(iTemp6 > 1200);
     
     analogWrite(S1, STP);
     fnGyroTurn(left90);
     delay(TurnDelay);

     do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 1500);

     analogWrite(S1, REV);

     do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 1500);

     analogWrite(S1, STP);
     fnGyroTurn(left90);
     delay(TurnDelay);
     
     
  }
  
  // Point G to D
   void fnGogd()
  {
     fnGyroTurn(right90);
     delay(TurnDelay);
     fnGyroTurn(right90);
     delay(TurnDelay);
    
     analogWrite(S1, FWD);
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 < 3000);
     
     analogWrite(S1, STP);

  
  }
  
  // Point H to F
   void fnGohf()
  {
     
    analogWrite(S1,REV);
    
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 1500);
     
     analogWrite(S1, STP);
     
     fnGyroTurn(right90);
     delay(TurnDelay);
     
     analogWrite(S1,REV);
     
       do
     {
       iTemp5 = fnIRRead(100);
     }
     while (iTemp5 > 3000);
     
     analogWrite(S1,STP);
  
  }
  
//  Point J to K
   void fnGojk()
  {
     analogWrite(S1,REV);
     delay(TurnDelay);
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 1500);
     
     analogWrite(S1, STP);
     
     fnGyroTurn(left90);
     delay(TurnDelay);
  
  }
  
  //Point K to J
  
   void fnGokj()
  {
     fnGyroTurn(left90);
     delay(TurnDelay);
    analogWrite(S1,FWD);
    
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 3000);
     
     analogWrite(S1, STP);
     
  }

  //Point G to F

  void fnGogf()
  {
    analogWrite(S1,REV);
    do
    {
    iTemp5 = fnIRRead(0);
    }
    while ( iTemp5 > 1500);

    analogWrite(S1,REV);
    do
    {
      iTemp5 = fnIRRead(0);
    }
    while( iTemp5 > 1500);
    analogWrite(S1,STP);

    fnGyroTurn(left90);
    delay(TurnDelay);
  }
  //point K to L
  
  void fnGokl()
  {
       fnGyroTurn(left90);
    
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 3000);
     
     analogWrite(S1, STP);
     
  
  }
  
  //point K to F
  
  void fnGokf()
  {
    analogWrite(S1,REV);
      do
    {
      iTemp6 = analogRead(bkIRRead);
      delay(2000);
    }
    while(iTemp6 > 3000);
    
    analogWrite(S1,STP);
  }
  
  //point L to M
  
  void fnGolm()
  {
    fnGyroTurn(left90);
    delay(TurnDelay);
    analogWrite(S1,FWD);
    
    do
    {
      iTemp5 = fnIRRead(100);
    }
    while(iTemp5 > 3000);
    
  }
  
  //point L to K
  
  
  void fnGolk()
  {
    analogWrite(S1,REV);
 
    do
    {
      iTemp6 = analogRead(bkIRRead);
      delay(2000);
    }
    while(iTemp6 > 3000);
    
    analogWrite(S1,STP);
    fnGyroTurn(left90);  
    delay(TurnDelay);
  }
  
  //point M to O
  
   void fnGomo()
  {
    
    analogWrite(S1,FWD);
    
    do
    {
      iTemp5 = fnIRRead(100);
    }
    while(iTemp5 < 3000);
    
    analogWrite(S1,STP);
    
    fnGyroTurn(left90);
    delay(TurnDelay);
    
  }
  
  //point M to L
  
   void fnGoml()
  {
    analogWrite(S1,REV);
     
    do
    {
       iTemp6 = analogRead(bkIRRead);
       delay(2000);
    }
    while(iTemp6 < 3000);

    fnGyroTurn(right90);
    delay(TurnDelay);
    
    analogWrite(S1,STP);
  }
  
  //point N to O
  
   void fnGono()
  {
    fnGyroTurn(right90);
    delay(TurnDelay);
    analogWrite(S1,FWD);
    
    do
    {
     iTemp5 = fnIRRead(100);
    }
    while(iTemp5 < 3000);
    
    analogWrite(S1,STP);
    
    fnGyroTurn(right90);
    delay(TurnDelay);
    
    analogWrite(S1,FWD);
    
    do
    {
      iTemp5 = fnIRRead(100);
    }
    while (iTemp5 < 3000);
    
    analogWrite(S1,STP);
    
    fnGyroTurn(left90);
    delay(TurnDelay);
    fnGyroTurn(left90);
    delay(TurnDelay);
    
  }
 
    // Point D to M
   void fnGodm()
  {
     fnGyroTurn(right90);
     delay(TurnDelay);
     fnGyroTurn(right90);
     delay(TurnDelay);
    
     analogWrite(S1, FWD);
        do
     {
       iTemp5 = fnIRRead(100);   
     }
     while(iTemp5 > 1500);
     
     analogWrite(S1, STP);
     
     fnGyroTurn(left90);
     delay(TurnDelay);
     
    analogWrite(S1, FWD);
      do
     {
       iTemp5 = fnIRRead(100);
     }
     while(iTemp5 > 1500);
     
     analogWrite(S1,STP);
     
     fnGyroTurn(right90);
     delay(TurnDelay);
     
     analogWrite(S1,FWD);
     do
     {
       iTemp5 = fnIRRead(100);
     }
     while (iTemp5 < 3000);
     
     analogWrite(S1,STP);
     
     fnGyroTurn(left90);
     delay(TurnDelay);
     
     analogWrite(S1,FWD);
      do
     {
       iTemp5 = fnIRRead(100);
     }
     while (iTemp5 < 3000);
     
     analogWrite(S1,STP);
     
   
  }
  
 //Point O to M
 void fnGoom()
 {
  analogWrite(S1,REV);

  do
  {
    iTemp6=analogRead(bkIRRead);
    delay(2000);
  }
  while(iTemp6 < 3000);

  analogWrite(S1,STP);

  fnGyroTurn(right90);
  delay(TurnDelay);

  analogWrite(S1,REV);
  do
  {
    iTemp5=fnIRRead(100);
    delay(2000);
  }
  while(iTemp5 < 3000);

  analogWrite(S1,STP);
 }
  //Point O to N

void fnGoon()
  {
   
    analogWrite(S1,FWD);
    
    do
    {
      iTemp5 = fnIRRead(100);
    }
    while(iTemp5 < 3000);
    
    analogWrite(S1,STP);
    
    fnGyroTurn(left90);
    delay(TurnDelay);
    
    analogWrite(S1,FWD);
    
    do
    {
      iTemp6 = analogRead(bkIRRead);
      delay(2000);
    }
    while (iTemp6 < 3000);
    
    analogWrite(S1,STP);
    
    fnGyroTurn(right90);
    delay(TurnDelay);
    
  }



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
  }while(iTemp5 > 1000);
  
  analogWrite(S1, FWD + 50);
  iTemp5 =0;
  do
  {
    iTemp5= fnIRRead(150);
  } while(iTemp5 < 2000);
  analogWrite(S1,STP);
  delay(TurnDelay);
  fnGyroTurn(center);
  delay(TurnDelay);
  //********* END  
  

}

void loop()
{
 
 //Search for victim 1
  
  
  fnGobe();
//  fnGoeg();
  //Send a signal to second Arduino
  digitalWrite(NavtoCon,HIGH);
   //Initial delay for tranfer of command to second arduino
   delay(2000);

   //Do while to wait for second arduino.
    do 
    {
       delay(3000);
    }
     while( digitalRead(ContoNav) == LOW ); 
  
   
   if ( digitalRead(RedVictim)== HIGH)
   {
//     fnGoge();
     fnGoeb();
     fnGobc();
     
     //  Gripper function
     
     fnGocb();
     fnGobe();
     fnGoef();
    }
    else 
     {
      fnGogd();
      
     //Gripper function
     
     fnGodf();
    }
    
    //Search for Victim 2
    
    fnGofh();
    //Initial delay for tranfer of command to second arduino
    delay(2000);
    
  digitalWrite(NavtoCon,HIGH);
   //Do while to wait for second arduino.
    do 
    {
       delay(3000);
    }
     while(digitalRead(ContoNav) == LOW);
    
  
    
     if ( digitalRead(RedVictim) == HIGH )
  
   {
     fnGohf();
     fnGofe();
     fnGoeb();
     fnGobc();
     
      // Gripper function
     
     fnGocb();
     fnGobe();
     fnGoef();
    }
    else 
     {
     fnGohf();
     fnGofd();
      
       //Gripper function
     
     fnGodf();
    }
  
    
    //Search for Victim 3.a
    
     fnGofk();
     fnGokl();
     fnGolm();
   
   digitalWrite(NavtoCon,HIGH);
     //Initial delay for tranfer of command to second arduino
      delay(2000);
     
   //Do while to wait for second arduino.
    do 
    {
       delay(3000);
    }
     while(digitalRead(ContoNav) == LOW);
     
   
   if (digitalRead(RedVictim) == HIGH)
   {
     fnGoml();
     fnGolk();
     fnGokf();
     fnGofe();
     fnGoeb();
     fnGobc();
     
      // Gripper function
     
     fnGocb();
     fnGobe();
     fnGoef();
    }
    else if (digitalRead(YellowVictim) == HIGH)
   {
     fnGoml();
     fnGolk();
     fnGokf();
     fnGofd();
      
      // Gripper function
     
     fnGodf();
    }
    else 
   {
     fnGoml();
     fnGolk();
     fnGokf();
     }
     
    //Search for Victim 3.b
    
     fnGofk();
     fnGokl();
     fnGolm();
     fnGomo();
   
   digitalWrite(NavtoCon,HIGH);
     //Initial delay for tranfer of command to second arduino
      delay(2000);
      
   //Do while to wait for second arduino.
    do 
    {
       delay(3000);
    }
     while(digitalRead(ContoNav) == LOW);
   
   
   if ( digitalRead(RedVictim)==HIGH)
   {
     
     fnGoom();
     fnGolk();
     fnGokf();
     fnGofe();
     fnGoeb();
     fnGobc();
     
      // Gripper function
     
     fnGocb();
     fnGobe();
     fnGoef();
    }
    else if (digitalRead(YellowVictim)== HIGH)
  {
     fnGoom();
     fnGolk();
     fnGokf();
     fnGofd();
      
       //Gripper function
     
     fnGodf();
    }
    else 
 {
     fnGoom();
     fnGolk();
     fnGokf();
     }
  
 
 //Search for Victim 4.a
    
     fnGofk();
     fnGokj();
     digitalWrite(NavtoCon,HIGH);
   //Initial delay for tranfer of command to second arduino
     delay(2000);

   //Do while to wait for second arduino.
    do 
    {
       delay(3000);
    }
     while(digitalRead(ContoNav) == LOW);
     
   
   if ( digitalRead(RedVictim) == HIGH)
   {
     fnGojk();
     fnGokf();
     fnGofe();
//     fnGoeb();
     fnGobc();
     
       //Gripper function
     
     fnGocb();
     fnGobe();
     fnGoef();
    }
    else if (digitalRead(YellowVictim) == HIGH)
   {
     fnGojk();
     fnGokf();
     fnGofd();
      
       //Gripper function
     
     fnGodf();
    }
  else 
    {
     fnGojk();
     fnGokf();
     }
   
   //Search for Victim 4.b
  
    fnGofk();
     fnGokl();
     fnGolm();
     fnGomo();
     fnGoon();
   
   digitalWrite(NavtoCon,HIGH);
   //Initial delay for tranfer of command to second arduino
      delay(2000);

   //Do while to wait for second arduino.
    do 
    {
       delay(3000);
    }
     while(digitalRead(ContoNav) == LOW);
     
   
   
   if ( digitalRead(RedVictim) == HIGH)
   {
     fnGono();
     fnGoom();
     fnGolk();
     fnGokf();
     fnGofe();
//     fnGoeb();
     fnGobc();
     //
      // Gripper function
     
     fnGocb();
     fnGobe();
     fnGoef();
    }
    else if (digitalRead(YellowVictim)== HIGH)
    {
     fnGono();
     fnGoom();
     fnGolk();
     fnGokf();
     fnGofd();
      
     
       //Gripper function
     
     fnGodf();
    }
    else 
  {
    
     fnGono();
     fnGoom();
     fnGolk();
     fnGokf();
     }
  


  

}











