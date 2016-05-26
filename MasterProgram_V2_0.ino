//Robot Drive Program
//Brandon Malone
// 4/12/2016
// Adaptation of V1.8 based off printed corrections, coppies in from RobotSetup, 
// fnGyroTurn from RobotSetup programed 4/11
// fnIRRead from Diagnostics1.9 programmed 4/11
// fnBackIR created 4/12



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

// COPY AND PASTE THESE VARIABLES FROM SETUP PROGRAM RUN
int right90 = 1050;     
int left90 = 3000;
int center = 2048;
int FWD = 2025;
int sFWD = 2050;
int REV= 2250;
int sREV = 2200;
int STP = 2100;

int centeroffset = center - 300;
int centerleft = center + 100;
int ShortDelay = 1000;
int TurnDelay = 1500;


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
//prototypes
int fnIRRead(int);
void fnGyroTurn(int);
int fnBackIR();
void fnGoeb();
void fnGobe();
void fnGobc();
void fnGocb();
void fnGoeg();
void fnGoed();
void fnGode();
void fnGoef();
void fnGofh();
void fnGofd();
void fnGofe();
void fnGofk();
void fnGoge();
void fnGohf();
void fnGojk();
void fnGokj();
void fnGokl();
void fnGokf();
void fnGolm();
void fnGolk();
void fnGomo();
void fnGoml();
void fnGono();
void fnGoom();
void fnGoon();





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
  digitalWrite(RLYMotor, HIGH); // RESET INTEGRATOR WITH MOTOR DISABLED
  digitalWrite(RLYPostAmp, HIGH); //POST AMP RELAY SET TO LOW SENSITIVITY MODE
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
  } while (digitalRead(Switch) == HIGH);

  digitalWrite(stpdir, HIGH);

  for (int i = iSMCorrect; i > 0; i--) //drive left until you reach the correction factor
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

  analogWrite(S1, sFWD);
  do
  {
    iTemp1 = fnBackIR();
  } while (iTemp1 > 975); //VARIABLE
  analogWrite(S1, STP);
  delay(1000);

  fnGyroTurn(left90);
  delay(2000);

  analogWrite(S1, FWD);
  do
  {
    iTemp1 = fnBackIR();
    Serial.println(iTemp1);
  } while (iTemp1 > 975); //VARIABLE
  analogWrite(S1, STP);

  analogWrite(S1, REV);
  do
  {
    iTemp1 = fnBackIR();
  } while (iTemp1 < 3800); //VARIABLE
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


  //********* END


}

void loop()
{
  //Assuming every victim is present, and every color is yellow
  //Search for victim 1
  
  //V1
  
  fnGobe();
  fnGoeg();
  fnGoge();
  fnGoed();
  fnGode();
  fnGoef();
  //V2
  fnGofh();
  fnGohf();
  fnGohf();
  fnGofd();
  fnGode();
  fnGoef();
  fnGofk();
  fnGokj();
  fnGojk();
  fnGokf();
  fnGofd();
  fnGodf();
  //V3A
  fnGofk();
  fnGokl();
  fnGolk();
  fnGokf();
  fnGofd();
  fnGodf();
  //V4A
  fnGofk();
  fnGokl();
  fnGolm();
  fnGoml();
  fnGolk();
  fnGokf();
  fnGofd();
  fnGodf();
  //V4B
  fnGofk();
  fnGokl();
  fnGolm();
  fnGomo();
  fnGoom();
  fnGoml();
  fnGolk();
  fnGokf();
  fnGofd();
  fnGodf();
  //V3A
  fnGofk();
  fnGokl();
  fnGolm();
  fnGomo();
  fnGoon();
  fnGono();
  fnGoom();
  fnGoml();
  fnGolk();
  fnGokf();
  fnGofd();
}



















//***************************************************************************************
//                 IR (top sensor) READING FUNCTION. PASS IN THE BEARING
//                WITH 0 BEING RIGHT, 100 BEING CENTER, AND 200 BEING LEFT
//                  RETURNS A DISTANCE MEASUREMENT (HIGH VALUE = CLOSER)
//                  PULLED FROM PROGRAM Nstepper1_3.ino 3/8/2016
//                                                  -Brandon
//***************************************************************************************
int fnIRRead(int iSMCMD) //FROM diagnostics1.9 4/11/2016
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
  
  
  delay(15);  //delay added for accuracy (1.3)

  iTemp4 = analogRead(A2);
  delay(15);
  iTemp5 = analogRead(A2);

  int iDiff4 = iTemp4 - iTemp5;
  int iDiff5 = iTemp5 - iTemp4;
  int iAVG = iTemp4 + iTemp5;
  iAVG = iAVG / 2;

  if (iDiff4 > 200)
  {
    return iTemp5;
  }
  else if(iDiff5 > 200)
  {
  
  return iTemp4;
  }
  else
  {
    return iAVG;
  }
  
}
//***************************************************************************************
//                                   GYRO TURN CODE
//***************************************************************************************

void fnGyroTurn(int iVAL)   //Smoothed out anti-jerkage from RobotSetup 4/11/2016
{
  int i;
  bool bOverTurn = false;
  int iVALOverTurn;
  if(iS2Current > 2100)
  {
    bOverTurn = true;
    iVALOverTurn = iVAL - 500;
  }
 /* ANTI JERKAGE CODE!!!! */   
  if(iVAL <= iS2Current)                      //if turning right
  {
    if(bOverTurn == false)
    {
      do
      {
        i = iS2Current - iVAL;
        if(i > 50)                             //if diff between current S2 and commanded > 100
        {
          iS2Current = iS2Current - 50;        //incrimentally update S2 
          analogWrite(S2, iS2Current);
          Serial.print("Writing to S2: ");
          Serial.println(iS2Current);
          delay(25);                           //delay long enough for robot to move
          Serial.println("25ms wait");
        }
      }while(i > 50);                          //while that difference is greater that 100
    }
    else
    {
      do
      {
        i = iS2Current - iVALOverTurn;
        if(i > 50)                             //if diff between current S2 and commanded > 100
        {
          iS2Current = iS2Current - 50;        //incrimentally update S2 
          analogWrite(S2, iS2Current);
          Serial.print("Writing to S2: ");
          Serial.println(iS2Current);
          delay(25);                           //delay long enough for robot to move
          Serial.println("25ms wait");
        }
      }while(i > 50);                          //while that difference is greater that 100
    }
  }
  
  
  else                                        //if robot turning left same thing
  {
    do
    {
      i = iVAL - iS2Current;
      if(i > 50)
      {
        iS2Current = iS2Current + 50;
        analogWrite(S2, iS2Current);
        Serial.print("Writing to S2: ");
        Serial.println(iS2Current);
        delay(25);
        Serial.println("25ms wait");
      }
    }while(i > 50);
  }
  
  iS2Current = iVAL;                          //final update
  analogWrite(S2, iVAL);                      //writing remainder
  Serial.print("Writing to S2: ");
  Serial.println(iVAL);

}
//***************************************************************************************
//                                 BACK IR CODE
//***************************************************************************************
int fnBackIR() //CREATED IN MASTER V2.0
{
  iTemp5 =0;
  iTemp6 =0;
  delay(15);
  iTemp5 = analogRead(bkIRRead);
  delay(15);
  iTemp6 = analogRead(bkIRRead);
  int i5high = iTemp5 - iTemp6;
  int i6high = iTemp6 - iTemp5;
  int i56AVG = iTemp5 + iTemp6;
  i56AVG = i56AVG/2;
  if (i5high > 200)
  {
    return iTemp6;
  }
  else if(i6high > 200)
  {
  
  return iTemp5;
  }
  else
  {
    return i56AVG;
  }
}

  
//***************************************************************************************
//                                 ROUTES
//***************************************************************************************  
  
  
  
  
// Point e to b
void fnGoeb()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 < 3800);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


// Point b to e
void fnGobe()    //V2.0 fixed
{

  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }while (iTemp5 < 3330);
  analogWrite(S1, STP);
  delay(ShortDelay);

}


// Point b to c
void fnGobc() //VERIFIED    //V2.0 fixed
{
  fnGyroTurn(right90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }while (iTemp5 < 1800);
  analogWrite(S1, STP);
  delay(ShortDelay);

}


// Point c to b
void fnGocb()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 < 975);
  analogWrite(S1, STP);
  fnGyroTurn(center);
  delay(TurnDelay);
  delay(ShrotDelay);

}

// REMOVED IN MASTER 2.0
//Point D to F

//void fnGodf()
//{
//  analogWrite(S1, REV);
//  do
//  {
//    iTemp5 = fnIRRead(200);
//  }
//  while (iTemp5 > 15000);
//
//  analogWrite(S1, STP);
//
//  //   fnGyroTurn(left90);
//  delay(TurnDelay);
//}

//Point E to G
void fnGoeg() //VERIFIED    //V2.0 fixed
{
  fnGyroTurn(right90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  iTemp5 = 0;
  do
  {
    iTemp5 = fnIRRead(100);
  } while (iTemp5 < 1100);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


//Point E to D
void fnGoed()  //verified   //V2.0 fixed
{
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  delay(1000);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


//Point D to E
void fnGode() //verified   //V2.0 fixed
{
  analogWrite(S1, REV);
  iTemp6 = 0;
  do
  {
    iTemp6 = fnIRRead(100);
  } while (iTemp6 > 1000);
  analogWrite(S1, STP);
  fnGyroTurn(center);
  delay(TurnDelay);
  delay(ShortDelay);
}


// Point E to F
void fnGoef()    //V2.0 fixed
{

  fnGyroTurn(right90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp6 = fnIRRead(200);
  }while (iTemp6 > 1200);
  analogWrite(S1, STP);
  delay(ShortDelay);
  analogWrite(S1, sREV);
  delay(750);
  analogWrite(S1, STP);
  fnGyroTurn(center);
  delay(TurnDelay);
  delay(ShortDelay);
}


// Point F to H
void fnGofh()   //V2.0 fixed
{
  analogWrite(S1, FWD);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 > 1500);
  analogWrite(S1, sFWD);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 > 850);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }
  while (iTemp5 < 1100);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


// Point F to D
void fnGofd()   //V2.0 fixed
{
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100); 
  }while (iTemp5 < 1800);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


// Point F to E
void fnGofe()   //V2.0 fixed
{
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }while (iTemp5 < 1000);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(right90);
  delay(TurnDelay);
  delay(ShortDelay);
}


//point F to K
void fnGofk()   //V2.0 fixed  NEEDS VERIFIED
{
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnBackIR();
  }while (iTemp5 > 1100);
  
  analogWrite(S1, sFWD);
  
  do
  {
    iTemp5 = fnIRRead(200);
  }while (iTemp5 < 1000);
  delay(1000);                 //KEY VARIABLE TIME
  analogWrite(S1, STP);
  delay(ShortDelay);
}


// point G to E
void fnGoge()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 < 975);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(center);
  delay(TurnDelay);
  delay(ShortDelay);
}

//// Point G to D eliminated v2.0
//void fnGogd()
//{
//  analogWrite(S1, REV);
//  iTemp5 = 0;
//  do
//  {
//    iTemp5 = fnIRRead(0);
//    
//  }while(iTemp5 > 2000);
//  analogWrite(S1, STP);
//  fnGyroTurn(center);
//  delay(1000);
//  fnGyroTurn(left90);
//  analogWrite(S1, FWD);
//  do
//  {
//    iTemp5 = fnIRRead(0);
//  }while(iTemp5<3500);
//  analogWrite(S1, STP);
//  delay(TurnDelay);
//    
//  
//
//}

// Point H to F
void fnGohf()   //V2.0 fixed NEEDS VERIFIED
{
  analogWrite(S1, REV);
  do
  {
    iTemp5 = fnIRRead(190);
  }while (iTemp5 > 1300);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(center);
  delay(TurnDelay);
  analogWrite(S1, REV);
  do
  {
    iTemp5 = fnBackIR();
  }while (iTemp5 < 3800);
  analogWrite(S1, STP);
  delay(ShortDelay);
}

//  Point J to K
void fnGojk()   //V2.0 fixed
{
  fnGyroTurn(center);
  delay(TurnDelay);
  analogWrite(S1, REV);
  do
  {
    iTemp5 = fnBackIR();
  }while(iTemp5 < 3600);
  analogWrite(S1, STP);
  delay(ShortDelay);
  
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnIRRead(200);
  }while (iTemp6 > 1000);
  analogWrite(S1, sREV);
  delay(500);                  //KEY VARIABLE
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(center);
  delay(TurnDelay);
  delay(ShortDelay);
}


//Point K to J
void fnGokj()   //V2.0 fixed
{
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }
  while (iTemp5 < 1100);
  analogWrite(S1, STP);
  delay(ShortDelay);
}

//Point G to F

//void fnGogf()    //V2.0 eliminated
//{
//  analogWrite(S1, REV);
//  do
//  {
//    iTemp5 = fnIRRead(0);
//  }
//  while ( iTemp5 > 1500);
//
//  analogWrite(S1, REV);
//  do
//  {
//    iTemp5 = fnIRRead(0);
//  }
//  while ( iTemp5 > 1500);
//  analogWrite(S1, STP);
//
//  fnGyroTurn(left90);
//  delay(TurnDelay);
//}


//point K to L
void fnGokl()   //V2.0 fixed
{
  fnGyroTurn(right90);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }while (iTemp5 < 3330);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


//point K to F
void fnGokf()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 < 3800);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


//point L to M
void fnGolm()   //V2.0 fixed
{
  fnGyroTurn(center);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }
  while (iTemp5 < 1000); // KEY VARIABLE
  delay(ShortDelay);
  
}


//point L to K
void fnGolk()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnIRRead(100);
  }
  while (iTemp6 > 1000);
  iTemp6 = fnIRRead(0);
  delay(1000);
  do
  {
    iTemp5 = fnIRRead(200);
  }while(iTemp5 > 450);
  analogWrite(S1, sREV);
  delay(500);                     //KEY VARIABLE
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(center);
  delay(TurnDelay);
  delay(ShortDelay);
}


//point M to O
void fnGomo()   //V2.0 fixed
{
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }while (iTemp5 < 3330);
  analogWrite(S1, STP);
  fnGyroTurn(left90);
  delay(TurnDelay);
  delay(ShortDelay);

}

//point M to L

void fnGoml()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 < 3800);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(right90);
  delay(TurnDelay);
  delay(ShortDelay);
}

//point N to O

void fnGono()   //V2.0 fixed
{
  fnGyroTurn(center);
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }while(iTemp5 < 3330);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(left90);
  delay(TurnDelay);
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while(iTemp6 < 2500); //arbitrary
  analogWrite(S1, STP);
  delay(ShortDelay);
}

//// Point D to M
//void fnGodm()  //absolutely no reason to have this
//{
//  fnGyroTurn(right90);
//  delay(TurnDelay);
//  fnGyroTurn(right90);
//  delay(TurnDelay);
//
//  analogWrite(S1, FWD);
//  do
//  {
//    iTemp5 = fnIRRead(100);
//  }
//  while (iTemp5 > 1500);
//
//  analogWrite(S1, STP);
//
//  fnGyroTurn(left90);
//  delay(TurnDelay);
//
//  analogWrite(S1, FWD);
//  do
//  {
//    iTemp5 = fnIRRead(100);
//  }
//  while (iTemp5 > 1500);
//
//  analogWrite(S1, STP);
//
//  fnGyroTurn(right90);
//  delay(TurnDelay);
//
//  analogWrite(S1, FWD);
//  do
//  {
//    iTemp5 = fnIRRead(100);
//  }
//  while (iTemp5 < 3000);
//
//  analogWrite(S1, STP);
//
//  fnGyroTurn(left90);
//  delay(TurnDelay);
//
//  analogWrite(S1, FWD);
//  do
//  {
//    iTemp5 = fnIRRead(100);
//  }
//  while (iTemp5 < 3000);
//
//  analogWrite(S1, STP);
//
//
//}


//Point O to M
void fnGoom()   //V2.0 fixed
{
  analogWrite(S1, REV);
  do
  {
    iTemp6 = fnBackIR();
  }while (iTemp6 < 3800);
  analogWrite(S1, STP);
  delay(ShortDelay);
  
  fnGyroTurn(center);
  delay(TurnDelay);
  analogWrite(S1, REV);
  do
  {
    iTemp5 = fnIRRead(100);
  }while (iTemp5 > 2000);
  analogWrite(S1, STP);
  delay(ShortDelay);
}


//Point O to N
void fnGoon()   //V2.0 fixed
{
  analogWrite(S1, FWD);
  do
  {
    iTemp5 = fnIRRead(100);
  }
  while (iTemp5 < 1000);
  analogWrite(S1, STP);
  delay(ShortDelay);
  
  fnGyroTurn(3500);              //SPECIAL
  delay(TurnDelay);
  analogWrite(S1, FWD);
  do
  {
    iTemp6 = fnIRRead(100);
  }
  while (iTemp6 < 1800);
  analogWrite(S1, STP);
  delay(ShortDelay);
  fnGyroTurn(left90);
  delay(TurnDelay);
  delay(ShortDelay);
}










