/****************************************************/
/*Authors :  - Abdelmuniem Bahaa Eldien  18011045 ***/
/*           - Ezz Eldine  Elsayed       18011052 ***/ 
/*           - Salma       Alaâ          18010801 ***/
/*           - Osman       Ali           18011049 ***/
/*           - Mohammed    Rashad        18011462 ***/
/*           - Mohamed     Roshdy        18011463 ***/
/*           - Saeed       magdy         18010789 ***/
/*           - Mohammed    Muharram      18011352 ***/
/*                                                ***/
/*Date    : 19 NOV 2021                           ***/
/*Version : V10                                   ***/
/****************************************************/



//Defining sensors' pins
//**********************************************

#define SensorCenterLeft_Pin  A1
#define SensorCenterRight_Pin A2
#define SensorLeft_Pin        A0 
#define SensorRight_Pin       A3

//**********************************************

//array to store the reading of the sensors
long sensors[5];

//Defining motors' pins
//**********************************************

#define RightMotor_Dir1 10
#define RightMotor_Dir2 11
#define LeftMotor_Dir1  12
#define LeftMotor_Dir2  13
#define RightMotor_PWM  5
#define LefttMotor_PWM  6

//**********************************************


//PID calculations variables
//***********************************************************************************
//***********************************************************************************

const int BaseSpeed=150 ;     //the base speed to add and subtract correction from it
int RightSpeed          ;     //Right motor speed corrected
int LeftSpeed           ;     //left motor speed corrected
int Position            ;     //variable contains calculated position of the robot

//PID constants
//**********************
//**********************
float Kp = 5    ;  //***
float Kd = 0.00 ;  //***
float Ki = 0.0  ;  //***
//**********************
//**********************

//error ,correction and setpoint variables 
//**********************

float lasterror      = 0   ;
float error                ;
float errorsum             ;
float correction           ;
int   set_point            ;
long  sensor_average = 0   ;
int   sensor_sum     = 0   ; 
int   EndFlag              ;

//**********************

float SamplingTime = 0 ,CurrentTime = 0 , PreviousTime = 0 ;  //timer variables

//***********************************************************************************
//***********************************************************************************





// void setup for code runs only one time
//************************************************************************
//************************************************************************

void setup()
{

	//setting sensors' input  pins as inputs
	//***************************************
	pinMode(SensorCenterLeft_Pin  ,INPUT);
	pinMode(SensorCenterRight_Pin ,INPUT);
	pinMode(SensorRight_Pin       ,INPUT);
	pinMode(SensorLeft_Pin        ,INPUT);
	//***************************************
	
	//setting motors' pins as outputs
	//***************************************
	pinMode(LeftMotor_Dir1  ,OUTPUT);
	pinMode(LeftMotor_Dir2  ,OUTPUT);
	pinMode(RightMotor_Dir1 ,OUTPUT);
	pinMode(RightMotor_Dir2 ,OUTPUT);
	pinMode(LefttMotor_PWM  ,OUTPUT);
	pinMode(RightMotor_PWM  ,OUTPUT);
	//***************************************
	
	Serial.begin(9600);          //serial begin with baudrate 9600 
  delay       (2000);          //wait for 2 seconds to put the robot at its setpoint

  sensor_average = 0   ;
  sensor_sum     = 0   ; 
	//setting value of the setpoint
	//***************************************
	
    //filling array with sensors' values
	
    sensors[0]     =  analogRead(0);  
    sensors[1]     =  analogRead(1);
    sensors[2]     =  analogRead(2);
    sensors[3]     =  analogRead(3);
	
	
    sensor_average =  sensors[0]*2*1000 + sensors[1]*1*1000 - sensors[2]*1*1000 - sensors[3]*2*1000 ;  // weighted average 
    sensor_sum     =  sensors[0]        + sensors[1]        + sensors[2]        + sensors[3]        ;  // sum of values
    Position       =  int(sensor_average / sensor_sum);                                                // position which will be PID input
    set_point      =  Position;                                                                        // current position will be setpoint  
	  delay                (2000);                                                                        // wait for 2000 milli seconds before start
                                                                        
	//***************************************
}

//************************************************************************
//************************************************************************




// void loop for code runs forever
//************************************************************************
//************************************************************************

void loop()
{

    //check for end point flag 
	
	if(EndFlag)
	{
		
		End              ();      // run end function to stop robot
	
	}
	
	else
	{    
		
		PID_Calculate    ();      // PID variables calculation     
		Set_Motors       ();      // setting motors speed and direction depending on PID calculated values 
	}  
}

//************************************************************************
//************************************************************************





// PID variables calculation 
//************************************************************************
//************************************************************************

void PID_Calculate()
{

	CurrentTime       =  millis();                                                                         //current time set
	SamplingTime      =  (CurrentTime-PreviousTime)*0.001 ;                                                //sampling time as difference between current sample time and previous sample time
	sensor_average    =  0 ;
	sensor_sum        =  0 ;
	
	// filling array by sensors' values 
	
	sensors[0]        =  analogRead(0) ; 
	sensors[1]        =  analogRead(1) ;
	sensors[2]        =  analogRead(2) ;
	sensors[3]        =  analogRead(3) ;
	
	
	sensor_average    =  sensors[0]*2*1000 + sensors[1]*1*1000 - sensors[2]*1*1000 - sensors[3]*2*1000 ;   //weighted average
	sensor_sum        =  sensors[0]        + sensors[1]        + sensors[2]        + sensors[3]        ;   //sum of values
	Position          =  int(sensor_average / sensor_sum);                                                 //position (PID input variable)
	error             =  set_point - Position  ;                                                           //error between set point position and current position
	float delta_error =  error     - lasterror ;                                                           //difference between current and previous error
	errorsum         +=  error ;                                                                           //sum of current and previous error
	lasterror         =  error ;                                                                           //at the end putting current error as the last or previous error for comming loops
	
	//****************************************************************************************************************************************************
	correction        =  int(Kp*error + Ki*errorsum*SamplingTime +Kd*delta_error/SamplingTime);            //PID equation to calculate correction value ** 
	//****************************************************************************************************************************************************
	
	PreviousTime      =  CurrentTime ;                                                                     //at the end putting current time as the last or previous time for comming loops
	
}

//************************************************************************
//************************************************************************
  


  
  
// setting motors speed and direction depending on PID calculated values 
//************************************************************************
//************************************************************************
  void Set_Motors()
{
	RightSpeed = BaseSpeed - correction ;          //correction value subtraction from right motor base speed 
	LeftSpeed  = BaseSpeed + correction ;          //correction value addition to left motor base speed 
	
	//this because correction value depends on weighted average which is negative on the right side and positive on the left side
	
	//restricting speeds of motors between 250 and 0 after correction
	
	constrain(LeftSpeed  ,0 ,250) ;
	constrain(RightSpeed ,0 ,250) ;
	
	
	MotorDrive(RightSpeed,LeftSpeed);             //sending right and left speeds to motor drive function 
}

//************************************************************************
//************************************************************************




// driving motors' direction pins and PWM 
//************************************************************************
//************************************************************************

void MotorDrive(int right, int left)
{

	if(sensor_sum >1500)                         //checking for end point ... this value is measured by putting robot at end point and monitoring sensor sum value
	{
		EndFlag = 1;
	}
	else
	{ 
	
	
	// setting motors speed 
	//********************************************************
		analogWrite(RightMotor_PWM ,right) ;
	  analogWrite(LefttMotor_PWM ,left ) ;
	//********************************************************	
	
	
	// setting motors direction forward 
	//********************************************************
		digitalWrite(LeftMotor_Dir2 ,HIGH ) ;
		digitalWrite(LeftMotor_Dir1 ,LOW  ) ;
		digitalWrite(RightMotor_Dir2,HIGH ) ;
		digitalWrite(RightMotor_Dir1,LOW  ) ;
	//********************************************************

	
	}
}

//************************************************************************
//************************************************************************




//End function which stops robot
//************************************************************************
//************************************************************************

void End ()
{
 analogWrite (RightMotor_PWM ,0   ) ;    // zero PWMs
 analogWrite (LefttMotor_PWM ,0   ) ;    
 digitalWrite(LeftMotor_Dir2 ,LOW ) ;    // no directions
 digitalWrite(LeftMotor_Dir1 ,LOW ) ;
 digitalWrite(RightMotor_Dir2,LOW ) ;
 digitalWrite(RightMotor_Dir1,LOW ) ; 
}

//************************************************************************
//************************************************************************
