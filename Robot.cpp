 #include "WPILib.h"
#include "cameraHandler.h"
#define SHOOTER_SPEED 0.70
//#define SHOOTER_MOTOR_DELAY 1.0 //example to mae tuning faster
/**
 * 
 * 
 */
class ShooterEncoderPID : public PIDSource	{
public:
	ShooterEncoderPID(Encoder *shooter_encoder){
		m_shooter_encoder = shooter_encoder;
	}
	double PIDGet(){
		return (m_shooter_encoder->GetRate());
	}
private:
	Encoder *m_shooter_encoder;
};

class ShooterPIDOutput : public PIDOutput {
public:
	ShooterPIDOutput(Jaguar *shooter) {
		m_shooter = shooter;
}
	void PIDWrite(float output) {
		m_shooter->Set(output);
	}
private:
	Jaguar *m_shooter;
};
class BotCatz : public IterativeRobot
{
	// Declare variables
	RobotDrive *m_robotDrive;		// robot will use PWM 1-4 for drive motors
	Joystick *m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick *m_leftStick;			// joystick 2 (tank left stick)
	Joystick *gamepad;      // Logitech Gamepad
	
	//motors
	Jaguar *shooter;        // Jaguar motor for the shooter
	Victor *elevator;     // Motor for the height of shooter.
	
	//Solenoids
	Solenoid *shiftup;      // Solenoid up
	Solenoid *shiftdown;    // Solenoid down
	Solenoid *loaderopen;   // Solenoid to open the loader 
	Solenoid *loaderclose;  // Solenoid to close the loader
	Solenoid *liftup;      // Solenoid to change angle (up)
	Solenoid *liftdown;    // Solenoid to change angle (down)
	Solenoid *dumpup;       //dumper out
	Solenoid *dumpdown;		//dumper in

	//timers
	Timer *autotimer;
	Timer *autotimer2;   
	Timer *autotimer3;
	Timer *encodertimer;
	Timer *timer1;
	
	//encoder
	Encoder *shooterencoder;
	
	//compressor object
	Compressor *compressor;  
	
	//camera
	//AxisCamera *camera;
	
	//PID controller
	PIDOutput *pidShooterOutput;
	PIDSource *pidShooterSource;
	
	//all varibles
	int ShooterReady;		// This varible is used to see if the required motors to shoot are on.
	int step;				// For the Swich that controls the shooter.
	int angle;				// for if the angle of shotter one has changed
	int shift; 				// for if the gears have shifted
	int shiftcounter;		//shift counter
	int shiftreversecounter;//reverse shift counter
	int dump;
	int autonstep;
	int autonstep2;
	int step1;
	int frisbeecount;
	int autonfrisbeecount;
	int counter1;
	float leftvalue;
	float rightvalue;
	float rightslowvalue;
	float leftslowvalue;
	double shooterspeed;
	int shooteruptospeed;
	float distancetocenter;
	float adjustedspeed;
	
	//PID variables
	int prevalue;
	double ShooterRate;
	double avgrate;
	float JoyValue;
	float kp;
	float ki;
	float kd;
	float ShooterSet;
	float pkp;
	float pki;
	float pkd;
	float ShooterError;
	float poffset;
	float ioffset;
	float doffset;
	float Vmax;
	
	//Driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
    DriverStationLCD *m_dsLCD;
    SmartDashboard *dash;
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
	//camera handler init
	Relay *m_Light;
	Relay *m_Light2;
	//cameraHandler *m_camHandle;
	
	
		
public:
	BotCatz(void){	//Constructor for BotCatz class

		// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robotDrive = new RobotDrive(1, 2);

		// Define joysticks 
		m_rightStick = new Joystick(2);
		m_leftStick = new Joystick(1);
		gamepad = new Joystick (3);         // Defines the gamepad
		
		//define motors
		shooter = new Jaguar(3);           // Jaguar motor for the shooter on port 1
		elevator = new Victor(4); 
		
		//define solenoids
		shiftup = new Solenoid (1);         // Defines shiftup (Solenoid) to the port 1
		shiftdown = new Solenoid (2);       // Defines  shiftdown (Solenoid) to the port 2
		loaderopen = new Solenoid (3);      // Defines loaderopen (Solenoid) to the port of 3
		loaderclose = new Solenoid (4);     // Defines loaderclosed (Solenoid) to the port of 4
		liftup = new Solenoid (5);
		liftdown = new Solenoid (6);
		dumpup = new Solenoid (7);
		dumpdown = new Solenoid (8);
		
		//define timers
		autotimer = new Timer();
		autotimer2 = new Timer();           // Defines autotimer 2 as a new timer.
		autotimer3 = new Timer();
		encodertimer = new Timer();
		timer1 = new Timer();
		
		//encoder
		shooterencoder = new Encoder(2,3);
		
		//define compressor
		compressor = new Compressor (1,1);  // Define compressor.
		
		//Camera
		//camera = &AxisCamera::GetInstance();
		
		//PID
		pidShooterOutput = new ShooterPIDOutput(shooter);
		pidShooterSource = new ShooterEncoderPID(shooterencoder);
		
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		
		m_Light = new Relay(2,Relay::kForwardOnly);
		m_Light2 = new Relay(3);
		//dash = SmartDashboard::GetInstance();
		//m_camHandle = new cameraHandler(camera, m_dsLCD, dash, m_Light2);
		m_dsLCD = DriverStationLCD::GetInstance();
		
		//SmartDashboard.GetNumber(shooterencoder);
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		//compressor
		compressor->Start();
		m_robotDrive->SetSafetyEnabled(false);
		
		//PID
		shooterencoder->Start();
		
		Vmax = 3;
		kp = 1.0;
		ki = 0.0;
		kd = 0.0;
		
		poffset = 0.001;
		ioffset = 0.0001;
		doffset = 0.001;
		
		ShooterSet = 17000;
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		compressor->Start();

	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		autotimer->Start();
		autotimer3->Start();
		timer1->Start();
		autotimer->Reset();
		autotimer3->Reset();
		timer1->Reset();
		frisbeecount = 0;
		autonfrisbeecount = 0;
		counter1 = 0;
		autonstep = 0;
		autonstep2 = 0;
		step1 = 0;
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		ShooterReady = 0;                            
		step = 0;
		angle = 0;
		shift = 0;
		shiftcounter = 0;
		shiftreversecounter = 0;
		dump = 0;
		shooteruptospeed = 1;
		autotimer2->Start();
		adjustedspeed = SHOOTER_SPEED;
		
		//PID
		shooterencoder->Start();
		encodertimer->Start();	
		
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		}

	void AutonomousPeriodic(void) {
		// increment the number of autonomous periodic loops completed
		m_autoPeriodicLoops++;
		//AutonShootAndDrive();
		//AutonQuickShoot();
		AutonQuickShootAndDrive();
	}

	
	void TeleopPeriodic(void) {
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		m_dsPacketsReceivedInCurrentSecond++;// increment DS packets received

		
		//Teleoperated code goes here
		/*if(gamepad->GetRawButton(6) == 1){
			CameraController();
			m_Light->Set(Relay::kOn);
		}
		else{*/
		DriveRobot();
		//}
		
		Shift();
		ShooterController();
		//ShooterPIDControl();
		LiftController();
		
			
	}
	
	
	void DriveRobot(void){
		//Drives the robot in tank dive with two joysticks
		leftvalue = m_leftStick->GetY();                          // Get the Y value for the left joystick
		rightvalue = m_rightStick->GetY();                         // Get the Y value for the right joystick
		if (m_rightStick->GetRawButton(4) == 1){
			leftslowvalue = leftvalue*0.5;
			rightslowvalue = rightvalue*0.5;
			m_robotDrive->TankDrive(-leftslowvalue,-rightslowvalue);   // Drive with tank style (using both sticks)
			m_Light->Set(Relay::kOn);
		}
		else{
			m_robotDrive->TankDrive(-leftvalue,-rightvalue);   // Drive with tank style (using both sticks)
			m_Light->Set(Relay::kOff);
		}
	}
	
	void Shift(void){
		if (m_leftStick->GetRawButton(1) == 1  && m_rightStick->GetRawButton(1) == 1 && shift == 0)                // Detects if the trigger of the left joystick is pressed 
		{
			shiftcounter++;
			shiftreversecounter = 0;
			if(shiftcounter > 2){
				shiftup->Set(true);
				shiftdown->Set(false);
				shift = 1;
			}
		}
		else if (m_leftStick->GetRawButton(3) == 1  && m_rightStick->GetRawButton(3) == 1 && shift == 1)         // Detects if the trigger of the right joystick 
		{
	       shiftreversecounter++;
	       shiftcounter = 0;
	       if(shiftreversecounter > 2){
	    	   shiftdown->Set(true);                  
	    	   shiftup->Set(false); 
	    	   shift = 0;
	       }
		}
	}
	

	
	void ShooterController(void){
		//The following has to do with the shooter
		//printf("Shooter Encoder >> %f \n",shooterencoder->GetRate());
		
		if(gamepad->GetRawButton(1) == 1){
			adjustedspeed = 0.72;
		}
		else if(gamepad->GetRawButton(2) == 1){
			adjustedspeed = SHOOTER_SPEED;
		}
	
		if (gamepad->GetRawButton(7))           // If the button 5 is pressed:
		{
		  shooter->Set(adjustedspeed);              // Set Jaguar motor to LUDICROUS-SPEED (Full Speed)!
		  ShooterReady = 1;                
		}
	  
	    else if (gamepad->GetRawButton(5))      // If the button 7 is pressed:
	    {
		  shooter->Set(0.0);                // Turn motor off.
		  ShooterReady = 0;
	    }
		
		
	  switch (step){//uses autotimer2 and step
		  case 0:
			  //If the right bottom trigger (8) of the Logitech gamepad and the shooter is ready (as defined above) then:
			  if (gamepad->GetRawButton(8) == 1 && ShooterReady == 1 && shooteruptospeed == 1)   
			  {
				  loaderopen->Set(true);                 
				  loaderclose->Set(false); 
				  autotimer2->Reset();          //Reset AutoTimer2  
				  shooteruptospeed = 0;
				  step = 1;                     // Go to step one
				  
			  }
			  break;
		  case 1:
			  if (autotimer2->Get() >= 0.20) //consider using constant to tune value if needed
			  {
				  loaderclose->Set(true);
				  loaderopen->Set(false);
				  shooter->Set(1.0); //consider using a constant
				  autotimer2->Reset(); // Reset the timer
				  step = 2;            // Got to step 2
			  }
			  break;
		  case 2:
			  if (autotimer2->Get() >= 0.30)
			  {
				  shooter->Set(adjustedspeed);
				  autotimer2->Reset();
				  step = 3;
			  }
			  break;
		  case 3:
			  if (autotimer2->Get() >= 0.28){
				  shooteruptospeed = 1;
				  step = 0;
			  }
			  break;
	  }
	}
	
	void LiftController(void){
		  if (gamepad->GetRawButton(3) == 1)
		  {
			  liftup->Set(true);
			  liftdown->Set(false);
		  }
		  if (gamepad->GetRawButton(4) == 1)
		  {
			  liftup->Set(false);
			  liftdown->Set(true);
		  }
	}
	/*
	void CameraController(void){
		m_dsLCD->Clear();
		distancetocenter = m_camHandle->getCenter()*1000;
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"distance to center %f \n",distancetocenter);
		printf("Cen X Norm: %f \n",distancetocenter);
		m_dsLCD->UpdateLCD();
		
		if(distancetocenter > -90.0){
			m_robotDrive->Drive(0.16,1.0);
		}
		else if(distancetocenter < -170.0){
			m_robotDrive->Drive(0.16,-1.0); 
		}
		else{
			m_robotDrive->Drive(0.0,0.0);
		}
		
	}
	*/
	void AutonShootAndDrive(void){
		switch(autonstep){//uses autotimer and autonstep
				case 0:
					shooter->Set(SHOOTER_SPEED);
					autotimer->Reset();
					autonstep = 1;
					break;
				case 1:
					if(autotimer->Get()>= 3.5){
						loaderopen->Set(true);
						loaderclose->Set(false);
						frisbeecount =(frisbeecount+1);
						autotimer->Reset();
						autonstep = 2;
					}
					break;
				case 2:
					if(autotimer->Get()>= .2 && frisbeecount < 3){
						loaderopen->Set(false);
						loaderclose->Set(true);
						autotimer->Reset();
						autonstep = 1;
					}
					else if(frisbeecount >= 3 && autotimer->Get() >= 0.5){
						autonstep = 3;
					}
					break;
				case 3:
						shooter->Set(0.0);
						loaderopen->Set(false);
						loaderclose->Set(true);
						m_robotDrive->Drive(-0.8,0.0);
						autotimer->Reset();
						autonstep = 4;
						break;
				case 4:
					if(autotimer->Get() >= 1.5){
						m_robotDrive->Drive(0.0,0.0);
					}
					break;
				}
	}
	
	void AutonQuickShoot(void){//uses autotimer3 and autonstep2
		switch(autonstep2){
			case 0:
				shooter->Set(SHOOTER_SPEED);
				if(autotimer3->Get() >= 5.0){
					autonstep2 = 1;
				}
				break;
			case 1:
				if(autonfrisbeecount <= 2){
					loaderopen->Set(true);                 
					loaderclose->Set(false); 
					autotimer3->Reset();          //Reset AutoTimer2 
					autonfrisbeecount ++;
					autonstep2 = 2;                     // Go to step one
				}
				else{
					autonstep2 = 5;
				}
				break;
			case 2:
				if (autotimer3->Get() >= 0.2) 
					{
					  loaderclose->Set(true);
					  loaderopen->Set(false);
					  shooter->Set(1.0);
					  autotimer3->Reset(); // Reset the timer
					  autonstep2 = 3;            // Got to step 2
					 }
				 break;
			case 3:
				if (autotimer3->Get() >= 0.28)
				 {
					shooter->Set(SHOOTER_SPEED);
					autotimer3->Reset();
					autonstep2 = 4;
				}
				break;
			case 4:
				 if (autotimer3->Get() >= 0.28){
					autonstep2 = 1;
				}
				 break;
			case 5:
				shooter->Set(0.0);
				break;
		}
	}
	
	void AutonQuickShootAndDrive(void){//uses timer1 and step1 and counter1
		switch(step1){
			case 0:
				shooter->Set(1.0);
				if(timer1->Get() >= 0.75){
					timer1->Reset();
					step1 = 1;
				}
				break;
			case 1:
				shooter->Set(SHOOTER_SPEED);
				if(timer1->Get() >= 1.0){
					timer1->Reset();
					step1 = 2;
				}
				break;
			case 2:
				if(counter1 <= 2){
					loaderopen->Set(true);                 
					loaderclose->Set(false); 
					timer1->Reset();          //Reset timer1 
					counter1 ++;
					step1 = 3;                     // Go to step one
				}
				else{
					step1 = 6;
				}
				break;
			case 3:
				if (timer1->Get() >= 0.2) 
					{
					  loaderclose->Set(true);
					  loaderopen->Set(false);
					  shooter->Set(1.0);
					  timer1->Reset(); // Reset the timer
					  step1 = 4;            // Got to step 2
					 }
				 break;
			case 4:
				if (timer1->Get() >= 0.28)
				 {
					shooter->Set(SHOOTER_SPEED);
					timer1->Reset();
					step1 = 5;
				}
				break;
			case 5:
				 if (timer1->Get() >= 0.28){
					step1 = 2;
				}
				 break;
			case 6:
				shooter->Set(0.0);
				m_robotDrive->Drive(-0.8,0.0);
				timer1->Reset();
				step1 = 7;
				break;
			case 7:
				if(timer1->Get() >= 1.5){
					m_robotDrive->Drive(0.0,0.0);
				}
				break;
		}
	}


	void ShooterPIDControl(void){
		
		//PID controller
		PIDController ShooterController(kp,
				ki,
				kd,
				pidShooterSource,
				pidShooterOutput,
				0.005);
		ShooterController.SetInputRange(0,20000);
		ShooterController.SetOutputRange(-1, 1);
	
		printf("Shooter Error>> %f\n",ShooterController.GetSetpoint());
		if (gamepad->GetRawButton(7))           // If the button 5 is pressed:
		{
			ShooterController.Enable();
			ShooterController.SetSetpoint(17000);
			ShooterReady = 1;                
		}
	  
	    else if (gamepad->GetRawButton(5))      // If the button 7 is pressed:
	    {
	    	ShooterController.Disable();
	    	shooter->Set(0.0);                // Turn motor off.
	    	ShooterReady = 0;
	    }
		  switch (step)
		  {
			  case 0:
				  //If the right bottom trigger (8) of the Logitech gamepad and the the shooter is ready (as defined above) then:
				  if (gamepad->GetRawButton(8) == 1 && ShooterReady == 1)   
				  {
					  loaderopen->Set(true);                 
					  loaderclose->Set(false);      
					  step = 1;                     // Go to step one
					  autotimer2->Reset();          //Reset AutoTimer2
				  }
				  break;
			  case 1:
				  if (autotimer2->Get() >= 0.25) 
				  {
					  loaderclose->Set(true);
					  loaderopen->Set(false);
					  autotimer2->Reset(); // Reset the timer
					  step = 2;            // Got to step 2
				  }
				  break;
			  case 2:
				  if (autotimer2->Get() >= 0.25)
				  {
					  step = 0;
				  }
				  break;
		  }
	}
			
};

START_ROBOT_CLASS(BotCatz);
