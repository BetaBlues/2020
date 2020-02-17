package frc.robot;
import frc.robot.subsystems.Piston;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
//for the USB Camera set up
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;

public class Robot extends TimedRobot {
	
	public Gyro gyro;
	double KpG = 0.03;
	
	DifferentialDrive myRobot;
	Spark leftMotor;
	Spark rightMotor;
	Spark armMotor;
	Spark wheelMotor;
	Piston hook;
	
	// RoboRio mapping
	int leftMotorChannel=1;
	int rightMotorChannel=2;
	int armMotorChannel=3;
	int wheelMotorChannel=0;
	DriverStation ds = DriverStation.getInstance();
	
	// Driver Station / controller mapping
	int joyPort1=0; //driver xbox controller
	int joyPort2=1; //manipulator xbox controller
	
	//Driver Controls
	int lTriggerID = 2;
	int rTriggerID = 3;
	int sandstormStartID = 4;//Y button, not in use
	int slowSpeedButtonID = 3;//X button, not in usw
	int leftStickID = 1; //left and right sticks are joysticks (axis)
	int rightStickID = 5; 
	XboxController driveController;
	XboxController manipController;
	
	//Manipulator Controls
	int hookID = 5; //left trigger
	int armStickID = 1; 
	int redID = 2 ; //B button
	int blueID = 3 ; //X button
	int greenID = 1; //A button
	int yellowID = 4; //Y button
	//verify that stick1 and stick2 correspond to the left and right joysticks on the controller
	//6 is right, 5 is left - bumpers
	
	//global variables
	long lastCompresserUseTime;

	//digital inputs
	final double speedScalingFactor = 0.9;
	double lowSpeed = 0.2;
	double mediumSpeed = 0.3;
	double highSpeed = 0.6;
	
	DigitalInput armSwitch;

	double maxSpeed = 0.5; // max speed for the limelight fucntion
	double minSpeed = 0.3; // min speed for the limelight fuction
	int runTime; //number of cycles function has gone through
	
	boolean sandstormStartState = false; //the state of the button to start sandstorm 

	I2C.Port i2cPort = I2C.Port.kOnboard;
	UsbCamera camera1;
	CameraServer server;
	NetworkTableEntry cameraSelection;
 	private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

	private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

	private final Color fakeRedTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	private final Color fakeYellowTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	private final Color fakeBlueTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private final Color fakeGreenTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
	
	int colorCounter =0 ;
	boolean yellowState = false;
	boolean redState = false;
	boolean greenState = false;
	boolean blueState = false;

	public void robotInit() {
		gyro = new ADXRS450_Gyro(); // Gyro on Analog Channel 1
		
		hook = new Piston (6,7, "hook");

		leftMotor = new Spark(leftMotorChannel);
		rightMotor = new Spark(rightMotorChannel);
		armMotor = new Spark(armMotorChannel);
		leftMotor.setInverted(false);
		rightMotor.setInverted(false);
		armMotor.setInverted(false);
		myRobot = new DifferentialDrive(leftMotor,rightMotor);
		
		driveController  = new XboxController(joyPort1);
		manipController = new XboxController(joyPort2);
		
		armSwitch = new DigitalInput(2); 
		
		m_colorMatcher.addColorMatch(kBlueTarget);
		m_colorMatcher.addColorMatch(kGreenTarget);
		m_colorMatcher.addColorMatch(kRedTarget);
		m_colorMatcher.addColorMatch(kYellowTarget);  

		server = CameraServer.getInstance();
   		server.startAutomaticCapture();
		
	} 

	public void robotPeriodic(){
		Color detectedColor = m_colorSensor.getColor();

		String colorString;
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
			
		if (match.color == kBlueTarget) {
		colorString = "Blue";
		} else if (match.color == kRedTarget) {
		colorString = "Red";
		} else if (match.color == kGreenTarget) {
		colorString = "Green";
		} else if (match.color == kYellowTarget) {
		colorString = "Yellow";
		} else {
		colorString = "Unknown";
		}
		
		SmartDashboard.putNumber("Red", detectedColor.red);
		SmartDashboard.putNumber("Green", detectedColor.green);
		SmartDashboard.putNumber("Blue", detectedColor.blue);
		SmartDashboard.putNumber("Confidence", match.confidence);
		SmartDashboard.putString("Detected Color", colorString);
		System.out.println("camera setting below direct");

		System.out.println(UsbCamera.enumerateUsbCameras());
	}
	
	public void autonomousInit(){
		lastCompresserUseTime = System.currentTimeMillis();
		gyro.reset();
	
		runTime = 90;
		sandstormStartState = false;
	}
	
	public void autonomousPeriodic() {
		autoStart();
		if (sandstormStartState){
			autoTurn(.2);
		}

	}
	
	public void teleopInit() {
		lastCompresserUseTime = System.currentTimeMillis();
		gyro.reset();
		sandstormStartState = false;
	}
	
	public void teleopPeriodic() {

		double rightAxis = -driveController.getRawAxis(rightStickID);
		double leftAxis = -driveController.getRawAxis(leftStickID);
		double armAxis = -manipController.getRawAxis(armStickID);
		// (); //which channels?

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		armAxis = limitAxis(armAxis);

		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		armAxis = limitSpeed(armAxis);

		myRobot.tankDrive(leftAxis, rightAxis);

		if(armSwitch.get()){
			armMotor.set(0);
		}
		else {
			armMotor.set(armAxis);
		}

		togglePiston(hookID, hook);

		findColor(redID, fakeRedTarget);
		findColor(blueID, fakeBlueTarget);
		findColor(greenID, fakeGreenTarget);
		findColor(yellowID, fakeYellowTarget);

		SmartDashboard.putNumber("time since piston last used",
								 (System.currentTimeMillis()-lastCompresserUseTime)/1000);
		if(((System.currentTimeMillis()-lastCompresserUseTime)/1000)<=5){
			SmartDashboard.putString("compressor power", "low");
		}
		else if(((System.currentTimeMillis()-lastCompresserUseTime)/1000)<=8){
			SmartDashboard.putString("compressor power", "medium");
		}
		else if(((System.currentTimeMillis()-lastCompresserUseTime)/1000)>10){
			SmartDashboard.putString("compressor power", "high");
			
		}
		
	}
	
	private double limitAxis (double axis) {//mess with this so it doesn't go so fast
		if (axis > 1.0){  
			axis = 1.0;
		}
		else if (axis < -1.0){
			axis = -1.0;
		}

		return axis;
	}

	//drive
	private double limitSpeed (double axis) {
	
		axis = axis * axis * axis; //The axis (like the y coordinate of the joystick axis) is cubed because the exponentiality of it allows more control at lower speeds  
		axis = axis * speedScalingFactor;
		return axis;
	}


	public void togglePiston(int buttonID, Piston pistonn) {
	
		if (manipController.getRawButtonReleased(buttonID)){ //if the button has been pressed
		
			pistonn.toggle();
			lastCompresserUseTime = System.currentTimeMillis();
		}
	}

	public void autoStart(){
		double angle = gyro.getAngle();
	
		System.out.println("runTime " + runTime);
	
		if (runTime!=0){
			myRobot.arcadeDrive(.6, -angle*KpG); // drive towards heading 0, 0.2 probably slowest possible to run 
			runTime -= 1;
		}
		
	}
	
	public void autoTurn(double speed){
		double angle = gyro.getAngle();
		if ((272 <= angle) || (angle <= 267)){
			myRobot.arcadeDrive(speed,-(angle+90)*0.01); 
		}
		
	}

	public void findColor(int buttonID, Color target) {
		
		m_colorMatcher.addColorMatch(fakeRedTarget);
		m_colorMatcher.addColorMatch(fakeYellowTarget);
		m_colorMatcher.addColorMatch(fakeBlueTarget);
		m_colorMatcher.addColorMatch(fakeGreenTarget); 

		Color detectedColor = m_colorSensor.getColor();
    	ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);	
		if (manipController.getRawButtonReleased(buttonID)){ 
			if (target != match.color) {
				wheelMotor.set(.3);

			}else{
				wheelMotor.set(0.0);
			}

		}
	}

	public void turnWheel(int buttonID, Color target) {
		Color detectedColor = m_colorSensor.getColor();
   		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
		if (colorCounter <32) {
			if (match.color == kBlueTarget && blueState == false){
				colorCounter +=1;
				blueState=true;
				redState=false;
				yellowState=false;
				greenState=false;
			}
			else if (match.color == kRedTarget && redState == false){
				colorCounter +=1;
				redState=true;
				blueState=false;
				yellowState=false;
				greenState=false;
			}
			else if (match.color == kGreenTarget && greenState == false){
				colorCounter +=1;
				greenState=true;
				redState=false;
				yellowState=false;
				blueState=false;
			}
			else if (match.color == kYellowTarget && yellowState == false){
				colorCounter +=1;
				yellowState=true;
				redState=false;
				blueState=false;
				greenState=false;
			}
			wheelMotor.set(.3);
		}else{
			wheelMotor.set(.0);
		}
	}
}

