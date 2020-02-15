package frc.robot;
import frc.robot.subsystems.Piston;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.interfaces.Gyro;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.cameraserver.CameraServer;



//import org.usfirst.frc.team5975.robot.subsystems.Pixy2;
//import org.usfirst.frc.team5975.robot.subsystems.Pixy2.LinkType;
//import org.usfirst.frc.team5975.robot.subsystems.links.Link;
//import org.usfirst.frc.team5975.robot.subsystems.links.SPILink;
//import edu.wpi.first.wpilibj.CameraServer;

import java.lang.Math;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
//import org.usfirst.frc.team5975.robot.subsystems.links.UARTLink;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;


// Limelight Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//encoder imports
//import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.DigitalInput;

//import org.usfirst.frc.team5975.robot.subsystems.Camera;

import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer; //seems like it can't exist with import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.cscore.VideoSource;

public class Robot extends TimedRobot {
	
	//public Gyro gyro;
	double KpG = 0.03;
	boolean x = true;
	
	DifferentialDrive myRobot;
	Spark leftMotor;
	Spark rightMotor;
	Spark hatchMotor;
	Spark armMotor;
	Spark wheelMotor;
	Piston frontLegs;
	Piston hatchPiston;
	Piston backLegs;
	Piston hook;
	//boolean pistonForward;
	
	// RoboRio mapping
	int leftMotorChannel=1;
	int rightMotorChannel=2;
	int armMotorChannel=3;
	int wheelMotorChannel=0;
	//int hatchMotorChannel=0;
	DriverStation ds = DriverStation.getInstance();
	
	// Driver Station / controller mapping
	int joyPort1=0; //driver xbox controller
	int joyPort2=1; //manipulator xbox controller
	
	//Driver Controls
	int lTriggerID = 2;
	int rTriggerID = 3;
	int sandstormStartID = 4;//Y button
	int slowSpeedButtonID = 3;//X button
	int leftStickID = 1; //left and right sticks are joysticks (axis)
	int rightStickID = 5;
	int armStickID = 1;
	XboxController driveController;
	XboxController manipController;
	
	//Manipulator Controls
	int limelightID= 2; //B button
	int hatchID = 3; //X button
	int linearMotionID = 4; //Y button
	int frontLegsID = 5; //left Bumper
	int backLegsID = 6; //right Bumper
	int hookID = 1; //a button (probably)
	//verify that stick1 and stick2 correspond to the left and right joysticks on the controller
	//6 is right, 5 is left
	
	//global variables
	long lastCompresserUseTime;
	int turnLimit;
	//for field pieces, right is true, left is false
	
	//digital inputs
	final double speedScalingFactor = 0.9;
	double lowSpeed = 0.2;
	double mediumSpeed = 0.3;
	double highSpeed = 0.6;
	//Encoder Variables
	Encoder hatchMotorEncoder;
	int hatchTurnLimit = 200;
	int i = 0;
	boolean linearMotionState;
	
	DigitalInput m_aSource;
	DigitalInput m_bSource;
	DigitalInput armSwitch;

	
	boolean limelightButtonState = false;

	double maxSpeed = 0.5; // max speed for the limelight fucntion
	double minSpeed = 0.3; // min speed for the limelight fuction
	int runTime; //number of cycles function has gone through
	

	//Link link = new UARTLink();
	boolean sandstormStartState = false; //the state of the button to start sandstorm 

	// Pixy2 pixy;

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
	
	//public boolean setVideoMode(camera1, PixelFormat pixelformat, int width, int height, int fps);
	int colorCounter =0 ;
	boolean yellowState = false;
	boolean redState = false;
	boolean greenState = false;
	boolean blueState = false;


	public void robotInit() {
		//gyro = new ADXRS450_Gyro(); // Gyro on Analog Channel 1
		
		frontLegs = new Piston (0,1, "front legs");
		backLegs = new Piston (5,4, "back legs");
		hatchPiston = new Piston (2,3, "hatch");
		hook = new Piston (6,7, "hook");

		leftMotor = new Spark(leftMotorChannel);
		rightMotor = new Spark(rightMotorChannel);
		armMotor = new Spark(armMotorChannel);
		//hatchMotor = new Spark(hatchMotorChannel);
		leftMotor.setInverted(false);
		rightMotor.setInverted(false);
		armMotor.setInverted(false);
		myRobot = new DifferentialDrive(leftMotor,rightMotor);
		
		driveController  = new XboxController(joyPort1);
		manipController = new XboxController(joyPort2);
		
		hatchMotorEncoder = new Encoder(3,4, false, Encoder.EncodingType.k4X); 

		
		armSwitch = new DigitalInput(2); 

		//camera1 = CameraServer.getInstance().startAutomaticCapture(1);
		//cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
		//pixy.getVideo();
		//CameraServer.getInstance().startAutomaticCapture();
		
		m_colorMatcher.addColorMatch(kBlueTarget);
		m_colorMatcher.addColorMatch(kGreenTarget);
		m_colorMatcher.addColorMatch(kRedTarget);
		m_colorMatcher.addColorMatch(kYellowTarget);  

		//camera1 = new UsbCamera("Usb Camera", 0);

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
		//gyro.reset();
		hatchMotorEncoder.reset();
		runTime = 90;
		sandstormStartState = false;
		linearMotionState = false;
		//boolean pistonForward = false;
	}
	
	public void autonomousPeriodic() {
		autoStart();
		if (sandstormStartState){
			autoTurn(.2);
		}

	}
	
	
	public void teleopInit() {
		lastCompresserUseTime = System.currentTimeMillis();
		//gyro.reset();
		hatchMotorEncoder.reset();
		hatchMotorEncoder.setDistancePerPulse(0.5*3.14/1024);
		linearMotionState = false;
		runTime = 190;
		sandstormStartState = false;
		//boolean pistonForward = false;
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

		togglePiston(frontLegsID, frontLegs);
		togglePiston(hatchID, hatchPiston);
		togglePiston(backLegsID, backLegs);
		togglePiston(hookID, hook);
		limelight(limelightID);
		//linearMotion(linearMotionID);

		slowSpeedButton(slowSpeedButtonID);
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
		//NetworkTableInstance.create();
		System.out.println("hatchMotorEncoder.get()");
		System.out.println(hatchMotorEncoder.get());
		System.out.println(hatchMotorEncoder.getDistance());
		
		NetworkTableInstance.getDefault().getTable("Usb Camera 1").getEntry("Camera Ubs");
		//NetworkTable tables = NetworkTableInstance.getDefault().getTable("USB Camera 0");
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

	public void linearMotion (int buttonID){

		if (manipController.getRawButtonReleased(buttonID)){
			if (linearMotionState == true){
				linearMotionState = false;
				System.out.println(linearMotionState);
			}
			else if (linearMotionState == false){
				linearMotionState = true;
				System.out.println(linearMotionState);
			}
		}

		if (linearMotionState){
		if ((hatchMotorEncoder.getRaw()) <= 90) {
				//System.out.println(hatchMotorEncoder.getRaw());
				System.out.println(hatchMotorEncoder.getRaw() + "    hatch motor encoder");
				//hatchMotor.set(-0.3); 
			}else {
				//hatchMotor.set(0.0);
			}
			//hatchMotor.set(-0.3);
		} else if (linearMotionState==false){
			if ((hatchMotorEncoder.getRaw()) >= 0 ) {
				System.out.println(hatchMotorEncoder.getRaw());
				System.out.println(hatchMotorEncoder.getRaw() + "    hatch motor encoder");
				//hatchMotor.set(0.3);
			}else {
				//hatchMotor.set(0.0);
			}
			//hatchMotor.set(0.3);
		}
		
	
	}
	
		
	
	public void limelight (int buttonID) {
		if (manipController.getRawButton(buttonID)){ //if button has been pressed 
		
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
			NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry tx = table.getEntry("tx");
			NetworkTableEntry ty = table.getEntry("ty");
			NetworkTableEntry ta = table.getEntry("ta");
		
			//read values periodically
			double horizonalOffset = tx.getDouble(0.0);
			double verticalOffset = ty.getDouble(0.0);
			double targetArea = ta.getDouble(0.0);
			//double angle = gyro.getAngle();
		
			//post to smart dashboard periodically
			SmartDashboard.putNumber("LimelightX", horizonalOffset);
			SmartDashboard.putNumber("LimelightY", verticalOffset);
			SmartDashboard.putNumber("LimelightArea", targetArea);
		
			double KpL = -0.03;
		
			if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)
			==0.0){
				myRobot.arcadeDrive(0.0,0.0);
				if (x){ //"i" and "x" are placeholders
					if (i < 30){
						myRobot.arcadeDrive(0.3,0.5);
						//hatchMotor.set(-0.3);
						i += 1;
					}
					x = false;
				}
			}else{
				myRobot.arcadeDrive(Math.max((((maxSpeed-minSpeed)*((10-targetArea)/5))+minSpeed), minSpeed), -horizonalOffset*KpL);
			}
		}
		x = true;
		i = 0;
	}

	public void autoStart(){
		//double angle = gyro.getAngle();
	
		System.out.println("runTime " + runTime);
	/*
		if (driveController.getRawButtonReleased(buttonID)){
			sandstormStartState = true;
		}
	*/
		if (runTime!=0){
			//myRobot.arcadeDrive(.6, -angle*KpG); // drive towards heading 0, 0.2 probably slowest possible to run 
			runTime -= 1;
		}else{
			sandstormStartState = true;
		}
		
	}

	public void slowSpeedButton(int buttonID){
		//double angle = gyro.getAngle();
	
		//if (driveController.getRawButtonPressed(buttonID)){
			//myRobot.arcadeDrive(lowSpeed, -angle*KpG); 
		
		//}	
	}
	
	public void autoTurn(double speed){
		//double angle = gyro.getAngle();
		//if ((272 <= angle) || (angle <= 267)){
			//myRobot.arcadeDrive(speed,-(angle+90)*0.01); 
		//}
		
	}
	public void switchTest() {
		System.out.println("m_aSource.get()");
		System.out.println(m_aSource.get());
		System.out.println("m_bSource.get()");
		System.out.println(m_bSource.get());
		
	}
	//public void pixyCam(){
	//	CameraServer.getInstance().startAutomaticCapture().getVideoMode();

	//}

	public void findColor(int buttonID, Color target) {
		
		m_colorMatcher.addColorMatch(fakeRedTarget);
		m_colorMatcher.addColorMatch(fakeYellowTarget);
		m_colorMatcher.addColorMatch(fakeBlueTarget);
		m_colorMatcher.addColorMatch(fakeGreenTarget); 

		Color detectedColor = m_colorSensor.getColor();
   		//String colorString;
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
    	//String colorString;
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

