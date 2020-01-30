package org.usfirst.frc.team5975.robot;
import org.usfirst.frc.team5975.robot.subsystems.Piston;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import org.usfirst.frc.team5975.robot.subsystems.Pixy2;
import org.usfirst.frc.team5975.robot.subsystems.Pixy2.LinkType;
import org.usfirst.frc.team5975.robot.subsystems.links.Link;
import org.usfirst.frc.team5975.robot.subsystems.links.SPILink;
//import edu.wpi.first.wpilibj.CameraServer;
//import com.revrobotics.ColorSensorV3;
import org.usfirst.frc.team5975.robot.subsystems.links.UARTLink;

//import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.util.ColorShim;

// Limelight Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//encoder imports
//import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;

import org.usfirst.frc.team5975.robot.subsystems.Camera;

//import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoSource;

public class Robot extends TimedRobot {
	
	//public Gyro gyro;
	double KpG = 0.03;
	boolean x = true;
	
	DifferentialDrive myRobot;
	Spark leftMotor;
	Spark rightMotor;
	Spark hatchMotor;
	Piston frontLegs;
	Piston hatchPiston;
	Piston backLegs;
	//boolean pistonForward;
	
	// RoboRio mapping
	int leftMotorChannel=1;
	int rightMotorChannel=2;
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
	XboxController driveController;
	XboxController manipController;
	
	//Manipulator Controls
	int limelightID= 2; //B button
	int hatchID = 3; //X button
	int linearMotionID = 4; //Y button
	int frontLegsID = 5; //left Bumper
	int backLegsID = 6; //right Bumper
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

	VideoSource cam;
	boolean limelightButtonState = false;

	double maxSpeed = 0.5; // max speed for the limelight fucntion
	double minSpeed = 0.3; // min speed for the limelight fuction
	int runTime; //number of cycles function has gone through
	

	Link link = new UARTLink();
	boolean sandstormStartState = false; //the state of the button to start sandstorm 

	 Pixy2 pixy;

	 I2C.Port i2cPort = I2C.Port.kOnboard;
	//UsbCamera camera1;
	NetworkTableEntry cameraSelection;
  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
 // private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  //private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  //private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
 // private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
 // private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


	
	public void robotInit() {
		//gyro = new ADXRS450_Gyro(); // Gyro on Analog Channel 1
		
		frontLegs = new Piston (0,1, "front legs");
		backLegs = new Piston (5,4, "back legs");
		hatchPiston = new Piston (2,3, "hatch");

		leftMotor = new Spark(leftMotorChannel);
		rightMotor = new Spark(rightMotorChannel);
		//hatchMotor = new Spark(hatchMotorChannel);
		leftMotor.setInverted(false);
		rightMotor.setInverted(false);
		
		myRobot = new DifferentialDrive(leftMotor,rightMotor);
		
		driveController  = new XboxController(joyPort1);
		manipController = new XboxController(joyPort2);
		
		hatchMotorEncoder = new Encoder(3,4, false, Encoder.EncodingType.k4X); 

		Pixy2 pixy = Pixy2.createInstance(link);
		pixy.init();
		//m_aSource = new DigitalInput(3);
		//m_bSource = new DigitalInput(4);

		//camera1 = CameraServer.getInstance().startAutomaticCapture(1);
		//cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
		pixy.getVideo();
		CameraServer.getInstance().startAutomaticCapture();
		
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
		boolean pistonForward = false;
	}
	
	public void teleopPeriodic() {

		double rightAxis = -driveController.getRawAxis(rightStickID);
		double leftAxis = -driveController.getRawAxis(leftStickID);

		//switchTest(); //which channels?

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		
		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		
		myRobot.tankDrive(leftAxis, rightAxis);
		
		togglePiston(frontLegsID, frontLegs);
		togglePiston(hatchID, hatchPiston);
		togglePiston(backLegsID, backLegs);
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
		//SmartDashboard.put(pixy.getVideo());
		//pixy.getVideo();

		System.out.println("Setting camera 1");
		//cameraSelection.setString(camera1.getName());
	

		//NetworkTableInstance.getDefault().getTable("USB Camera 0");
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
/*
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
	*/
		
	
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
	public void pixyCam(){
		CameraServer.getInstance().startAutomaticCapture().getVideoMode();

	}
}

