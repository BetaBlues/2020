package org.usfirst.frc.team5975.robot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// Limelight Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//encoder imports
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.Encoder;

public class Robot extends TimedRobot {
	
	public Gyro gyro;
	double KpG = 0.03;
	boolean x = true;
	
	RobotDrive myRobot;
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
	int hatchMotorChannel=5;
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
	int hatchForwardID = 4; //Y button
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
	boolean linearMotionState = true;
	

	boolean limelightButtonState = false;

	double maxSpeed = 0.5; // max speed for the limelight fucntion
	double minSpeed = 0.3; // min speed for the limelight fuction
	int runTime; //number of cycles function has gone through
	
	boolean sandstormStartState = false; //the state of the button to start sandstorm 
	
	public void robotInit() {
		gyro = new ADXRS450_Gyro(); // Gyro on Analog Channel 1
		
		frontLegs = new Piston (0,1, "front legs");
		backLegs = new Piston (5,4, "back legs");
		hatchPiston = new Piston (2,3, "hatch");

		leftMotor = new Spark(leftMotorChannel);
		rightMotor = new Spark(rightMotorChannel);
		hatchMotor = new Spark(hatchMotorChannel);
		leftMotor.setInverted(false);
		rightMotor.setInverted(false);
		
		myRobot = new RobotDrive(leftMotor,rightMotor);
		
		driveController  = new XboxController(joyPort1);
		manipController = new XboxController(joyPort2);
		
		hatchMotorEncoder = new Encoder(7,8); 
		
	} 
	public void autonomousInit(){
		lastCompresserUseTime = System.currentTimeMillis();
		gyro.reset();
		hatchMotorEncoder.reset();
		runTime = 90;
		sandstormStartState = false;
		//boolean pistonForward = false;
	}
	
	public void autonomousPeriodic() {
		

		double rightAxis = -driveController.getRawAxis(rightStickID);
		double leftAxis = -driveController.getRawAxis(leftStickID);

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		
		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		
		myRobot.tankDrive(leftAxis, rightAxis);
		
		togglePiston(frontLegsID, frontLegs);
		togglePiston(hatchID, hatchPiston);
		togglePiston(backLegsID, backLegs);
		limelight(limelightID);
		hatchForward(hatchForwardID);
		sandstormStart(sandstormStartID);

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
	}
	
	
	public void teleopInit() {
		lastCompresserUseTime = System.currentTimeMillis();
		gyro.reset();
		hatchMotorEncoder.reset();
		runTime = 90;
		sandstormStartState = false;
		//boolean pistonForward = false;
	}
	
	public void teleopPeriodic() {

		//hatchMotor.set(0.2);


		double rightAxis = -driveController.getRawAxis(rightStickID);
		double leftAxis = -driveController.getRawAxis(leftStickID);

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		
		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		
		myRobot.tankDrive(leftAxis, rightAxis);
		
		togglePiston(frontLegsID, frontLegs);
		togglePiston(hatchID, hatchPiston);
		togglePiston(backLegsID, backLegs);
		limelight(limelightID);
		hatchForward(hatchForwardID);
		sandstormStart(sandstormStartID);

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
	
		if (manipController.getRawButtonReleased(buttonID)){ //if the button is being pressed
		
			pistonn.toggle();
			lastCompresserUseTime = System.currentTimeMillis();
		}
	}

	public void hatchForward (int buttonID){

		if (manipController.getRawButtonReleased(buttonID)){
			if (linearMotionState == true){
				linearMotionState = false;
			}
			else if (linearMotionState == false){
				linearMotionState = true;
			}
		}

		if (linearMotionState){
			if ((hatchMotorEncoder.getRaw()) <= 90) {
				System.out.println(hatchMotorEncoder.getRaw());
				System.out.println(hatchMotorEncoder.getRaw() + "    hatch motor encoder");
				hatchMotor.set(-0.3); 
			}else {
				hatchMotor.set(0.0);
			}
		} else if (linearMotionState==false){
			if ((hatchMotorEncoder.getRaw()) >= 0 ) {
				System.out.println(hatchMotorEncoder.getRaw());
				System.out.println(hatchMotorEncoder.getRaw() + "    hatch motor encoder");
				hatchMotor.set(0.3);
			}else {
				hatchMotor.set(0.0);
			}
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
		
			//post to smart dashboard periodically
			SmartDashboard.putNumber("LimelightX", horizonalOffset);
			SmartDashboard.putNumber("LimelightY", verticalOffset);
			SmartDashboard.putNumber("LimelightArea", targetArea);
		
			double KpL = -0.03;
		
			if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)
			==0.0){
				myRobot.drive(0.0,0.0);
				if (x){ //"i" and "x" are placeholders
					if (i < 30){
						myRobot.drive(0.3,0.0);
						//hatchMotor.set(-0.3);
						i += 1;
					}
					x = false;
				}
			}else{
				myRobot.drive(Math.max((((maxSpeed-minSpeed)*((10-targetArea)/10))+minSpeed), minSpeed), -horizonalOffset*KpL);
			}
		}
		x = true;
		i = 0;
	}

	public void sandstormStart(int buttonID){
		double angle = gyro.getAngle();
	
		System.out.println("runTime " + runTime);
		System.out.println(angle%360);
		SmartDashboard.putNumber("Gyro", angle%360);
	
		if (driveController.getRawButtonReleased(buttonID)){
			sandstormStartState = true;
		}
	
		if (runTime!=0 && sandstormStartState){
			myRobot.drive(.5, -angle*KpG); // drive towards heading 0, 0.2 probably slowest possible to run 
			runTime -= 1;
		}
		
	}

	public void slowSpeedButton(int buttonID){
		double angle = gyro.getAngle();
	
		if (driveController.getRawButtonPressed(buttonID)){
			myRobot.drive(lowSpeed, -angle*KpG); 
		
		}	
	}
}

