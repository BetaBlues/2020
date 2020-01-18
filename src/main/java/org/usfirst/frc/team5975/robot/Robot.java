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





public class Robot extends TimedRobot {
	
	public Gyro gyro;
	double KpG = 0.03;
	
	DifferentialDrive myRobot;
	Spark leftMotor;
	Spark rightMotor;
	//boolean pistonForward;
	
	// RoboRio mapping
	int leftMotorChannel=1;
	int rightMotorChannel=2;
	DriverStation ds = DriverStation.getInstance();
	
	// Driver Station / controller mapping
	int joyPort1=0; //driver xbox controller
	int joyPort2=1; //manipulator xbox controller
	
	//Driver Controls
	int leftDriveTrigger = 2;
	int righDriveTrigger = 3;
	int yButtonDrive = 4;//Y button
	int xButtonDrive = 3;//X button
	int leftDriveStick = 1; //left and right sticks are joysticks (axis)
	int rightDriveStick = 5;
	XboxController driveController;
	XboxController manipController;
	
	//Manipulator Controls
	int bButtonManip = 2; //B button
	int xButtonManip = 3; //X button
	int yButtonManip = 4; //Y button
	int leftBumperManip = 5; //left Bumper
	int rightBumperManip = 6; //right Bumper
	//verify that stick1 and stick2 correspond to the left and right joysticks on the controller
	//6 is right, 5 is left

	//digital inputs
	final double speedScalingFactor = 0.9;
	double lowSpeed = 0.2;
	double mediumSpeed = 0.3;
	double highSpeed = 0.6;

	public void robotInit() {
		gyro = new ADXRS450_Gyro(); // Gyro on Analog Channel 1
		
		leftMotor = new Spark(leftMotorChannel);
		rightMotor = new Spark(rightMotorChannel);
		leftMotor.setInverted(false);
		rightMotor.setInverted(false);
		
		myRobot = new DifferentialDrive(leftMotor,rightMotor);
		
		driveController  = new XboxController(joyPort1);
		manipController = new XboxController(joyPort2);	
	} 
	
	public void autonomousInit(){
		gyro.reset();
	}
	
	public void autonomousPeriodic() {
		autoStart();
		if (sandstormStartState){
			autoTurn(.2);
	}
	
	
	public void teleopInit() {
		gyro.reset();
		hatchMotorEncoder.reset();
		linearMotionState = false;
		runTime = 190;
		sandstormStartState = false;
		//boolean pistonForward = false;
	}
	
	public void teleopPeriodic() {

		double rightAxis = -driveController.getRawAxis(rightDriveStick);
		double leftAxis = -driveController.getRawAxis(leftDriveStick);

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		
		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		
		myRobot.tankDrive(leftAxis, rightAxis);
		
		togglePiston(frontLegsID, frontLegs);
		togglePiston(hatchID, hatchPiston);
		togglePiston(backLegsID, backLegs);
		limelight(limelightID);
		linearMotion(linearMotionID);

		//slowSpeedButton(slowSpeedButtonID);
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
		/*if ((hatchMotorEncoder.getRaw()) <= 90) {
				//System.out.println(hatchMotorEncoder.getRaw());
				//System.out.println(hatchMotorEncoder.getRaw() + "    hatch motor encoder");
				hatchMotor.set(-0.3); 
			}else {
				hatchMotor.set(0.0);
			}*/
			hatchMotor.set(-0.3);
		} else if (linearMotionState==false){
			/*if ((hatchMotorEncoder.getRaw()) >= 0 ) {
				System.out.println(hatchMotorEncoder.getRaw());
				System.out.println(hatchMotorEncoder.getRaw() + "    hatch motor encoder");
				hatchMotor.set(0.3);
			}else {
				hatchMotor.set(0.0);
			}*/
			hatchMotor.set(0.3);
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
		double angle = gyro.getAngle();
	
		System.out.println("runTime " + runTime);
		System.out.println(angle%360);
		SmartDashboard.putNumber("Gyro", angle%360);
	/*
		if (driveController.getRawButtonReleased(buttonID)){
			sandstormStartState = true;
		}
	*/
		if (runTime!=0){
			myRobot.arcadeDrive(.6, -angle*KpG); // drive towards heading 0, 0.2 probably slowest possible to run 
			runTime -= 1;
		}else{
			sandstormStartState = true;
		}
		
	}

	public void slowSpeedButton(int buttonID){
		double angle = gyro.getAngle();
	
		if (driveController.getRawButtonPressed(buttonID)){
			myRobot.arcadeDrive(lowSpeed, -angle*KpG); 
		
		}	
	}
	public void autoTurn(double speed){
		double angle = gyro.getAngle();
		if ((272 <= angle) || (angle <= 267)){
			myRobot.arcadeDrive(speed,-(angle+90)*0.01); 
		}
		
	}
	
}

