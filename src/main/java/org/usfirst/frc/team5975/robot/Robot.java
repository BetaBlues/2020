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

		double rightAxis = -driveController.getRawAxis(rightDriveStick);
		double leftAxis = -driveController.getRawAxis(leftDriveStick);

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		
		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		
		myRobot.tankDrive(leftAxis, rightAxis);

		//slowSpeedButton(slowSpeedButtonID);
	}
	
	
	public void teleopInit() {
		gyro.reset();
	}
	
	public void teleopPeriodic() {

		double rightAxis = -driveController.getRawAxis(rightDriveStick);
		double leftAxis = -driveController.getRawAxis(leftDriveStick);

		leftAxis = limitAxis(leftAxis);
		rightAxis = limitAxis(rightAxis);
		
		leftAxis = limitSpeed(leftAxis);
		rightAxis = limitSpeed(rightAxis);
		
		myRobot.tankDrive(leftAxis, rightAxis);

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

	public void slowSpeedButton(int buttonID){
		double angle = gyro.getAngle();
	
		if (driveController.getRawButtonPressed(buttonID)){
			myRobot.arcadeDrive(lowSpeed, -angle*KpG); 
		
		}	
	}
}

