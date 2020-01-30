/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5975.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  int hatchID = 3; //X button
  /*
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
	//int limelightID= 2; //B button
	int hatchID = 3; //X button
//	int linearMotionID = 4; //Y button
//	int frontLegsID = 5; //left Bumper
	//int backLegsID = 6; //right Bumper
	//verify that stick1 and stick2 correspond to the left and right joysticks on the controller
  //6 is right, 5 is left
  
  driveController  = new XboxController(joyPort1);
  manipController = new XboxController(joyPort2);
	
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand()); 
  */
 // hatchID.whenReleased(new PistonToggle.PistonToggle(Example.hatchPiston)); 
}
