/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */ 
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5975.robot.commands;

//import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team5975.robot.subsystems.Piston;

/**
 * Add your docs here.
 */
public class PistonToggle {
  /**
   * Add your docs here.
   */

  public PistonToggle(Piston pistonName) {
    super();
    pistonName.toggle();

  }

  // Called once when the command executes
  

}
