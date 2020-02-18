/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.OI;
import frc.robot.Robot;

public class PercentLight extends Command {

  double percent;
  boolean activate;

  public PercentLight() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.photon);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    activate = Robot.prefs.getBoolean("Percent Light", false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (activate){
      percent = OI.driverController.triggers.getTwist(); 
      if(percent < 0){
        Robot.photon.setAnimation(Animation.PERCENTAGE, Color.WHITE, Color.THROTTLE);

      }
      else if(percent>0){
        Robot.photon.setAnimation(Animation.PERCENTAGE, Color.PURPLE, Color.THROTTLE);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
