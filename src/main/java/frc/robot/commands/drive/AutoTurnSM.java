/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.SmartMotionDrive;

public class AutoTurnSM extends Command {
  Command cmd = null;
  public AutoTurnSM(double deg) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    double leftDist = (deg / 360.0) * -7.85;
    double rightDist = (deg / 360.0) * 7.85;
    cmd = new SmartMotionDrive(leftDist, rightDist);
    this.setTimeout(2);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    cmd.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return cmd.isCompleted() || isTimedOut();
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
