/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Util;
import frc.robot.Robot;

public class SmartMotionDrive extends Command {
  double leftDistance;
  double rightDistance;
  int counter = 0;
  public SmartMotionDrive(double leftDistance, double rightDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
    this.leftDistance = leftDistance / (Math.PI/3.0) * 6.0;
    this.rightDistance = rightDistance / (Math.PI/3.0) * 6.0;
    counter = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    /**
     * As with other PID modes, Smart Motion is set by calling the
     * setReference method on an existing pid object and setting
     * the control type to kSmartMotion
     */
    counter = 0;
    Robot.drive.resetEncoders();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.leftPID.setReference(leftDistance, ControlType.kSmartMotion);
    Robot.drive.rightPID.setReference(rightDistance, ControlType.kSmartMotion);
    Robot.drive.print("left Dist: " + Robot.drive.leftEncoder.getPosition() + "Right Dist:" + Robot.drive.rightEncoder.getPosition());
    counter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return counter > 0 && onTarget();
  }

  public boolean onTarget(){
    return Util.closeTo(Robot.drive.leftEncoder.getPosition(), leftDistance, Robot.drive.leftPID.getSmartMotionAllowedClosedLoopError(0))
      && Util.closeTo(Robot.drive.rightEncoder.getPosition(), rightDistance, Robot.drive.rightPID.getSmartMotionAllowedClosedLoopError(0));
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
