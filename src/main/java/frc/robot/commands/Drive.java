package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class Drive extends Command {
  public Drive(){
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.drive.logEvent("DRIVE");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.drive.diffDrive.arcadeDrive(OI.driverController.triggers.getTwist(), OI.driverController.leftStick.getX(), true);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
