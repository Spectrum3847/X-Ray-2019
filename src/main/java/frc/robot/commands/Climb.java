package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class Climb extends Command {
  public Climb() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.climber.logEvent("CLIMB");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
      //Control the climber with the right stick, set a deadband value of 30%
      Robot.climber.set(1.0);
      Robot.drive.diffDrive.tankDrive(1.0, 1.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  //finish when we hit the limit switch of the climber current is too high.
  protected boolean isFinished() {
    return Robot.climber.getLimit();
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.climber.set(0.0);
    Robot.drive.diffDrive.stopMotor();;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
