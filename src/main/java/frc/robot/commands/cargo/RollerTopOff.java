package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class RollerTopOff extends Command {
  public RollerTopOff() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cargoMech);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.cargoMech.logEvent("TOP ROLLER OFF");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.cargoMech.setTop(0);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.cargoMech.setTop(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
