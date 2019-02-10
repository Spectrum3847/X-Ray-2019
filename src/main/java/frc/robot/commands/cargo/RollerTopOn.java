package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class RollerTopOn extends Command {
  private double s;
  public RollerTopOn(double speed) {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.cargoMech);
    s=speed;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.cargoMech.logEvent("TOP ROLLER ON");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.cargoMech.setTop(s);
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
