package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class RollerBottomOn extends Command {
  private double s;
  public RollerBottomOn(double speed) {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.cargoMech);
    s=speed;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.cargoMech.logEvent("BOTTOM ROLLER ON");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.cargoMech.setBottom(s);
    }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.cargoMech.setBottom(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
