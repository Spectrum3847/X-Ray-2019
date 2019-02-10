package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class HatchEject extends Command {
  public HatchEject() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.hatch.hatchEject();
    Robot.hatch.logEvent("HATCH EJECT");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.hatch.hatchRetract();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  end();
  }
}
