package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class TiltDown extends Command {
  public TiltDown() {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.cargoMech);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.cargoMech.tiltDown();
    Robot.cargoMech.logEvent("TILT DOWN");
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
    Robot.cargoMech.tiltUp();
  
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}