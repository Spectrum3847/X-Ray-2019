package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ClimberKicker extends Command {
  public ClimberKicker() {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.climber.setKicker(true);
    Robot.climber.logEvent("CLIMBER KICKER");
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
    Robot.climber.setKicker(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
