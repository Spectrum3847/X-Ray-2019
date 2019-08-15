package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ClimbRev extends Command {
  public ClimbRev() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    //Disable Ratchet
    Robot.climber.setRatchet(true);
    Robot.pneumatics.compressor.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    //Raise the climber
    Robot.climber.setClimbMotor(-1.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    //Turn climber motor off
    Robot.climber.setClimbMotor(0);
    
    Robot.pneumatics.compressor.start();

    //Rengauge the ratchet
    Robot.climber.setRatchet(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
