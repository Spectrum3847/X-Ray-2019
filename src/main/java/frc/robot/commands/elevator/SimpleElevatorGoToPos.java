package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class SimpleElevatorGoToPos extends Command {
  int pos;
  double upSpeed;
  double downSpeed;
  
  public SimpleElevatorGoToPos(int position) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    pos = position;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    upSpeed = Robot.prefs.getNumber("ElevatorUpSpeed", 0.6);
    downSpeed = Robot.prefs.getNumber("ElevatorDownSpeed", -0.3);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    if (pos > Robot.elevator.getPosition()){
      Robot.elevator.setOpenLoop(upSpeed);
    } else{
      Robot.elevator.setOpenLoop(downSpeed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return Robot.elevator.isNearPosition(pos);
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
