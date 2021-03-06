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
  SetElevator c;
  
  public SimpleElevatorGoToPos(int position) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    pos = position;
    
    //Command used at the end to set the elevator in place
    c = new SetElevator(0.05);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    if (Robot.elevator.isNearPosition(pos)){
      upSpeed = 0;
      downSpeed = 0;
    } else{
      upSpeed = Robot.prefs.getNumber("ElevatorUpSpeed", 0.8);
      downSpeed = Robot.prefs.getNumber("ElevatorDownSpeed", -0.4);
    }
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
    //start a new command that gives a little bit of voltage to hold elevator in place
    c.start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
