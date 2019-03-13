package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class MMElevator extends Command {
  int t = 0;
  boolean holdPos = false;
  public MMElevator(int target) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    t = target;
  }

  public MMElevator() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    holdPos = true;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    if (holdPos){
      t = (int)Robot.elevator.getPosition();
      if (t < 0){
        t = 0;
      }
    }
    Robot.elevator.setTargetPosition(t);
    Robot.elevator.logEvent("Postion MM: " + t);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.elevator.MotionMagicControl();
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.elevator.setOpenLoop(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
