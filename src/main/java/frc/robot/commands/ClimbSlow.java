package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ClimbSlow extends Command {
  double speed = 0.5;
  public ClimbSlow() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.climber.logEvent("CLIMB-SLOW");
    Robot.climber.setRatchet(true);
    Robot.pneumatics.compressor.stop();
    //If vacuum isn't on, turn it on
    if (!Robot.climber.getVaccumOn()){
      Robot.climber.vacuumOn();
    }
    speed = Robot.prefs.getNumber("C: Slow Speed", 0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
      //Control the climber with the right stick, set a deadband value of 30%
      //Robot.drive.arcadeDrive(-.12, 0);
      //if (Robot.climber.getCurrent() < 35){
        Robot.climber.setClimbMotor(speed);
      //} else {
        //Robot.climber.setClimbMotor(0.0);
      //}
  }

  // Make this return true when this Command no longer needs to run execute()
  //finish when we hit the limit switch of the climber current is too high.
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.climber.setClimbMotor(0.0);
    Robot.climber.setRatchet(false);
    Robot.pneumatics.compressor.start();
    Robot.drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
