package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class Drive extends Command {
  public Drive(){
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.drive.logEvent("DRIVE");
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    //Square the turn input while keeping the sign
    double turn = OI.driverController.leftStick.getX() * Math.abs(OI.driverController.leftStick.getX());
    turn = turn * 0.8;
    double throttle = OI.driverController.triggers.getTwist();

    if (OI.driverController.aButton.get() && Robot.vision.getLimelightHasValidTarget()){
      double steerAdjust = Robot.vision.getLimelightSteerCommand();
      turn = steerAdjust;
    }

    //If we aren't arcing one side, drive with throttle and turn values
    if (Math.abs(OI.driverController.rightStick.getX()) < .15){
      Robot.drive.diffDrive.arcadeDrive(throttle, turn , false);
    } else {
      double l = Math.max(OI.driverController.rightStick.getX(), 0);
      double r = Math.max((-1 * OI.driverController.rightStick.getX()), 0);
      Robot.drive.diffDrive.tankDrive(l, r);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
