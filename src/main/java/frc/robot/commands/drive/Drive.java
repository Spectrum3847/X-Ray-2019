package frc.robot.commands.drive;

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
    double turn = OI.driverController.leftStick.getX();
    double throttle = OI.driverController.triggers.getTwist();

    //Robot.drive.print("aBut: " + OI.driverController.aButton.get() + " valid: " + Robot.vision.getLimelightHasValidTarget());
    //If we are in single side steering, drive normally or by vision
    if (Math.abs(OI.driverController.rightStick.getX()) < .2){
      //If we are in vision mode use it to steer, if not drive normally, also check that elevator isn't blocking vision
      if (OI.driverController.aButton.get() && Robot.visionLL.getLimelightHasValidTarget() && !Robot.elevator.blockingVision()){
        Robot.drive.print("VISION TURNING!!!");
        Robot.drive.visionDrive(throttle);
      } else {      
        Robot.drive.DriverArcadeDrive(throttle, turn);
      } 
    }
    else {//Single side steering and put us in brake mode for it.
      double l = Math.max(OI.driverController.rightStick.getX(), 0);
      double r = Math.max((-1 * OI.driverController.rightStick.getX()), 0);
      Robot.drive.tankDrive(l * .9 , r * .9);
    }
    //If we aren't arcing one side, drive with throttle and turn values
    
    //Robot.drive.printDebug("Turn: " + turn + " Throttle: " + throttle);
    //Robot.drive.printDebug("LeftOut: " + Robot.drive.leftFrontMotor.get() + " RightOut: " + Robot.drive.rightFrontMotor.get());
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
