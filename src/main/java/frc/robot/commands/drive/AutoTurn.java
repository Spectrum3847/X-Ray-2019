package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.lib.util.Util;
import frc.robot.HW;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoTurn extends PIDCommand {
  double m_steerValue = 0;
  double m_MaxValue = 180;
  double m_MinValue = -180;
  double target = 0;
  boolean clockwiseTurn = true; //Negative is clockwise
  Button driverButton = null;
  boolean gyroIsReset = false;
  int btnCounter = 0;

  public AutoTurn(double deg) {

    super(0,0,0); // Intialize with no P, I, D, values
    requires(Robot.drive);
    target = Util.clamp(deg, m_MinValue, m_MaxValue);

    //Check if we are turning counterclockwise
    if (target > 0){
      clockwiseTurn = false;
    }

    getPIDController().setAbsoluteTolerance(2);
    setInputRange(m_MinValue, m_MaxValue);

  }

  public AutoTurn(double deg, Button driverButton){
    this(deg);
    this.driverButton = driverButton;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    gyroIsReset = false;
    //this.updatePrefsPDF();
    this.setPDF(0.04,0,0.1);
    btnCounter = 0;
    if (Robot.drive.pigeon.setFusedHeading(0).value > 0){
      Robot.drive.printDebug("ERROR ON SETTING HEADING");
    }; 
    setSetpoint(target);             //Move are target to the right spot
    m_steerValue = 0;                //Set steerValue to zero
    //Robot.drive.logEvent("AutoTurn Started target: " + target +" Heading: " + Robot.pigeon.getFusedHeading());
    //Get PDF values on inditalize
    this.getPIDController().setOutputRange(-.8, .8);
    setTimeout(10);
    Robot.drive.brakeMode();
    gyroIsReset = true;
    getPIDController().enable();
    Robot.photon.addAnimation("AutoTurn", Animation.BLINK_DUAL, Color.WHITE, Color.PURPLE, 50, 5);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
      if (getPIDController().isEnabled()){
        //Robot.drive.printDebug("SteerValue: " + m_steerValue);
        Robot.drive.DriverArcadeDrive(0, m_steerValue);
      }

      //Used to set our setupoint to 180 if you hold the driver button instead of just tap it.
      if (driverButton != null && Math.abs(getSetpoint()) != 180){
        if (driverButton.get()){
          btnCounter++;
        }
        if (btnCounter >= 12){
          setSetpoint(2 * target);
          //Robot.drive.logEvent("180 Turn: " + this.getSetpoint());
        }
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    //Check if we have reset the gyro, if not don't finish until we do.
    if (gyroIsReset && this.timeSinceInitialized() > 0.2){
      if (Math.abs(this.getSetpoint()) < Math.abs(returnPIDInput())){
        Robot.drive.printDebug("Auto Turn Overshot");
        return true;  //If we over shot, just stop
      }
      if (isTimedOut()){
        Robot.drive.printDebug("Auto Turn Timeout");
        return true;
      }
      return false; //getPIDController().onTarget();
    } else{
      return false;
    }
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.drive.stop();
    Robot.drive.defaultIdleMode();
    getPIDController().disable();
    setSetpoint(0);                         //Reset the target on exit
    Robot.drive.logEvent("Turn Finished");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }

  /**
   * Returns the input for the pid loop.
   *
   * <p>It returns the input for the pid loop, so if this command was based off of a gyro, then it
   * should return the angle of the gyro.
   *
   * <p>All subclasses of {@link PIDCommand} must override this method.
   *
   * <p>This method will be called in a different thread then the {@link Scheduler} thread.
   *
   * @return the value the pid loop should use as input
   */
  protected double returnPIDInput(){
      return Robot.drive.pigeon.getFusedHeading();
  };

  /**
   * Uses the value that the pid loop calculated. The calculated value is the "output" parameter.
   * This method is a good time to set motor values, maybe something along the lines of
   * <code>driveline.tankDrive(output, -output)</code>
   *
   * <p>All subclasses of {@link PIDCommand} must override this method.
   *
   * <p>This method will be called in a different thread then the {@link Scheduler} thread.
   *
   * @param output the value the pid loop calculated
   */
  protected void usePIDOutput(double output){
    this.m_steerValue = output * -1;
  };

  public void setPDF(double P, double D, double F){
    getPIDController().setP(P);
    getPIDController().setD(D);
    getPIDController().setF(F);
  }

  public void updatePrefsPDF(){
    double p = Robot.prefs.getNumber("D: Steer P", 0.1);
    double d = Robot.prefs.getNumber("D: Steer D", 0.0);
    double f = Robot.prefs.getNumber("D: Steer F", 0.0);
    this.setPDF(p, d, f);
  }

}