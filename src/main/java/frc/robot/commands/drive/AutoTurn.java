package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.lib.util.Util;
import frc.robot.HW;
import frc.robot.Robot;
import frc.robot.commands.BrakeMode;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoTurn extends PIDCommand {
  double m_steerValue;
  double m_MaxValue = 180;
  double m_MinValue = -180;
  double target;
  boolean clockwiseTurn = true; //Negative is clockwise
  Button driverButton = null;

  public AutoTurn(double deg) {

    super(0,0,0); // Intialize with no P, I, D, values
    requires(Robot.drive);
    m_steerValue = 0;
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
    Robot.drive.logEvent("AutoDrive");
    //Get PDF values on inditalize
    double p = Robot.prefs.getNumber("D: Steer P", 0.1);
    double d = Robot.prefs.getNumber("D: Steer D", 0.0);
    double f = Robot.prefs.getNumber("D: Steer F", 0.0);
    Robot.pigeon.setYaw(0); //Set Yaw to Zero
    this.setPDF(p, d, f);
    setSetpoint(target);
    getPIDController().enable();
    this.getPIDController().setOutputRange(-.8, .8);
    setTimeout(2);
    Robot.drive.brakeMode();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
      Robot.drive.DriverArcadeDrive(0, m_steerValue);

      //Used to set our setupoint to 180 if you hold the driver button instead of just tap it.
      if (driverButton != null && Math.abs(this.returnPIDInput()) > 45 && Math.abs(getSetpoint()) != 180){
        if (driverButton.get()){
          setSetpoint(180 * Math.signum(target));
        }
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    if (Math.abs(target) < Math.abs(returnPIDInput())){
      return true;  //If we over shot, just stop
    }
    if (isTimedOut()){
      return true;
    }
    return getPIDController().onTarget();
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.drive.stop();
    Robot.drive.defaultIdleMode();
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
      return Robot.pigeon.getFusedHeading();
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
    this.m_steerValue = output;
  };

  public void setPDF(double P, double D, double F){
    getPIDController().setP(P);
    getPIDController().setD(D);
    getPIDController().setF(F);
  }

}