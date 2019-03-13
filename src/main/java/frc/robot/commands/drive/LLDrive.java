package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.lib.util.Util;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class LLDrive extends PIDCommand {
  double m_driveValue;
  double m_MaxValue = 40;
  double m_MinValue = 0;
  int   m_lostTargetCtn = 0;
  int btnCtn = 0;

  public LLDrive(){
    // Use requires() here to declare subsystem dependencies
    super(0,0,0); // Intialize with no P, I, D, values
    requires(Robot.drive);
    m_driveValue = 0;
    setInputRange(m_MinValue, m_MaxValue);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    btnCtn = 0;
    this.m_lostTargetCtn = 0;
    Robot.drive.logEvent("LimeLight - DRIVE");
    //Get PDF values on inditalize
    double p = Robot.prefs.getNumber("V: Throttle P", 0.009);
    double d = Robot.prefs.getNumber("V: Throttle D", 0.08);
    double f = Robot.prefs.getNumber("V: Throttle F", 0.0);
    this.setPDF(p, d, f);
    this.getPIDController().setOutputRange(0, .75);
    setSetpoint(m_MaxValue);
    getPIDController().setAbsoluteTolerance(1);
    getPIDController().enable();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    double turn = OI.driverController.leftStick.getX();
    double throttle = OI.driverController.triggers.getTwist();
    Robot.photon.addAnimation("LLDrive", Animation.PULSE_TO_WHITE, Color.GREEN, Color.WHITE, 50, 2);
    
    //If we are in vision mode use it to steer and drive, if not drive normally
    if (Robot.visionLL.getLimelightHasValidTarget()){
      Robot.drive.visionDrive(m_driveValue);
      if (m_lostTargetCtn >0){
        this.m_lostTargetCtn--;
      }
    } else {      
      Robot.drive.DriverArcadeDrive(throttle, turn); // Stop if we don't have a target, let driver find target
      this.m_lostTargetCtn++;
    } 
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    //If we aren't holding and the hatch switch is hit stop driving forward.
    if (!Robot.hatch.holding & Robot.hatch.getHatchSW()){
      btnCtn++;
      if (btnCtn > 10){
        return true;
      }
    }
    return getPIDController().onTarget() || returnPIDInput() > m_MaxValue || m_lostTargetCtn > 10;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.drive.stop();
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
      double v =  Robot.visionLL.getLLDegVertical();
      return Util.clamp(v, m_MinValue, m_MaxValue); //Force inputvalues to be within range.
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
    this.m_driveValue = output;
  };

  public void setPDF(double P, double D, double F){
    getPIDController().setP(P);
    getPIDController().setD(D);
    getPIDController().setF(F);
  }

}
