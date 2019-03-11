package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SpectrumSparkMax;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.HW;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.LLDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {

  public final SpectrumSparkMax leftFrontMotor;
  public final SpectrumSparkMax leftMiddleMotor;
  public final SpectrumSparkMax leftRearMotor;
  public final SpectrumSparkMax rightFrontMotor;
  public final SpectrumSparkMax rightMiddleMotor;
  public final SpectrumSparkMax rightRearMotor;
  public final CANEncoder leftEncoder;
  public final CANEncoder rightEncoder;
  public final CANPIDController leftPID;
  public final CANPIDController rightPID;

  //set this up after we setup the cargo mechanism with the pigeon on it
  //public PigeonIMU imu = new PigeonIMU(1);

  double targetAngle;
	double turnThrottle;
	
	double currentAngle;
	double currentAngularRate;
  public double error = 0;
  private boolean isBrake = false;

  private double maxOutput = 1.0;
  private double maxTurnRate = 0.8;
  
  public Drivetrain() {
    super("drivetrain");
    leftFrontMotor = new SpectrumSparkMax(HW.LEFT_DRIVE_FRONT);
    leftMiddleMotor = new SpectrumSparkMax(HW.LEFT_DRIVE_MIDDLE);
    leftRearMotor = new SpectrumSparkMax(HW.LEFT_DRIVE_REAR);
    rightFrontMotor = new SpectrumSparkMax(HW.RIGHT_DRIVE_FRONT);
    rightMiddleMotor = new SpectrumSparkMax(HW.RIGHT_DRIVE_MIDDLE);
    rightRearMotor = new SpectrumSparkMax(HW.RIGHT_DRIVE_REAR);

    leftFrontMotor.restoreFactoryDefaults();
    leftMiddleMotor.restoreFactoryDefaults();
    leftRearMotor.restoreFactoryDefaults();
    rightMiddleMotor.restoreFactoryDefaults();
    rightMiddleMotor.restoreFactoryDefaults();
    rightRearMotor.restoreFactoryDefaults();

    leftFrontMotor.setInverted(true);
    leftMiddleMotor.setInverted(true);
    leftRearMotor.setInverted(true);
    rightFrontMotor.setInverted(false);
    rightMiddleMotor.setInverted(false);
    rightRearMotor.setInverted(false);

    defaultIdleMode();

    /*double voltageCompensation = 10.5;
    leftFrontMotor.enableVoltageCompensation(voltageCompensation);
    leftMiddleMotor.enableVoltageCompensation(voltageCompensation);
    leftRearMotor.enableVoltageCompensation(voltageCompensation);
    rightFrontMotor.enableVoltageCompensation(voltageCompensation);
    rightMiddleMotor.enableVoltageCompensation(voltageCompensation);
    rightRearMotor.enableVoltageCompensation(voltageCompensation);*/

    int currentLimit = 65;
    leftFrontMotor.setSmartCurrentLimit(currentLimit);
    leftMiddleMotor.setSmartCurrentLimit(currentLimit);
    leftRearMotor.setSmartCurrentLimit(currentLimit);
    rightFrontMotor.setSmartCurrentLimit(currentLimit);
    rightMiddleMotor.setSmartCurrentLimit(currentLimit);
    rightRearMotor.setSmartCurrentLimit(currentLimit);

    double rampRate = 0.0;
    leftFrontMotor.setOpenLoopRampRate(rampRate);
    rightFrontMotor.setOpenLoopRampRate(rampRate);

    leftEncoder = new CANEncoder(leftFrontMotor);
    rightEncoder = new CANEncoder(rightFrontMotor);
    leftPID = new CANPIDController(leftFrontMotor);
    rightPID = new CANPIDController(rightFrontMotor);
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //Follow Motors
    leftMiddleMotor.follow(leftFrontMotor);
    leftRearMotor.follow(leftFrontMotor);
    rightMiddleMotor.follow(rightFrontMotor);
    rightRearMotor.follow(rightFrontMotor);
  }
	
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }

  public void periodic(){
  }

  public void brakeMode(){
    isBrake = true;
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftMiddleMotor.setIdleMode(IdleMode.kBrake);
    leftRearMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightMiddleMotor.setIdleMode(IdleMode.kBrake);
    rightRearMotor.setIdleMode(IdleMode.kBrake);
  }

  public void defaultIdleMode(){
    isBrake = false;
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftMiddleMotor.setIdleMode(IdleMode.kCoast);
    leftRearMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightMiddleMotor.setIdleMode(IdleMode.kCoast);
    rightRearMotor.setIdleMode(IdleMode.kCoast);
  }

  public boolean getIsBrake(){
    return isBrake;
  }

  public void zeroSensors() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    /*NEED TO ZERO PIGEON HERE*/
  }

    /**
   * Limit motor values to the -1.0 to +1.0 range.
   */
  protected double limit(double value) {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }

    /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public void DriverArcadeDrive(double throttle, double steer){
    steer = Math.copySign(Math.pow(steer,2), steer);
    steer = limit(steer) * maxTurnRate;
    arcadeDrive(throttle, steer);
  }
  public void arcadeDrive(double xSpeed, double zRotation) {
    xSpeed = limit(xSpeed);

    //Make the deadzone bigger if we are driving fwd or backwards and not turning in place
    if (Math.abs(xSpeed) > 0.1 && Math.abs(zRotation)< 0.05){
      zRotation = 0;
    }
    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    if (xSpeed == 0){
      leftMotorOutput = zRotation;
      rightMotorOutput = -zRotation;
    } else {
      if (xSpeed >= 0.0) {
        // First quadrant, else second quadrant
        if (zRotation >= 0.0) {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
        } else {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
        }
      } else {
        // Third quadrant, else fourth quadrant
        if (zRotation >= 0.0) {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
        } else {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
        }
      }
    }

    leftFrontMotor.set(limit(leftMotorOutput) * maxOutput);
    rightFrontMotor.set(limit(rightMotorOutput) * maxOutput);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftSpeed = limit(leftSpeed);

    rightSpeed = limit(rightSpeed);

    leftFrontMotor.set(leftSpeed * maxOutput);
    rightFrontMotor.set(rightSpeed * maxOutput);
  }

  public void visionDrive(double speed){
    double steerAdjust = 0;
    if (OI.driverController.aButton.get() && Robot.visionLL.getLimelightHasValidTarget()){
      steerAdjust = Robot.visionLL.getLimelightSteerCommand();
    }
    arcadeDrive(speed, steerAdjust);
  }

  public void stop(){
    leftFrontMotor.set(0.0);
    rightFrontMotor.set(0.0);
  }

  public void dashboard(){
    //Add values that need to be updated on the dashboard.
    SmartDashboard.putNumber("Drive/left-output", leftFrontMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Drive/leftM-output", leftMiddleMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Drive/leftR-output", leftRearMotor.getAppliedOutput());
    SmartDashboard.putNumber("Drive/left-pos", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive/left-velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/right-output", rightFrontMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Drive/rightM-output", rightMiddleMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Drive/rightR-output", rightRearMotor.getAppliedOutput());
    SmartDashboard.putNumber("Drive/right-pos", rightEncoder.getPosition());
    SmartDashboard.putNumber("Drive/right-velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/SteerStick", OI.driverController.leftStick.getX());
    SmartDashboard.putNumber("Drive/left-Current", leftFrontMotor.getOutputCurrent());
    SmartDashboard.putNumber("Drive/right-Current", rightFrontMotor.getOutputCurrent());
  }

  public void printDebug(String msg){
    Debugger.println(msg, Robot._drive, Debugger.debug2);
  }
  
  public void printInfo(String msg){
    Debugger.println(msg, Robot._drive, Debugger.info3);
  }
  
  public void printWarning(String msg) {
    Debugger.println(msg, Robot._drive, Debugger.warning4);
  }

  public void print(String msg){
    System.out.println(msg);
  }

  public void logEvent(String event){
		SpectrumLogger.getInstance().addEvent(Robot._drive, event);
	}
	public boolean checkSystem() {
    return true;
  }
}
