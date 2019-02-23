package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SpectrumSparkMax;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.HW;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.Drive;

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

  public DifferentialDrive diffDrive;;

  //set this up after we setup the cargo mechanism with the pigeon on it
  //public PigeonIMU imu = new PigeonIMU(1);

  double targetAngle;
	double turnThrottle;
	
	double currentAngle;
	double currentAngularRate;
  public double error = 0;
  
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
    rightFrontMotor.setInverted(true);
    rightMiddleMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    defaultIdleMode();

    leftEncoder = new CANEncoder(leftFrontMotor);
    rightEncoder = new CANEncoder(rightFrontMotor);
    leftPID = new CANPIDController(leftFrontMotor);
    rightPID = new CANPIDController(rightFrontMotor);
    diffDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    diffDrive.setSubsystem(this.getName());
    diffDrive.setDeadband(0.04);
    diffDrive.setMaxOutput(1.0);
    diffDrive.setSafetyEnabled(true);
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
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftMiddleMotor.setIdleMode(IdleMode.kBrake);
    leftRearMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightMiddleMotor.setIdleMode(IdleMode.kBrake);
    rightRearMotor.setIdleMode(IdleMode.kBrake);
  }

  public void defaultIdleMode(){
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftMiddleMotor.setIdleMode(IdleMode.kCoast);
    leftRearMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightMiddleMotor.setIdleMode(IdleMode.kCoast);
    rightRearMotor.setIdleMode(IdleMode.kCoast);
  }

  public void zeroSensors() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    /*NEED TO ZERO PIGEON HERE*/
  }

  public void dashboard(){
    //Add values that need to be updated on the dashboard.
    SmartDashboard.putNumber("Drive/left-output", leftFrontMotor.getAppliedOutput());
    SmartDashboard.putNumber("Drive/left-pos", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive/left-velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/right-output", rightFrontMotor.getAppliedOutput());
    SmartDashboard.putNumber("Drive/right-pos", rightEncoder.getPosition());
    SmartDashboard.putNumber("Drive/right-velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/SteerStick", OI.driverController.leftStick.getX());
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
