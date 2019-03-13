package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SpectrumSparkMax;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.lib.util.Util;
import frc.robot.HW;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.drive.CoastMode;
import frc.robot.commands.drive.Drive;

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
  public final PigeonIMU pigeon;
  public double[] xyz_dps = new double[3];
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAccel, allowedErr;
  private int slotID = 0;
  private Command coastCmd;

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

    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();
    leftPID = leftFrontMotor.getPIDController();
    rightPID = rightFrontMotor.getPIDController();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //Follow Motors
    leftMiddleMotor.follow(leftFrontMotor);
    leftRearMotor.follow(leftFrontMotor);
    rightMiddleMotor.follow(rightFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    //Setup Motion Magic
    // PID coefficients
    kP = 5e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = .75; 
    kMinOutput = -.75;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 3000; // rpm
    maxAccel = 2000;

    //Put values into the controllers
    setPIDF(kP, kI, kD, kFF);
    setMotionMagicParams(maxVel, maxAccel);

    if(Robot.cargoMech != null){
      pigeon = new PigeonIMU(Robot.cargoMech.cargoBottomSRX);
    } else{
      pigeon = null;
    }
    
    coastCmd = new CoastMode();
  }
	
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }

  public void periodic(){
    //Get updated yaw rates
    Robot.drive.pigeon.getRawGyro(Robot.drive.xyz_dps);

    if (Robot.elevator.getPosition() > Elevator.posCargoL2){ //If elevator above cargo2, move to coast
      if (isBrake){ //only start it if we are in brakeMode.
        coastCmd.start();
      }
    } else if (coastCmd.isRunning()){//If elevator below and coastCmd is running, stop it.
      coastCmd.cancel();
    }
  }

  public void setMotionMagicParams(double maxVel, double maxAccel){
    this.maxVel = maxVel;
    this.maxAccel = maxAccel;
    leftPID.setSmartMotionMaxVelocity(maxVel, slotID);
    leftPID.setSmartMotionMaxAccel(maxAccel, slotID);

    rightPID.setSmartMotionMaxVelocity(maxVel, slotID);
    rightPID.setSmartMotionMaxAccel(maxAccel, slotID);
  }

  public void setPIDF(double kP, double kI, double kD, double kFF){
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kFF = kFF;

    leftPID.setP(this.kP);
    leftPID.setI(this.kI);
    leftPID.setD(this.kD);
    leftPID.setFF(this.kFF);

    rightPID.setP(this.kP);
    rightPID.setI(this.kI);
    rightPID.setD(this.kD);
    rightPID.setFF(this.kFF);
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

  public void coastMode(){
    isBrake = false;
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftMiddleMotor.setIdleMode(IdleMode.kCoast);
    leftRearMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightMiddleMotor.setIdleMode(IdleMode.kCoast);
    rightRearMotor.setIdleMode(IdleMode.kCoast);
  }

  public void defaultIdleMode(){
    brakeMode();
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
    if (Robot.visionLL.getLimelightHasValidTarget()){
      steerAdjust = Robot.visionLL.getLimelightSteerCommand();
    }
    arcadeDrive(speed, steerAdjust);
  }

  public void driveStraight(double speed){
    double steerAdjust = 0;
    //Super easy drive straight code based on the Z yaw rate of the pigeon, just a P loop
    if (pigeon != null){
      steerAdjust = xyz_dps[2] * Robot.prefs.getNumber("D: Straight P", 0.05);
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
  
  //Check if velocities and currents are working on each motor
	public boolean checkSystem() {
    print("Testing CARGO MECH.--------------------------------------------------");
    final double kCurrentThres = 3;
    final double kVelocityThres = 50;
    
		//Zero All Motors, turn off followers
    this.leftFrontMotor.set(0.0);
    this.rightFrontMotor.set(0.0);
    this.leftMiddleMotor.set(0.0);
    this.rightMiddleMotor.set(0.0);
    this.leftRearMotor.set(0.0);
    this.rightMiddleMotor.set(0.0);

    double testSpeed = 0.2;
    double testTime = 0.5;
		// test Drive left front
		leftFrontMotor.set(testSpeed);
		Timer.delay(testTime);
    final double currentLF = leftFrontMotor.getOutputCurrent();
    final double velocityLF = leftFrontMotor.getEncoder().getVelocity();
		leftFrontMotor.set(0);
    Timer.delay(0.25);
    
		// test Drive right front
		rightFrontMotor.set(testSpeed);
		Timer.delay(testTime);
		final double currentRF = rightFrontMotor.getOutputCurrent();
    final double velocityRF = rightFrontMotor.getEncoder().getVelocity();
		rightFrontMotor.set(0);
    Timer.delay(0.25);
    
    // test Drive leftMiddle
		leftMiddleMotor.set(testSpeed);
		Timer.delay(testTime);
		final double currentLM = leftMiddleMotor.getOutputCurrent();
    final double velocityLM = leftMiddleMotor.getEncoder().getVelocity();
		leftMiddleMotor.set(0);
    Timer.delay(0.25);
    
    // test Drive rightMiddle
		rightMiddleMotor.set(testSpeed);
		Timer.delay(testTime);
		final double currentRM = rightMiddleMotor.getOutputCurrent();
    final double velocityRM = rightMiddleMotor.getEncoder().getVelocity();
		rightMiddleMotor.set(0);
    Timer.delay(0.25);
    
    // test Drive leftRear
		leftRearMotor.set(testSpeed);
		Timer.delay(testTime);
		final double currentLR = leftRearMotor.getOutputCurrent();
    final double velocityLR = leftRearMotor.getEncoder().getVelocity();
		leftRearMotor.set(0);
    Timer.delay(0.25);

    // test Drive leftRear
    rightRearMotor.set(testSpeed);
    Timer.delay(testTime);
    final double currentRR = rightRearMotor.getOutputCurrent();
    final double velocityRR = rightRearMotor.getEncoder().getVelocity();
    rightRearMotor.set(0);
    Timer.delay(0.25);

    
    print("CURRENT LF: " + currentLF + " LM: " + currentLM + " LR: " + currentLR + " RF: " + currentRF + " RM: " + currentRM + " RR: " + currentRR);
    print("VELOCITY LF: " + velocityLF + " LM: " + velocityLM + " LR: " + velocityLR + " RF: " + velocityRF + " RM: " + velocityRM + " RR: " + velocityRR);

		boolean failure = false;

		if (currentLF < kCurrentThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE LEFT FRONT Current Low !!!!!!!!!!!!!!!!!");
    }
		if (velocityLF < kVelocityThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE LEFT FRONT VELOCITY Low !!!!!!!!!!!!!!!!!");
    }
    if (currentRF < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!!! DRIVE RIGHT FRONT Current Low !!!!!!!!!!!!!!!!!");
    }
		if (velocityRF < kVelocityThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE RIGHT FRONT VELOCITY Low !!!!!!!!!!!!!!!!!");
    }
    if (currentLM < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!!! DRIVE LEFT MIDDLE Current Low !!!!!!!!!!!!!!!!!");
    }
		if (velocityLM < kVelocityThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE LEFT MIDDLE VELOCITY Low !!!!!!!!!!!!!!!!!");
    }

		if (currentRM < kCurrentThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!DRIVE RIGHT MIDDLE Current Low !!!!!!!!!!!!!!!!!!!");
    }
		if (velocityRM < kVelocityThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE RIGHT MIDDLE VELOCITY Low !!!!!!!!!!!!!!!!!");
    }
    
    if (currentLR < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!!DRIVE LEFT REAR Current Low !!!!!!!!!!!!!!!!!!!");
    }
		if (velocityLR < kVelocityThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE LEFT REAR VELOCITY Low !!!!!!!!!!!!!!!!!");
    }

    if (currentRR < kCurrentThres) {
        failure = true;
        print("!!!!!!!!!!!!!!!!DRIVE RIGHT REAR Current Low !!!!!!!!!!!!!!!!!!!");
    }
		if (velocityRR < kVelocityThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! DRIVE RIGHT REAR VELOCITY Low !!!!!!!!!!!!!!!!!");
    }

    if (!Util.allCloseTo(Arrays.asList(currentLF, currentLM, currentLR), currentLF, 5.0)) {
      failure = true;
      print("!!!!!!!!!!!!!!!! LEFT DRIVE Currents Different !!!!!!!!!!!!!!!!!");
    }

    if (!Util.allCloseTo(Arrays.asList(velocityLF, velocityLM, velocityLR), velocityLF, 5.0)) {
      failure = true;
      print("!!!!!!!!!!!!!!!! LEFT DRIVE Velocities Different !!!!!!!!!!!!!!!!!");
    }

    if (!Util.allCloseTo(Arrays.asList(currentRF, currentRM, currentRR), currentLF, 5.0)) {
      failure = true;
      print("!!!!!!!!!!!!!!!! RIGHT DRIVE Currents Different !!!!!!!!!!!!!!!!!");
    }

    
    if (!Util.allCloseTo(Arrays.asList(velocityRF, velocityRM, velocityRR), velocityRF, 5.0)) {
      failure = true;
      print("!!!!!!!!!!!!!!!! RIGHT DRIVE Velocities Different !!!!!!!!!!!!!!!!!");
    }

    //Turn back on followers
    //Follow Motors
    leftMiddleMotor.follow(leftFrontMotor);
    leftRearMotor.follow(leftFrontMotor);
    rightMiddleMotor.follow(rightFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    return failure;
  }

}
