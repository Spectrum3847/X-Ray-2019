package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LeaderTalonSRX;
import frc.lib.drivers.SpectrumVictorSPX;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.lib.util.Util;
import frc.robot.HW;
import frc.robot.Robot;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.MMElevator;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // needs to be determined manually
  public final static int posDownLimit = 0;
  public final static int posCargoL1 = 4500;
  public final static int posCargoShip = 10500;
  public final static int posHatchL2 = 15000;
  public final static int posCargoL2 = 18000;
  public final static int posHatchL3 = 29400;
  public final static int posCargoL3 = 32500;
  public final static int posUpLimit = 32500;

  private int targetPosition = 0;
  private int epsilon = 1000;

  private int kMaxSensorVelocity = 3500;
  private int kCruiseVelocity = 3400;
  private int kAcceleration = kCruiseVelocity * 3; //Set accerlaeration to happen in half a second
  private double kF = 0.292;
  private double kP = 0.7;
  private double kI = 0.0;
  private double kD = 80.0;

  private final SpectrumVictorSPX spx;
  public final LeaderTalonSRX srx;
  // private boolean zeroWhenDownLimit = true;

  // // private int targetPosition = 0;
  // // private int accel = 0;
  // // private int cruiseVel = 0;

  public Elevator() {
    super("Elevator");
    boolean elevatorInvert = false;
    boolean elevatorPhase = false;
    spx = new SpectrumVictorSPX(HW.ELEVATOR_SPX);
    srx = new LeaderTalonSRX(HW.ELEVATOR_SRX, spx);
    srx.setNeutralMode(NeutralMode.Brake);
    srx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    srx.setSensorPhase(elevatorPhase);
    srx.setInverted(elevatorInvert);

    // Enable Soft Limits
    srx.configForwardSoftLimitEnable(false);
    srx.configForwardSoftLimitThreshold(posUpLimit);
    srx.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  
    srx.configReverseSoftLimitEnable(false);
    srx.configReverseSoftLimitThreshold(posDownLimit);
    srx.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    srx.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    srx.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    // Set Ramp Rate and voltage compensation
    srx.configClosedloopRamp(0.25);
    srx.configVoltageCompSaturation(11.5);
    srx.enableVoltageCompensation(true);
    setMotionMagicParams();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MMElevator());
  }

  public void softLimitsOn(boolean v){
    srx.configReverseSoftLimitEnable(v);
    srx.configForwardSoftLimitEnable(v);
  }

  public void zeroPosition(){
    srx.setSelectedSensorPosition(0);
  }

  public void setOpenLoop(double speed) {
    srx.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    setOpenLoop(0.0);
  }

  public int getTargetPosition() {
    return this.targetPosition;
  }

  public boolean getBottomLimitSW() {
    return srx.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean getTopLimitSW() {
    return srx.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public double getPosition() {
    return srx.getSelectedSensorPosition();
  }

  public boolean isNearPosition(int pos){
    return Util.closeTo(pos, getPosition(), epsilon);
  }

  public double getCurrentDraw() {
    return srx.getOutputCurrent();
  }

  public boolean setTargetPosition(int position) {
    if (!isValidPosition(position)) {
      return false;
    } else {
      this.targetPosition = position;
      return true;
    }
  }

  public boolean blockingVision(){
    if (getPosition() >  this.posCargoL1 && getPosition() < this.posHatchL2){
      return true;
    }
    return false;
  }

  public void MotionMagicControl(){
    srx.set(ControlMode.MotionMagic, targetPosition);
  }

  public void setMotionMagicParams(){
    srx.configPIDF(0, kP, kI, kD, kF);
    srx.configMotionCruiseVelocity(kCruiseVelocity);
    srx.configMotionAcceleration(kAcceleration);
  }

  public void forceSetTargetPosition(int position) {
    this.targetPosition = position;
  }

  public void incrementTargetPosition(int increment) {
    int currentTargetPosition = this.targetPosition;
    int newTargetPosition = currentTargetPosition + increment;
    if (isValidPosition(newTargetPosition)) {
      this.targetPosition = newTargetPosition;
    }
  }

  public boolean isValidPosition(int position) {
    boolean withinBounds = position <= posUpLimit && position >= posDownLimit;
    return withinBounds;
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._elevator, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._elevator, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._elevator, Debugger.warning4);
  }

  public static void print(String msg){
		System.out.println(msg);
  }

  public void logEvent(String event){
    printDebug(event);
		SpectrumLogger.getInstance().addEvent(Robot._elevator, event);
	}

  // Modify to run a system check for the robot
  public boolean checkSystem() {
    print("Testing ELEVATOR.--------------------------------------------------");
    final double kCurrentThres = 0.5;
    if(getPosition() < 0){
      zeroPosition();
    }
    spx.set(ControlMode.PercentOutput, -0.1);
    srx.set(ControlMode.PercentOutput, -0.1);
    Timer.delay(0.2);
    srx.set(ControlMode.PercentOutput, 0.0);
    spx.set(ControlMode.PercentOutput, 0.0);
    Timer.delay(0.1);
;   srx.setNeutralMode(NeutralMode.Coast);
    spx.setNeutralMode(NeutralMode.Coast);

    double testSpeed = .75;
    double testDownSpeed = -0.05;
    double testUpTime = 1;


    if(getPosition() < 0){
      zeroPosition();
    }
    // test climber srx
    final double SRXintialEncoderPos = Math.abs(getPosition());
    srx.set(ControlMode.PercentOutput, testSpeed);
    Timer.delay(testUpTime);
    srx.set(ControlMode.PercentOutput, 0.0);
    final double currentSRX = HW.PDP.getCurrent(HW.ELEVATOR_SRX_PDP);
    final double positionSRX = getPosition();
    //srx.set(ControlMode.PercentOutput, testDownSpeed);
    //Timer.delay(0.1);
    
    Timer.delay(2.0);

    if(getPosition() < 0){
      zeroPosition();
    }

    // Test climber spx
    final double SPXintialEncoderPos = Math.abs(getPosition());
    spx.set(ControlMode.PercentOutput, testSpeed);
    Timer.delay(testUpTime);
    final double currentSPX = HW.PDP.getCurrent(HW.ELEVATOR_SPX_PDP);
    final double positionSPX = getPosition();
    spx.set(ControlMode.PercentOutput, testDownSpeed);
    Timer.delay(0.1);
    spx.set(ControlMode.PercentOutput, 0.0);

    Timer.delay(1.0);
    //Reset Motors
    spx.follow(srx);    
    srx.setNeutralMode(NeutralMode.Brake);
    spx.setNeutralMode(NeutralMode.Brake);

    print("ELEVATOR SRX CURRENT: " + currentSRX + " SPX CURRENT: " + currentSPX);
    print("ELEVATOR SRX ENCODER INIT POS: " + SRXintialEncoderPos + " END POS: " + positionSRX);
    print("ELEVATOR SPX ENCODER INIT POS: " + SRXintialEncoderPos + " END POS: " + positionSPX);

    boolean failure = false;

    print("!%!%#$!%@ - WRITE A TEST FOR THE ELEVATOR LIMIT SWITCHES!!!!!!!!!");

    if (currentSRX < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!!! ELEVATOR SRX Current Low !!!!!!!!!!!!!!!!!");
    }

    if (currentSPX < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!! ELEVATOR SPX Current Low !!!!!!!!!!!!!!!!!!!");
    }

    if (!Util.allCloseTo(Arrays.asList(currentSRX, currentSPX), currentSRX, 5.0)) {
      failure = true;
      print("!!!!!!!!!!!!!!!! ELEVATOR Currents Different !!!!!!!!!!!!!!!!!");
    }

    if (Math.abs(positionSRX - SRXintialEncoderPos) <= 0){
      failure = true;
      print("!!!!!!!!!!!! ELEVATOR Encoder didn't change position SRX !!!!!!!!!!!!!");
    }

    if (Math.abs(positionSPX - SPXintialEncoderPos) <= 0){
      failure = true;
      print("!!!!!!!!!!!! ELEVATOR Encoder didn't change position SPX !!!!!!!!!!!!!");
    }

    return failure;
  }

  public void dashboard() {
    SmartDashboard.putNumber("Elevator/Pos", getPosition());
    SmartDashboard.putBoolean("Elevator/BottomLimitSW", getBottomLimitSW());
    SmartDashboard.putBoolean("Elevator/TopLimitSW", getTopLimitSW());
    SmartDashboard.putNumber("Elevator/SRXout", srx.getMotorOutputPercent());
    //SmartDashboard.putNumber("Elevator/SPXout", spx.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator/SRXcurrent", srx.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/Veloctiy", srx.getSelectedSensorVelocity());
  }
}
