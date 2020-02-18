package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LeaderTalonSRX;
import frc.lib.drivers.SpectrumDigitalInput;
import frc.lib.drivers.SpectrumSolenoid;
import frc.lib.drivers.SpectrumTalonSRX;
import frc.lib.drivers.SpectrumVictorSPX;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.lib.util.Util;
import frc.robot.HW;
import frc.robot.Robot;
import frc.robot.commands.Climb;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final static boolean invert = false; //Invert the motor here FWD should be climbing, negative retract
	public final static int downPositionLimit = 25000; //Needs to be determined manually

  final SpectrumVictorSPX spx;
  final SpectrumTalonSRX srx;
  final SpectrumTalonSRX srx_slave;
  final SpectrumSolenoid ratchet;
  final SpectrumDigitalInput sensor;

  // private int targetPosition = 0;
  // private int accel = 0;
	// private int cruiseVel = 0;

  public Climber() {
    super("Climber");
    spx = new SpectrumVictorSPX(HW.VACUUM_SPX);
    srx_slave = new SpectrumTalonSRX(HW.CLIMBER_SLAVE);
    srx = new LeaderTalonSRX(HW.CLIMBER_SRX, srx_slave);
    ratchet = new SpectrumSolenoid(HW.CLIMBER_SOL);
    sensor = new SpectrumDigitalInput(HW.CLIMBER_SENSOR);
		// boolean climberInvert = true;
    // boolean climberPhase = true;
    ratchet.set(false);
    srx.setNeutralMode(NeutralMode.Brake);
    srx.setInverted(invert);
    srx.configContinuousCurrentLimit(45);
    srx.configPeakCurrentLimit(0);
    srx.configPeakCurrentDuration(0);
    srx.enableCurrentLimit(true);
    srx.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

  public boolean getLimit(){
    return !sensor.get();
  }

  public double getCurrent(){
    return srx.getOutputCurrent(); 
  }

  public double getEncoderPosition(){
    return srx.getSensorCollection().getQuadraturePosition();
  }

  public void zeroSensor(){
    srx.setSelectedSensorPosition(0);
  }

  public void setClimbMotor(double speed){
    if (getLimit() && speed >= 0){
      srx.set(ControlMode.PercentOutput, 0);
      printInfo("CLIMBER LIMIT HIT");
    } else {
      srx.set(ControlMode.PercentOutput, speed);
    }
  }

  public void vacuumOn(){
    spx.set(1);
  }

  public void vaccumOff(){
    spx.set(0);
  }

  public boolean getVaccumOn(){
    if (spx.get() > 0){
      return true;
    } else {
      return false;
    }
  }

  public void setRatchet(boolean b){
    ratchet.set(b);
  }


  public void dashboard() {
    SmartDashboard.putNumber("Climber/SRXoutput", srx.getMotorOutputPercent());
    SmartDashboard.putNumber("Climber/SRXcurrent", srx.getOutputCurrent());
    SmartDashboard.putNumber("Climber/SPXoutput", spx.getMotorOutputPercent());
    SmartDashboard.putBoolean("Climber/Sensor", getLimit());
    SmartDashboard.putBoolean("Climber/VacuumOn", getVaccumOn());
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._climber, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._climber, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._climber, Debugger.warning4);
  }

  public static void print(String msg){
		System.out.println(msg);
  }
  
  public void logEvent(String event){
    printDebug(event);
		SpectrumLogger.getInstance().addEvent(Robot._climber, event);
	}

  //Modify to run a system check for the robot
  public boolean checkSystem() {
    print("Testing CLIMBER.--------------------------------------------------");
    final double kCurrentThres = 0.5;
    final double intialEncoderPos = srx.getSensorCollection().getQuadraturePosition();

    spx.set(ControlMode.PercentOutput, 0.0);
    srx.set(ControlMode.PercentOutput, 0.0);
    // test climber srx
    srx.set(ControlMode.PercentOutput, 0.1);
    Timer.delay(1.0);
    final double currentSRX = HW.PDP.getCurrent(HW.CLIMBER_SRX_PDP);
    srx.set(ControlMode.PercentOutput, 0.0);

    Timer.delay(2.0);

    // Test climber spx
    spx.set(ControlMode.PercentOutput, 0.1);
    Timer.delay(1.0);
    final double currentSPX = HW.PDP.getCurrent(HW.CLIMBER_SPX_PDP);
    spx.set(ControlMode.PercentOutput, 0.0);

    // reset spx to follow srx
    spx.follow(srx);
    final double endEncoderPos = srx.getSensorCollection().getQuadraturePosition();

    print("CLIMBER SRX CURRENT: " + currentSRX + " SPX CURRENT: " + currentSPX);
    print("CLIMBER ENCODER INIT POS: " + intialEncoderPos + " END POS: " + endEncoderPos);

    boolean failure = false;

    //WRITE A TEST FOR THE CLIMBER SENSOR
    print("!%!%#$!%@ - WRITE A TEST FOR THE CLIMBER SENSOR!!!!!!!!!");

    if (currentSRX < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!!! Climber SRX Current Low !!!!!!!!!!!!!!!!!");
    }

    if (currentSPX < kCurrentThres) {
      failure = true;
      print("!!!!!!!!!!!!!!!! Climber SPX Current Low !!!!!!!!!!!!!!!!!!!");
    }

    if (!Util.allCloseTo(Arrays.asList(currentSRX, currentSPX), currentSRX, 5.0)) {
      failure = true;
      print("!!!!!!!!!!!!!!!! Climber Currents Different !!!!!!!!!!!!!!!!!");
    }

    if (Math.abs(endEncoderPos - intialEncoderPos) > 0){
      failure = true;
      print("!!!!!!!!!!!! Climber Encoder didn't change position !!!!!!!!!!!!!");
    }

    print("********Please press Climber Limit Switch************");
    double startTime = Timer.getFPGATimestamp();
    boolean sw = false;
    while(Timer.getFPGATimestamp() - startTime < 5 || !srx.getSensorCollection().isFwdLimitSwitchClosed()){
      sw = srx.getSensorCollection().isFwdLimitSwitchClosed();
    }

    if (!sw){
      failure = true;
      print("!!!!!!!!!!!!! Climber Limit Switch Not Pressed !!!!!!!!!!!!!!!!!");
    }

    return !failure;
  }

}
