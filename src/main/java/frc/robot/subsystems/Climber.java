package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.drivers.LeaderTalonSRX;
import frc.lib.drivers.SpectrumVictorSPX;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.lib.util.Util;
import frc.robot.HW;
import frc.robot.Robot;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

	public final static int downPositionLimit = 25000; //Needs to be determined manually

  SpectrumVictorSPX spx = new SpectrumVictorSPX(HW.CLIMBER_SPX);
  LeaderTalonSRX srx = new LeaderTalonSRX(HW.CLIMBER_SRX, spx);

  // private int targetPosition = 0;
  // private int accel = 0;
	// private int cruiseVel = 0;

  public Climber() {
    super("Climber");
		// boolean climberInvert = true;
    // boolean climberPhase = true;
    srx.setNeutralMode(NeutralMode.Brake);
    srx.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean getLimit(){
    return srx.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public double getCurrent(){
    return srx.getOutputCurrent() + HW.PDP.getCurrent(HW.CLIMBER_SPX); 
  }

  public double getEncoderPosition(){
    return srx.getSensorCollection().getQuadraturePosition();
  }

  public void zeroSensor(){
    srx.setSelectedSensorPosition(0);
  }

  public void set(double speed){
    srx.set(ControlMode.PercentOutput, speed);
  }

  public void dashboard() {
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
  public void logEvent(String event){
		SpectrumLogger.getInstance().addEvent(Robot._climber, event);
	}
}
