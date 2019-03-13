package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SpectrumDigitalInput;
import frc.lib.drivers.SpectrumSolenoid;
import frc.lib.drivers.SpectrumSparkMax;
import frc.lib.drivers.SpectrumTalonSRX;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.HW;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * CargoMech Subsystem
 */
public class CargoMech extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	//Cargo Front is the stationary roller on the bottom
	//Cargo Rear is the moving roller on the top
	public SpectrumTalonSRX cargoBottomSRX = new SpectrumTalonSRX(HW.CARGO_BOTTOM);
	//public SpectrumTalonSRX cargoTopSRX = new SpectrumTalonSRX(HW.CARGO_TOP);
	public SpectrumSparkMax cargoTopMAX = new SpectrumSparkMax(50);
	
	public SpectrumSolenoid intakeSol = new SpectrumSolenoid(HW.CARGO_INTAKE_SOL);
	public SpectrumSolenoid tiltSol = new SpectrumSolenoid(HW.CARGO_TILT_SOL);

	public SpectrumDigitalInput CargoSW = new SpectrumDigitalInput(0);
	
	public static double thresholdStart;
	private static boolean intakeComplete;
	private static boolean tiltUp;
	private static boolean intakeDown;
	 public CargoMech() {
		//cargoTopSRX.setInverted(true);
		cargoTopMAX.setInverted(true);
		cargoBottomSRX.setInverted(true);
		//cargoTopSRX.configVoltageCompSaturation(12.0);
		//cargoTopSRX.enableVoltageCompensation(false);
		cargoTopMAX.disableVoltageCompensation();
		cargoTopMAX.setSecondaryCurrentLimit(40);
		cargoTopMAX.setSmartCurrentLimit(32,32,2000);
		cargoTopMAX.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
		cargoTopMAX.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
		cargoTopMAX.setIdleMode(IdleMode.kBrake);
		cargoBottomSRX.configVoltageCompSaturation(11.5);
		cargoBottomSRX.enableVoltageCompensation(true);
	}

	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new ManualCargo());
	}
	
	public void periodic() {
		 //Rumble operator controller when intake is on
		    //if (cargoTopSRX.getMotorOutputPercent() != 0) {
		    if (cargoTopMAX.getAppliedOutput() != 0) {
		    	printDebug("Rumbling Operator because Intake On");
		    	OI.opController.setRumble(.6, .6);
		    } else {
		    	OI.opController.setRumble(0, 0);
		    }
	 }
	
	public boolean getIntakeSW(){
		return !CargoSW.get();
	}
	
	public void setTop(double value) {
		//cargoTopSRX.set(ControlMode.PercentOutput, value);
		cargoTopMAX.set(value);
		//printDebug("Cargo Top Set To" + cargoTopSRX.getMotorOutputPercent());
	}

	public void setBottom(double value){
		cargoBottomSRX.set(ControlMode.PercentOutput, value);
		printDebug("Cargo Bottom Set To" + cargoBottomSRX.getMotorOutputPercent());
	}
	
	public void stop() {
		setTop(0);
		setBottom(0);
	}
	
	//extends(opens) the intake
	public void intakeDown() {
		intakeSol.set(true);
		intakeDown = true;
		printDebug("Intake Down");
	}
	
	//retracts(closes) the intake
	public void intakeUp() {
		intakeSol.set(false);
		intakeDown = false;
		printDebug("Intake Up");
	}

	//extends the tilt cylinders up
	public void tiltUp(){
		tiltSol.set(false);
		tiltUp = true;
		printDebug("Tilt Up");
	}

	//retracts the tilt cylidners down
	public void tiltDown(){
		tiltSol.set(true);
		tiltUp = false;
		printDebug("Tilt Down");
	}
	
	//returns the current from one of the SRXs 
	public double getCurrent() {
		return (cargoBottomSRX.getOutputCurrent() + cargoTopMAX.getOutputCurrent());//cargoTopSRX.getOutputCurrent());
	}
	
	//check if the intake is complete, right now just the intakeSW but might need more logic in the future
	public boolean isIntakeComplete() {
		intakeComplete = getIntakeSW() && !OI.opController.Dpad.Right.get();
		return intakeComplete;
	}
	
	//Add the dashboard values for this subsystem
	public void dashboard() {
		SmartDashboard.putNumber("Cargo/Bottom Output", cargoBottomSRX.getMotorOutputPercent());
		SmartDashboard.putNumber("Cargo/Top Output", cargoTopMAX.getAppliedOutput());//cargoTopSRX.getMotorOutputPercent());
		SmartDashboard.putNumber("Cargo/Bottom Current", cargoBottomSRX.getOutputCurrent());
		SmartDashboard.putNumber("Cargo/Top Current", cargoTopMAX.getOutputCurrent());//cargoTopSRX.getOutputCurrent());
		SmartDashboard.putNumber("Cargo/Top In Voltage", cargoTopMAX.getBusVoltage());
		SmartDashboard.putNumber("Cargo/Top Temp", cargoTopMAX.getMotorTemperature());
		SmartDashboard.putNumber("Cargo/Top PDP Current", HW.PDP.getCurrent(10));
		SmartDashboard.putNumber("Cargo/Top RPM", cargoTopMAX.getEncoder().getVelocity());
		//SmartDashboard.putBoolean("Cargo/Mech On?", cargoTopSRX.getMotorOutputPercent() != 0 && 
													//cargoBottomSRX.getMotorOutputPercent() != 0);
		SmartDashboard.putBoolean("Cargo/TiltUp", tiltUp);
		SmartDashboard.putBoolean("Cargo/IntakeDown", intakeDown);
		SmartDashboard.putBoolean("Cargo/Sensor SW", getIntakeSW());
	}
	
    public void printDebug(String msg){
    	Debugger.println(msg, Robot._cargo, Debugger.debug2);
    }
    
    public void printInfo(String msg){
    	Debugger.println(msg, Robot._cargo, Debugger.info3);
    }
    
    public void printWarning(String msg) {
    	Debugger.println(msg, Robot._cargo, Debugger.warning4);
	}
	
	public void print(String msg){
		System.out.println(msg);
	}

	public void logEvent(String event){
		SpectrumLogger.getInstance().addEvent(Robot._cargo, event);
	}
	
	/*Modify this method to return false if there is a problem with the subsystem
	  Based on 254-2017 Code
	*/
	public boolean checkSystem() {
		print("Testing CARGO MECH.--------------------------------------------------");
		final double kCurrentThres = 2;

		//Zero Both Motors
		cargoTopMAX.set(0.0);
		cargoBottomSRX.set(ControlMode.PercentOutput, 0.0);

		// test cargoMech Top Roller
		cargoTopMAX.set(1);
		Timer.delay(1.0);
		final double currentTop = cargoTopMAX.getOutputCurrent();;
		cargoTopMAX.set(0);
		Timer.delay(0.25);

		// test cargoMech Bottom Roller
		cargoBottomSRX.set(ControlMode.PercentOutput, 1.0);
		Timer.delay(1.0);
		final double currentBottom = cargoBottomSRX.getOutputCurrent();
		cargoBottomSRX.set(ControlMode.PercentOutput, 0.0);
		Timer.delay(0.25);

		print("CARGO TOP CURRENT: " + currentTop + " BOTTOM CURRENT: " + currentBottom);

		boolean failure = false;

		if (currentTop < kCurrentThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!!! CARGO TOP Current Low !!!!!!!!!!!!!!!!!");
		}

		if (currentBottom < kCurrentThres) {
		failure = true;
		print("!!!!!!!!!!!!!!!! CARGO BOTTOM Current Low !!!!!!!!!!!!!!!!!!!");
		}

		print("PRESS CARGO BUTTON LEFT within 10 secs");
		double startTime = Timer.getFPGATimestamp();
		boolean leftButtonPressed = false;
		while(Timer.getFPGATimestamp() - startTime < 10){
			if(this.getIntakeSW()){
				leftButtonPressed = true;
				print("Thank You");
				break;
			}
		}

		if (!leftButtonPressed) {
			failure = true;
			print("!!!!!!!!!!!!!!!! CARGO BUTTON LEFT NOT PRESSED !!!!!!!!!!!!!!!!!!!");
		}

		print("PRESS CARGO BUTTON RIGHT within 10 secs");
		double startTimeR = Timer.getFPGATimestamp();
		boolean rightButtonPressed = false;
		while(Timer.getFPGATimestamp() - startTimeR < 10){
			if(this.getIntakeSW()){
				rightButtonPressed = true;
				print("Thank You");
				break;
			}
		}

		if (!rightButtonPressed) {
			failure = true;
			print("!!!!!!!!!!!!!!!! CARGO BUTTON RIGHT NOT PRESSED !!!!!!!!!!!!!!!!!!!");
		}

		print("WRITE TEST FOR CARGO PNEUMATICS");

		return failure;
		/*
		//Example checkSystem from 254's 2017 Robot
		System.out.println("Testing INTAKE.--------------------------------------");
        final double currentThres = 0.5;
        
        intakeSRX.set(0.0);

        intakeSRX.set(ControlMode.PercentOutput,-6.0);
        Timer.delay(4.0);
        final double currentLeader = intakeSRX.getOutputCurrent();
        intakeSRX.set(0.0);

        Timer.delay(2.0);

        intakeBottomSRX.set(ControlMode.PercentOutput,6.0);
        Timer.delay(4.0);
        final double currentFollower = intakeBottomSRX.getOutputCurrent();
        intakeBottomSRX.set(0.0);

        System.out.println("Hopper Leader Current: " + currentLeader + " Slave current: " + currentFollower);

        boolean failure = false;

        if (currentLeader < currentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!! Hopper Master Current Low !!!!!!!!!!!!!!!!!");
        }

        if (currentFollower < currentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!! Hooper Slave Current Low !!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentMaster, currentSlave), currentMaster, 5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!! Hopper Currents Different !!!!!!!!!!!!!!!!!");
        }

        return !failure;
        */
	}
}
