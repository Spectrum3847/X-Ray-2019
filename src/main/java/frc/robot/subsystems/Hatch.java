package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SpectrumSolenoid;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.HW;
import frc.robot.Robot;

public class Hatch extends Subsystem {

    public boolean  holding = true;
    public boolean ejecting = false;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
    public SpectrumSolenoid hatchEjectSol = new SpectrumSolenoid(HW.HATCH_EJECT_SOL);
    public SpectrumSolenoid hatchHoldSol = new SpectrumSolenoid(HW.HATCH_HOLD_SOL);

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
    
    //Extend the eject cylinders
	public void hatchEject() {
        hatchEjectSol.set(true);
        ejecting = true;
	}
	
	public void hatchRetract() {
        hatchEjectSol.set(false);
        ejecting = false;
    }
    
    //Hold the hatch in place, this is false because it's the default state
    public void hatchHold(){
        hatchHoldSol.set(false);
        holding = true;
    }

    public void hatchRelease(){
        hatchHoldSol.set(true);
        holding = false;
    }

	//Add the dashboard values for this subsystem
	public void dashboard() {
        SmartDashboard.putBoolean("Hatch/Holding", holding);
        SmartDashboard.putBoolean("Hatch/Ejecting", ejecting);
    }

    public static void printDebug(String msg){
        Debugger.println(msg, Robot._hatch, Debugger.debug2);
      }
      
      public static void printInfo(String msg){
        Debugger.println(msg, Robot._hatch, Debugger.info3);
      }
      
      public static void printWarning(String msg) {
        Debugger.println(msg, Robot._hatch, Debugger.warning4);
      }
    
      public static void print(String msg){
            System.out.println(msg);
      }
    
    public void logEvent(String event){
        printDebug(event);
		SpectrumLogger.getInstance().addEvent(Robot._hatch, event);
	}
    
	/*Modify this method to return false if there is a problem with the subsystem
	  Based on 254-2017 Code
	*/
	public boolean checkSystem() {
		return true;
		
		/** Example checkSystem from 254's 2017 Robot
		 System.out.println("Testing HOPPER.--------------------------------------");
        final double kCurrentThres = 0.5;

        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);

        mMasterTalon.set(0.0);
        mSlaveTalon.set(0.0);

        mMasterTalon.set(-6.0f);
        Timer.delay(4.0);
        final double currentMaster = mMasterTalon.getOutputCurrent();
        mMasterTalon.set(0.0);

        Timer.delay(2.0);

        mSlaveTalon.set(6.0f);
        Timer.delay(4.0);
        final double currentSlave = mSlaveTalon.getOutputCurrent();
        mSlaveTalon.set(0.0);

        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        System.out.println("Hopper Master Current: " + currentMaster + " Slave current: " + currentSlave);

        boolean failure = false;

        if (currentMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!! Hopper Master Current Low !!!!!!!!!!!!!!!!!");
        }

        if (currentSlave < kCurrentThres) {
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
