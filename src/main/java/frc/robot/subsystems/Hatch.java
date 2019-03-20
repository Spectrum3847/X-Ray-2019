package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.Timer;
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
    public SpectrumSolenoid hatchEjectSol;
    public SpectrumSolenoid hatchHoldSol;
    public CANDigitalInput hatchSW;

    public Hatch(){
        hatchEjectSol = new SpectrumSolenoid(HW.HATCH_EJECT_SOL);
        hatchHoldSol = new SpectrumSolenoid(HW.HATCH_HOLD_SOL);
        if (Robot.cargoMech != null){
            hatchSW = new CANDigitalInput(Robot.cargoMech.cargoTopMAX, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
        }
    }

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
    }
    
    public boolean getHatchSW(){
        return hatchSW.get();
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
        //SmartDashboard.putBoolean("Hatch/Holding", holding);
        //SmartDashboard.putBoolean("Hatch/Ejecting", ejecting);
        SmartDashboard.putBoolean("Hatch SW", getHatchSW());
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

        print("Testing HATCH MECH.--------------------------------------------------");
        print("HATCH RELEASE--------------------------------------------------");
        
        boolean failure = false;
        
        Timer.delay(2);
        this.hatchRelease();
        Timer.delay(1);
		print("PRESS HATCH BUTTON within 10 secs");
		double startTimeR = Timer.getFPGATimestamp();
		boolean buttonPressed = false;
		while(Timer.getFPGATimestamp() - startTimeR < 10){
			if(this.getHatchSW()){
				buttonPressed = true;
				print("Thank You");
				break;
			}
        }
        Timer.delay(2);

		if (!buttonPressed) {
			failure = true;
			print("!!!!!!!!!!!!!!!! CARGO BUTTON RIGHT NOT PRESSED !!!!!!!!!!!!!!!!!!!");
		}

        print("WRITE TEST FOR HATCH PNEUMATICS");
        this.hatchHold();
		return failure;	
	}
}
