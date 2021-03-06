package frc.robot;

import edu.wpi.first.wpilibj.command.Scheduler;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.commands.elevator.MMElevator;

public class Autonomous {

	public static void init() {
        Scheduler.getInstance().removeAll();
		Scheduler.getInstance().enable();
		new MMElevator().start(); //Force a new hold position at the begining of telop, should keep it at the right position.
        Robot.visionLL.initialize();
		Debugger.println("Auto Init");
		SpectrumLogger.getInstance().addEvent("System", "Auto Init Complete");
	}

	// Periodic method called roughly once every 20ms
	public static void periodic() {
		Scheduler.getInstance().run();
	}

	public static void cancel() {
		Scheduler.getInstance().removeAll();
		SpectrumLogger.getInstance().addEvent("System", "Auto Complete");
		Robot.pneumatics.compressor.start();
	}

    public static void printDebug(String msg){
    	Debugger.println(msg, Robot._auton, Debugger.debug2);
    }
    
    public static void printInfo(String msg){
    	Debugger.println(msg, Robot._auton, Debugger.info3);
    }
    
    public static void printWarning(String msg) {
    	Debugger.println(msg, Robot._auton, Debugger.warning4);
    }
}
