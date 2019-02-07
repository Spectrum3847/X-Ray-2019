package frc.robot;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Scheduler;
import frc.lib.util.Debugger;

public class Autonomous {

	public static void init() {
        Scheduler.getInstance().removeAll();
        Scheduler.getInstance().enable();
		Debugger.println("Auto Init");
		HelixEvents.getInstance().addEvent("System", "Auto Init Complete");
	}

	// Periodic method called roughly once every 20ms
	public static void periodic() {
		Scheduler.getInstance().run();
	}

	public static void cancel() {
		Scheduler.getInstance().removeAll();
		HelixEvents.getInstance().addEvent("System", "Auto Complete");
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
