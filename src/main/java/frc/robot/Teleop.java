package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.PercentLight;
import frc.robot.commands.Rainbow;
import frc.robot.commands.drive.AntiTipSystem;
import frc.robot.commands.drive.BrakeMode;
import frc.robot.commands.elevator.MMElevator;


/**
 * The Driver Control period of the competition
 */
public class Teleop {
	
    public static void init() {
        Scheduler.getInstance().removeAll();
        new MMElevator().start(); //Force a new hold position at the begining of telop, should keep it at the right position.
        Robot.visionLL.initialize();
        new AntiTipSystem().start();
        new PercentLight().start();
        new Rainbow().start();
        
        

        //Do things if connected to FMS only, so when telop starts during a match but not during testing
        if (DriverStation.getInstance().isFMSAttached()){
        }
        new BrakeMode().start();
    }

    public static void periodic() {
        Scheduler.getInstance().run();
    }

    public static void cancel() {
        Scheduler.getInstance().removeAll();
    }
}
