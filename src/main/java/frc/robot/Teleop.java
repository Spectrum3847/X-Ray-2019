package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.elevator.MotionMagicElevator;


/**
 * The Driver Control period of the competition
 */
public class Teleop {
	
    public static void init() {
        Scheduler.getInstance().removeAll();
        new MotionMagicElevator().start(); //Force a new hold position at the begining of telop, should keep it at the right position.
        Robot.visionLL.initialize();

        //Do things if connected to FMS only, so when telop starts during a match but not during testing
        if (DriverStation.getInstance().isFMSAttached()){
        }
    }

    public static void periodic() {
        Scheduler.getInstance().run();

    }

    public static void cancel() {
        Scheduler.getInstance().removeAll();
    }
}
