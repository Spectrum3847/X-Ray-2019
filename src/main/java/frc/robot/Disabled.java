package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Disabled {
    
    public static void init() {
        Scheduler.getInstance().removeAll();
    }

    //Periodic method called roughly once every 20ms
    public static void periodic() {
        Dashboard.updateDashboard();
        Timer.delay(0.001);
    }
}
