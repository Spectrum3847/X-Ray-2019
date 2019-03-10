package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.lib.drivers.Photon;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.commands.RumbleOff;

public class Disabled {
    
    public static void init() {
        Command a = new RumbleOff();
        a.start();
        Scheduler.getInstance().removeAll();
        a.close();
    }

    //Periodic method called roughly once every 20ms
    public static void periodic() {
        Scheduler.getInstance().run();
        if (HW.oi.isDriverButtonPushed()){
            Robot.photon.addAnimation("DriverButton", Animation.CYLON, Photon.Color.GREEN, Color.WHITE, 50, 5);
        }

        if (HW.oi.isOperatorButtonPushed()){
            Robot.photon.addAnimation("OperatorButton", Animation.CYLON, Color.RED, Color.WHITE, 51, 5);
        }
    }
}
