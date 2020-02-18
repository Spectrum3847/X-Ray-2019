package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.Robot;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ExampleSubsystem extends Subsystem {
  //Put the HW declarations here

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void dashboard() {
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._general, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._general, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._general, Debugger.warning4);
  }

  public static void print(String msg){
        System.out.println(msg);
  }

  public void logEvent(String event){
    printDebug(event);
    SpectrumLogger.getInstance().addEvent(Robot._general, event);
  }

  //Modify to run a system check for the robot
  public boolean checkSystem() {
    return true;
  }
}
