package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.cargo.IntakeUp;
import frc.robot.subsystems.Elevator;

public class ElevatorCargoIntakeControl extends Command {
  /**
   * Add your docs here.
   */
  public ElevatorCargoIntakeControl() {

  }
  protected void intialize(){
    Robot.cargoMech.logEvent("Waiting to raise elevator for intaking");
    Robot.printDebug("Waiting to raise elevator for intake");
  }
  protected boolean isFinished() {
    if (Robot.cargoMech.cargoTopSRX.getOutputCurrent() > Robot.prefs.getNumber("C: InAmpsThreshold", 29)){
      new MotionMagicElevator((int)Robot.prefs.getNumber("C: ElevatorHeight", 2000)).start();
      
      return true;
    }else{
      return false;
    }
  }

  protected void end(){
  }

  protected void interrupted(){
  }
}
