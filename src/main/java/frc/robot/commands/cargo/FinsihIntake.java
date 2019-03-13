package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class FinsihIntake extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FinsihIntake() {
    requires(Robot.cargoMech);
    this.addParallel(new RollerTopOn(1.0),1);
    addParallel(new RollerBottomOn(1.0),1);
  }
  protected void intialize(){
    this.setTimeout(1);
  }

  protected boolean isFinished() {
    return isTimedOut();
  }

  protected void end(){

  }
}
