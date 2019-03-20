package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.elevator.ElevatorCargoIntakeControl;
import frc.robot.commands.elevator.MMElevator;
import frc.robot.subsystems.Elevator;

public class BullDozer extends CommandGroup {
  /**
   * Add your docs here.
   */

   int buttonDebounce;
   Button intakeButton;
  public BullDozer() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    requires(Robot.cargoMech);
    addParallel(new IntakeDown());
    //addParallel(new RollerBottomOn(-1.0));
    //addParallel(new RollerTopOn(-1.0));
  }
  protected void intialize(){
    Robot.cargoMech.logEvent("Intaking Cargo");
    buttonDebounce = -1;
    if (Robot.cargoMech.isIntakeComplete()){
      this.cancel();
    }
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end(){

  }
}