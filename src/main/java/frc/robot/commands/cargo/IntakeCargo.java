package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class IntakeCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IntakeCargo() {
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
    addParallel(new TiltDown());
    addParallel(new RollerBottomOn(1.0));
    addParallel(new RollerTopOn(1.0));
  }
  protected boolean isFinished() {
    return Robot.cargoMech.isIntakeComplete();
  }
}
