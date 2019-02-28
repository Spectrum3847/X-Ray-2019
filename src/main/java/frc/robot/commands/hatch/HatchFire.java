package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.cargo.*;

public class HatchFire extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HatchFire() {
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
    addParallel(new TiltDown());
    addSequential(new WaitCommand(.03));
    addParallel(new HatchRelease());
    addSequential(new WaitCommand(.03));
    addParallel(new HatchEject());
  }

  public void initialize(){
    Robot.hatch.logEvent("HATCH READY");
    //this.setTimeout(2);
  }
}