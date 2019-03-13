package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.cargo.TiltDown;
import frc.robot.commands.cargo.TiltUp;
import frc.robot.commands.drive.AutoTurn;
import frc.robot.commands.drive.DriveSpeed;
import frc.robot.commands.drive.LLDrive;
import frc.robot.commands.elevator.ElevatorCargoIntakeControl;
import frc.robot.commands.elevator.MMElevator;
import frc.robot.commands.hatch.HatchFire;
import frc.robot.commands.hatch.HatchHold;
import frc.robot.commands.hatch.HatchReady;
import frc.robot.commands.hatch.HatchRelease;
import frc.robot.subsystems.Elevator;

public class LvlOneTwoRocketRight extends CommandGroup {
  /**
   * Add your docs here.
   */

  public LvlOneTwoRocketRight() {
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
    addSequential(new LLDrive());
    addParallel(new HatchFire(),1);
    addSequential(new DriveSpeed(-.3),.3);
    addSequential(new TimedCommand(.25));
    addSequential(new AutoTurn(-108));
    addParallel(new HatchReady());
    addSequential(new DriveSpeed(.4), 1.6);
    addSequential(new LLDrive());
    addSequential(new TimedCommand(.4));
    addParallel(new HatchHold());
    addParallel(new TiltUp());
    addSequential(new DriveSpeed(-.5),3.5);
    //addSequential(new AutoTurn(-01));
    //addSequential(new DriveSpeed(-.3),3.5);
    addSequential(new AutoTurn(30));
    addParallel(new TiltDown());
    addSequential(new LLDrive());
    addParallel(new HatchFire(),1);
  }
  protected void intialize(){

  }

  protected boolean isFinished() {
    return false;
  }


  protected void end(){
  }
}
