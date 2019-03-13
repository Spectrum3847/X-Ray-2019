package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.RumbleController;
import frc.robot.commands.cargo.TiltUp;
import frc.robot.commands.elevator.MMElevator;
import frc.robot.commands.hatch.HatchHold;
import frc.robot.commands.hatch.HatchReady;

public class AutoHatchIntake extends CommandGroup {
    boolean hasTarget = false;

    public AutoHatchIntake() {
        this.addParallel(new MMElevator(Robot.elevator.posDownLimit));
        this.addParallel(new HatchReady());
        this.addSequential(new LLDrive());
        this.addParallel(new RumbleController(OI.driverController, .75));
        this.addSequential(new TimedCommand(.2));
        this.addParallel(new HatchHold());
        this.addSequential(new TimedCommand(.15));
        this.addParallel(new TiltUp());
        this.addSequential(new TimedCommand(.15));
        this.addSequential(new DriveSpeed(-.5),.5);
        this.setInterruptible(false);
    }

    @Override
    protected void initialize() {
        if (Robot.visionLL.getLimelightHasValidTarget()){
            hasTarget = true;
        } else{
            hasTarget = false;
        }
    }

    protected void execute(){
        Robot.photon.addAnimation("AutoLoad", Animation.SIREN, Color.YELLOW, Color.WHITE, 101, 1);
    }

    protected boolean isFinished(){
        return !hasTarget || super.isFinished();
    }
}
