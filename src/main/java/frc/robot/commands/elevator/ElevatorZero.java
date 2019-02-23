package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Zero the Extension
 */
public class ElevatorZero extends Command {
	double s;
	boolean alreadyDown = false;
	public ElevatorZero() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.softLimitsOn(false);
		
		//Get Zero Speed from the prefrences
		s = -1 * Robot.prefs.getNumber("E: Zero Speed", 0.4);

		if (Robot.elevator.getBottomLimitSW()){
			alreadyDown = Robot.elevator.getBottomLimitSW();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.elevator.setOpenLoop(s);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.elevator.getBottomLimitSW();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		//We actually reached our bottom limit switch, set to zero
		if (Robot.elevator.getBottomLimitSW()){
			Robot.elevator.zeroPosition();
			Robot.elevator.setTargetPosition(0);
		}
		Robot.elevator.setOpenLoop(0.0);
		Robot.elevator.softLimitsOn(true);
		if (!alreadyDown){
			Robot.elevator.logEvent("Elevator Zero");
			Robot.elevator.setMotionMagicParams();
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
