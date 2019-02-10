package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Zero the Extension
 */
public class ElevatorZero extends Command {
	public ElevatorZero() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.logEvent("Elevator Zero");
		Robot.elevator.softLimitsOn(false);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.elevator.setOpenLoop(-.1);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.elevator.getBottomLimitSW();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.elevator.setOpenLoop(0);
		Robot.elevator.zeroPosition();
		Robot.elevator.softLimitsOn(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
