package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.controllers.SpectrumXboxController;
import frc.robot.OI;

/**
 * An example command.  You can replace me with your own command.
 */
public class RumbleController extends Command {
  private SpectrumXboxController controller;
  private double value;

  public RumbleController(SpectrumXboxController controller, double value) {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.m_subsystem);\
    this.controller = controller;
    this.value = value;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    controller.setRumble(value, value);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  protected void end() {
    controller.setRumble(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
