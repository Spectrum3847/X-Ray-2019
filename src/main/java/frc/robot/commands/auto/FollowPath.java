/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.io.IOException;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.util.Debugger;
import frc.robot.Robot;
import frc.robot.subsystems.PathFollower;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

public class FollowPath extends Command {
  Trajectory leftTraj, rightTraj;
  boolean reverse;

  public FollowPath(String leftTraj, String rightTraj, boolean reverse){
    requires(Robot.drive);
    requires(Robot.pathFollower);
    try{
      this.leftTraj = PathfinderFRC.getTrajectory(leftTraj);
      this.rightTraj = PathfinderFRC.getTrajectory(rightTraj);
    } catch (Exception e){
      Debugger.println("Failed to Load Paths: " + leftTraj + " : " + rightTraj, Robot._auton, Debugger.fatal6);
      leftTraj = null;
      rightTraj = null;
    }
    this.reverse = reverse;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Update PIDVA prefrences and reset encoders to zero and reset pigeon to zero
    Robot.drive.resetEncoders();
    Robot.pathFollower.updatePIDVAprefs();
    Robot.pathFollower.setTrajectories(leftTraj, rightTraj, reverse);
    PathFollower.printDebug("Follow Path Started");
    Robot.pathFollower.startPathFollowing();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.pathFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.pathFollower.stopPathFollowing();
    Robot.pathFollower.printDebug("Follow Path Ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
