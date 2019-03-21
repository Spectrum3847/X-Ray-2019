/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * Add your docs here.
 */
public class PathFollower extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private static final int k_ticks_per_rev = 1;
  private static final double k_wheel_diameter = 0.333333333333;
  private static final double k_max_velocity = 15;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;

  private Trajectory leftTraj, rightTraj;

  private Notifier m_follower_notifier;
  private boolean reverse = false;

  public PathFollower(){
    m_left_follower = new EncoderFollower();
    m_right_follower = new EncoderFollower();

    m_left_follower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_right_follower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_follower_notifier = new Notifier(this::followPath);
  }

  //Enter How you reset your left and right drive encoders here.
  public void resetEncoders(){
    Robot.drive.resetEncoders();
  }

  //Insert how you get the position of your left encoder here
  public int getLeftEncoder(){
    return (int)Robot.drive.leftEncoder.getPosition();
  }

  //Insert how you get the position of your right encoder here
  public int getRightEncoder(){
    return (int)Robot.drive.rightEncoder.getPosition();
  }

  //Insert method for how you set your drive speeds here.
  public void setSpeeds(double left, double right){
    Robot.drive.tankDrive(left, right);
  }

  //Insert method to get your robot heading
  public double getHeading(){
    return Robot.drive.pigeon.getFusedHeading();
  }

  //Call in each command to set your trajectories
  public void setTrajectories(Trajectory leftTraj, Trajectory rightTraj, boolean reverse){
    this.leftTraj = leftTraj;
    this.rightTraj = rightTraj;
    m_left_follower.setTrajectory(leftTraj);
    m_right_follower.setTrajectory(rightTraj);
    this.reverse = reverse;
  }

 
  public void configurePIDVA(double kp, double ki, double kd, double kv, double ka){
    m_right_follower.configurePIDVA(kp, ki, kd, kv, ka);
    m_left_follower.configurePIDVA(kp, ki, kd, kv, ka);
  }

  public void updatePIDVAprefs(){
    double kp = Robot.prefs.getNumber("PathW: P", 0.7);
    double ki = Robot.prefs.getNumber("PathW: I", 0.0);
    double kd = Robot.prefs.getNumber("PathW: D", 0.0);
    double kv = 1 / Robot.prefs.getNumber("PathW: Velocity", 15);
    double ka = Robot.prefs.getNumber("PathW: Accel", 0.0);
    configurePIDVA(kp, ki, kd, kv, ka);
  }

  public void startPathFollowing(){
    m_follower_notifier.startPeriodic(leftTraj.get(0).dt);
  }

  public void stopPathFollowing(){
    m_follower_notifier.stop();
  }

  public boolean isFinished(){
    return m_left_follower.isFinished() || m_right_follower.isFinished();
  }

  private void followPath() {
    if (isFinished()) {
      m_follower_notifier.stop();
    } else {
      int leftPos = getLeftEncoder();
      int rightPos = getRightEncoder();
      print("Right P: " + rightPos + " Left P: " + leftPos);
      double left_speed = m_left_follower.calculate(leftPos);
      double right_speed = m_right_follower.calculate(rightPos);
      //double heading = getHeading();
      //double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      //double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      //double turn =  0.8 * (-1.0/80.0) * heading_difference;
      double turn = 0;
      if(reverse){
        //Flip left and right and multiply by negative one.
        setSpeeds(-1 * (right_speed - turn) , -1 * (left_speed + turn));
      } else{ 
        setSpeeds(left_speed + turn, right_speed - turn);
      }
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._controls, Debugger.debug2);
  }

  public static void printInfo(String msg){
    Debugger.println(msg, Robot._controls, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._controls, Debugger.warning4);
  }

  public static void print(String msg){
    System.out.println(msg);
  }

  public void logEvent(String event){
      printDebug(event);
      SpectrumLogger.getInstance().addEvent(Robot._controls, event);
  }


}
