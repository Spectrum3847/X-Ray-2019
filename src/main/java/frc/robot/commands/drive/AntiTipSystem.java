/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.Robot;
//import frc.robot.commands.elevator.CheckIfZero;
//import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.elevator.MMElevator;
//import frc.robot.commands.elevator.SimpleElevatorGoToPos;

public class AntiTipSystem extends Command {
  private double afsAngle; 
  private double afsTipTime;
  private double afsTipThreshold;
  private boolean afsActive; //Checks SmartDashboard for input
  private double afsPitch;
  private boolean tipped;
  private double goHome;
  private double[] yawPitchRoll = null;
  

  
  public AntiTipSystem() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
      //set AFS angle threshhold
      afsPitch=0;
      afsAngle = Robot.prefs.getNumber("Angle before Tip", 5);
      afsTipTime = 0;
      afsTipThreshold = Robot.prefs.getNumber("TipThreshold in sec",0.1);
      afsActive = Robot.prefs.getBoolean("AntiFlipSystem", true);
      tipped = SmartDashboard.getBoolean("Tipped",false);
      goHome = 0;
      yawPitchRoll = new double[3];
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.pigeon.getYawPitchRoll(yawPitchRoll);

    if(afsActive){

      SmartDashboard.putBoolean("Tipped",tipped);
      goHome -= 0.02;
      afsPitch = yawPitchRoll[1];
      if(Robot.drive.pigeon != null){
        if(Math.abs(afsPitch) >= afsAngle){
          afsTipTime += 0.02;
          new CoastMode().start();
        }
        if(Math.abs(afsPitch) < afsAngle ){ 
          afsTipTime = 0;
          tipped = false;
          new BrakeMode().start();
        }
      }
      if(afsTipTime >= afsTipThreshold && goHome <=0){
        tipped = true;
        new MMElevator(0).start(); 
        goHome = 1;
        Robot.photon.setAnimation(Animation.PERCENTAGE, Color.RED, Color.ELEVATORPOS);
      }

    }
  }
  



  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    tipped = false;
    SmartDashboard.putBoolean("Tipped",tipped);
    afsTipTime=0;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
