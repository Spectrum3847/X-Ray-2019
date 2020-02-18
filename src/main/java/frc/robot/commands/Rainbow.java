/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.Photon.Animation;
import frc.lib.drivers.Photon.Color;
import frc.robot.Robot;


public class Rainbow extends Command {
  
  Boolean activateRainbow;
  Boolean active;
    
  double matchTime;

  public Rainbow() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.photon);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    activateRainbow = Robot.prefs.getBoolean("Rainbow", true);
    active = SmartDashboard.getBoolean("Custom Colors", true); 
    
    Robot.photon.addAnimation("Custom Colors", Animation.CYLON,Color.PURPLE, Color.WHITE, 80, 2.0);



  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      
    SmartDashboard.putBoolean("Custom Colors", active);
        
    matchTime = Robot.DS.getMatchTime();
        
    if(matchTime <= 30)
    {
      active = true;
    Robot.photon.setAnimation(Animation.RAINBOW);
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
      end();
    }
  }
