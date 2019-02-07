/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.drivers.LeaderTalonSRX;
import frc.lib.drivers.SpectrumVictorSPX;
import frc.robot.HW;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

	public final static int downPositionLimit = 25000; //Needs to be determined manually

  SpectrumVictorSPX spx = new SpectrumVictorSPX(HW.CLIMBER_SPX);
  LeaderTalonSRX srx = new LeaderTalonSRX(HW.CLIMBER_SRX, spx);

  private int targetPosition = 0;
  private int accel = 0;
	private int cruiseVel = 0;

  public Climber() {
    super("Climber");
		boolean climberInvert = true;
    boolean climberPhase = true;
    srx.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //Modify to run a system check for the robot
  public boolean checkSystem() {
    return true;
  }

  public void dashboard() {
  }
}
