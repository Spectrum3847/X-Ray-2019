package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.drivers.LeaderTalonSRX;
import frc.lib.drivers.SpectrumVictorSPX;
import frc.robot.HW;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public final static int upPositionLimit = 22500;// needs to be determined manually
	public final static int downPositionLimit = 0;

  SpectrumVictorSPX spx = new SpectrumVictorSPX(HW.ELEVATOR_SPX);
  LeaderTalonSRX srx = new LeaderTalonSRX(HW.ELEVATOR_SRX, spx);
  // private boolean zeroWhenDownLimit = true;

  // // private int targetPosition = 0;
  // // private int accel = 0;
	// // private int cruiseVel = 0;

  public Elevator() {
    super("Elevator");
		//boolean extensionInvert = true;
    //boolean extensionPhase = true;
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
