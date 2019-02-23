package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.drivers.Photon;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class PhotonLEDs extends Subsystem {
  //Put the HW declarations here
	public static Photon photon;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public PhotonLEDs(){
		photon = new Photon(192,255);
		photon.SetNumberOfLEDs(1, 55);
		photon.SetNumberOfLEDs(2, 55);
		defaultAnimations();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void defaultAnimations(){
		photon.setAnimation(1,Photon.Animation.CYLON_MIDDLE_DUAL);
		photon.setAnimation(2,Photon.Animation.CYLON_MIDDLE_DUAL);
  }

  public void dashboard() {
  }

  //Modify to run a system check for the robot
  public boolean checkSystem() {
    return true;
  }
}
