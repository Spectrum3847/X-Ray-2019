package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LimeLight;
//import frc.lib.drivers.SpectrumJeVois;
import frc.lib.drivers.LimeLightControlModes.LedMode;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;

public class Vision extends Subsystem{
    //Use this for limelight code and Jevois code
    
    public final LimeLight limelight;
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    
	//public final SpectrumJeVois jevoisCam;

    public Vision() {
        limelight = new LimeLight();
        //jevoisCam = new SpectrumJeVois();
        //jevoisCam.setSerOutEnable(true);
    }

    //Tasks that need to be run at all times
    public void periodic() {
        //If disabled and LED-Toggle is false, than leave lights off, else they should be on
        if(Robot.s_robot_state == RobotState.DISABLED && !SmartDashboard.getBoolean("Limelight-LED Toggle", false)){
            limeLightLEDOff();
        } else {
            limeLightLEDOn();
        }

        Update_Limelight_Tracking();
    }

    public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

    public void limeLightLEDOff(){
        limelight.setLEDMode(LedMode.kforceOff);
    }

    public void limeLightLEDOn(){
        limelight.setLEDMode(LedMode.kforceOn);
    }

    public void setLimeLightLED(boolean b){
        if (b){
            limeLightLEDOn();
        } else{
            limeLightLEDOff();
        }
    }

    public double getLLDegToTarget(){
        return limelight.getdegRotationToTarget();
    }

    public boolean getLLIsTargetFound(){
        return limelight.getIsTargetFound();
    }

    public double getLLTargetArea(){
        return limelight.getTargetArea();
    }

    public boolean getLimelightHasValidTarget(){
        return m_LimelightHasValidTarget;
    }

    /*
    public double getLimelightDriveCommand(){
        return m_LimelightDriveCommand;
    }*/

    public double getLimelightSteerCommand(){
        return m_LimelightSteerCommand;
    }

    /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.02;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
        final double MIN_STEER = .06;                   // Minimum amount to adjust steering

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = limelight.getdegRotationToTarget(); //.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        //double ty = limelight.getdegVerticalToTarget(); //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        //double ta = limelight.getTargetArea();//NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steering_adjust = 0;
        if (tx > 0.5)
        {
                steering_adjust = STEER_K*tx + MIN_STEER;
        }
        else if (tx < -0.5)
        {
                steering_adjust = STEER_K*tx - MIN_STEER;
        }
        m_LimelightSteerCommand = steering_adjust;

        /* UNCOMMENT IF WE NEED TO USE DRIVE COMMAND
        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
        */
  }

    //Put values you want to show up on the dashboard here
    public void dashboard(){

    }

    

    
    
}