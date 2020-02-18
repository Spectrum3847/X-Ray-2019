package frc.robot.subsystems;

//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LimeLight;
//import frc.lib.drivers.SpectrumJeVois;
import frc.lib.drivers.LimeLightControlModes.LedMode;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;

public class VisionLL extends PIDSubsystem{
    //Use this for limelight code and Jevois code
    
    public final LimeLight limelight;
    //private boolean m_LimelightHasValidTarget = false;
    //private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    private boolean LEDstate = true;
    
	//public final SpectrumJeVois jevoisCam;

    public VisionLL(){
        super(0.0,0.0,0.0);
        limelight = new LimeLight();
        //jevoisCam = new SpectrumJeVois();
        //jevoisCam.setSerOutEnable(true);
        setInputRange(-27, 27);
        this.getPIDController().setOutputRange(-.3, .3);
        setSetpoint(0.0);
    }

    public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
    }

    //Runs everytime the robot is enabled
    public void initialize(){
        setPDF(Robot.prefs.getNumber("V: P", 0.17), Robot.prefs.getNumber("V: D", 0.2), Robot.prefs.getNumber("V: F", 0.0));
        setPercentTolerance(2);
        setSetpoint(0.0);
        enable();
    }
    
    //Tasks that need to be run at all times
    public void periodic() {
        //If disabled and LED-Toggle is false, than leave lights off, else they should be on
        if(Robot.s_robot_state == RobotState.DISABLED && !SmartDashboard.getBoolean("Limelight-LED Toggle", false) && !DriverStation.getInstance().isFMSAttached()){
            if (LEDstate == true){
                limeLightLEDOff();
                LEDstate = false;
            }   
        } else {
            if (LEDstate == false){
                limeLightLEDOn();
                LEDstate = true;
            }
        }
    }

    public double returnPIDInput(){
        return limelight.getdegRotationToTarget();
    }

    public void usePIDOutput(double output){
        m_LimelightSteerCommand = output * -1;
    }

    public void setPDF(double P, double D, double F){
        getPIDController().setP(P);
        getPIDController().setD(D);
        getPIDController().setF(F);
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
        return limelight.getIsTargetFound();
    }

    //50 to -50, on our robot 10-45 is about as much as you get, 45 being very close to target 10 being far away, after 45 it will go back down.
    public double getLLDegVertical(){
        return limelight.getdegVerticalToTarget();
    }

    public double getLimelightSteerCommand(){
        return m_LimelightSteerCommand;
    }

    //Put values you want to show up on the dashboard here
    public void dashboard(){
        SmartDashboard.putNumber("Vision/SteerOut", getLimelightSteerCommand());
    }

    

    
    
}