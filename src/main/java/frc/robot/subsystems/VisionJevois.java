package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SpectrumJeVois;
import frc.robot.Robot;

public class VisionJevois extends PIDSubsystem{
    
    public final SpectrumJeVois jevois;
    //private boolean m_JevoisHasValidTarget = false;
    //private double m_JeVoisDriveCommand = 0.0;
    private double m_JevoisSteerCommand = 0.0;
    //private boolean LEDstate = true;
    
	//public final SpectrumJeVois jevoisCam;

    public VisionJevois(){
        super(0.0,0.0,0.0);
        jevois = new SpectrumJeVois();
        jevois.setSerOutEnable(true);
        setInputRange(-1000, 1000);
        setOutputRange(-.3, .3 );
        setSetpoint(0.0);
    }

    public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
    }

    //Runs everytime the robot is enabled
    public void initialize(){
        setPDF(Robot.prefs.getNumber("Vj: P", 0.17), Robot.prefs.getNumber("Vj: D", 0.2), Robot.prefs.getNumber("Vj: F", 0.0));
        setPercentTolerance(2);
        setSetpoint(0.0);
        enable();
    }
    
    //Tasks that need to be run at all times
    public void periodic() {

    }

    public double returnPIDInput(){
        return getVanishingPoint();
    }

    public void usePIDOutput(double output){
        m_JevoisSteerCommand = output * -1;
    }

    public void setPDF(double P, double D, double F){
        getPIDController().setP(P);
        getPIDController().setD(D);
        getPIDController().setF(F);
    }

    public double getVanishingPoint(){
        return 0;  // NEED TO FIGURE OUT HOW TO GET JEVOIS VANISHING POINT VALUE
    }

    public boolean getIsTargetFound(){
        return false; //RETURN TRUE IF JEVOIS GIVING VALID VANISHING POINT
    }

    public double getJevoisSteerCommand(){
        return m_JevoisSteerCommand;
    }

    //Put values you want to show up on the dashboard here
    public void dashboard(){
        SmartDashboard.putNumber("Vision/JevoisSteerOut", getJevoisSteerCommand());
    }

    

    
    
}