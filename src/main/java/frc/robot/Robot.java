/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.subsystems.CargoMech;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.PathFollower;
import frc.robot.subsystems.PhotonLEDs;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionJevois;
import frc.robot.subsystems.VisionLL;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumLogger;
import frc.lib.util.SpectrumPreferences;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

  
  // Add Debug flags
  // You can have a flag for each subsystem, etc
  public static final String _controls = "CONTROL";
  public static final String _general = "GENERAL";
  public static final String _auton = "AUTON";
  public static final String _commands = "COMMAND";
  public static final String _drive = "DRIVE";
  public static final String _elevator = "ELEVATOR";
  public static final String _climber = "CLIMBER";
  public static final String _cargo = "CARGO";
  public static final String _hatch = "HATCH";
	
	public static SpectrumLogger logger;
	public static SpectrumPreferences prefs;
	
	public static Pneumatics pneumatics;
	public static CargoMech cargoMech;
	public static Drivetrain drive;
	public static Climber climber;
	public static Elevator elevator;
	public static Hatch hatch;
	public static VisionLL visionLL;
	public static VisionJevois visionJevois;
	public static PhotonLEDs photon;
	public static int brownOutCtn = 0;
	public static DriverStation DS;
	public static PathFollower pathFollower;
	
	public static void setupSubsystems(){
		prefs = SpectrumPreferences.getInstance();
		pneumatics = new Pneumatics();
		cargoMech = new CargoMech(); //CargoMech has to be before Drivetrain for pigeon and before hatch for limitswitch
		drive = new Drivetrain();
		pathFollower = new PathFollower();
		climber = new Climber();
		elevator = new Elevator();
		hatch = new Hatch();
		visionLL = new VisionLL();
		//visionJevois = new VisionJevois();
		photon = new PhotonLEDs();
    }
	
	//Used to keep track of the robot current state easily
    public enum RobotState {
        DISABLED, AUTONOMOUS, TELEOP, TEST
    }

    public static RobotState s_robot_state = RobotState.DISABLED;

    public static RobotState getState() {
        return s_robot_state;
    }

    public static void setState(RobotState state) {
        s_robot_state = state;
    }
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
		DS = DriverStation.getInstance();
		initDebugger(); //Init Debugger
		SpectrumLogger.getInstance().intialize();  //setup files for logging
		printInfo("Start robotInit()");
    	setupSubsystems(); //This has to be before the OI is created on the next line
		HW.oi = new OI();
		Dashboard.intializeDashboard();
        Robot.visionLL.initialize();
		SpectrumLogger.getInstance().finalize();  //Finalize the logging items
	}

	//Add any code that needs to run in all states
	public void robotPeriodic() {
		if (RobotController.isBrownedOut()){
			brownOutCtn++;
		}
	}
	
	 /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
        setState(RobotState.DISABLED);
        printInfo("Start disabledInit()");
        Disabled.init();
        printInfo("End disableInit()");
    }
    /**
     * This function is called while in disabled mode.
     */    
    public void disabledPeriodic(){
    	Disabled.periodic();
    }


    public void autonomousInit() {
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
    	setState(RobotState.AUTONOMOUS);
		printInfo("Start autonomousInit()");
		Autonomous.init();
        printInfo("End autonomousInit()");
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Autonomous.periodic();
    }

    public void teleopInit() {
		initDebugger();//Used to set debug level lower when FMS attached.
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
    	setState(RobotState.TELEOP);
    	printInfo("Start teleopInit()");
		Teleop.init();
        printInfo("End teleopInit()");
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Teleop.periodic();
    }
	
	public void testInit() {
		//Disable LiveWindow
		LiveWindow.setEnabled(false);
		LiveWindow.disableAllTelemetry();
        Scheduler.getInstance().removeAll();

		//Check Systems
		//** ADD SOMETHING TO CHECK THAT PDP doesn't have faults */
		//Check each subsytem including pneumatics
		setState(RobotState.TEST);
		 Timer.delay(0.5);
		 System.out.println("!!!!!!!!!!!!!SYSTEM CHECK STARTING!!!!!!!!!!");
		 
		 boolean results = true;
		results &= drive.checkSystem();
		//results &= cargoMech.checkSystem();
		//results &= hatch.checkSystem();
		results &= elevator.checkSystem();
		
		 /** Examples of testing subsystems based on 254-2017 Code
	        results &= Feeder.getInstance().checkSystem();
	        results &= Drive.getInstance().checkSystem();
	        results &= Intake.getInstance().checkSystem();
	        results &= MotorGearGrabber.getInstance().checkSystem();
	        results &= Shooter.getInstance().checkSystem();
	        results &= Hopper.getInstance().checkSystem();
		  */
        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
	        
	}
	/**
	 * This function is called periodically during test mode.
	 * Use this to have commands that we can test with like camera tests, etc.
	 */
	
	public void testPeriodic() {
	}
	
    private static void initDebugger(){
		if(DS.isFMSAttached()){
			Debugger.setLevel(Debugger.info3);
		}else{
			Debugger.setLevel(Debugger.info3); //Set the initial Debugger Level for when not in a match
		}
    	Debugger.flagOn(_general); //Set all the flags on, comment out ones you want off
    	Debugger.flagOn(_controls);
    	Debugger.flagOn(_auton);
    	Debugger.flagOn(_commands);
    	Debugger.flagOn(_drive);
    	Debugger.flagOn(_elevator);
    	Debugger.flagOn(_climber);
    	Debugger.flagOn(_cargo);
    	Debugger.flagOn(_hatch);
    }
    
    public static void printDebug(String msg){
    	Debugger.println(msg, _general, Debugger.debug2);
    }
    
    public static void printInfo(String msg){
    	Debugger.println(msg, _general, Debugger.info3);
    }
    
    public static void printWarning(String msg) {
    	Debugger.println(msg, _general, Debugger.warning4);
    }
    
    
}
