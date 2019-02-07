/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.lib.drivers.SpectrumSparkMax;
import frc.robot.HW;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {

  public SpectrumSparkMax leftFrontMotor = new SpectrumSparkMax(HW.LEFT_DRIVE_FRONT);
  public SpectrumSparkMax leftMiddleMotor = new SpectrumSparkMax(HW.LEFT_DRIVE_MIDDLE);
  public SpectrumSparkMax leftRearMotor = new SpectrumSparkMax(HW.LEFT_DRIVE_REAR);
  public SpectrumSparkMax rightFrontMotor = new SpectrumSparkMax(HW.RIGHT_DRIVE_FRONT);
  public SpectrumSparkMax rightMiddleMotor = new SpectrumSparkMax(HW.RIGHT_DRIVE_MIDDLE);
  public SpectrumSparkMax rightRearMotor = new SpectrumSparkMax(HW.RIGHT_DRIVE_REAR);
  public CANEncoder leftEncoder = new CANEncoder(leftFrontMotor);
  public CANEncoder rightEncoder = new CANEncoder(rightFrontMotor);
  public CANPIDController leftPID = new CANPIDController(leftFrontMotor);
  public CANPIDController rightPID = new CANPIDController(rightFrontMotor);

  public DifferentialDrive diffDrive = new DifferentialDrive(leftFrontMotor, rightRearMotor);

  //set this up after we setup the cargo mechanism with the pigeon on it
  //public PigeonIMU imu = new PigeonIMU(1);

  double targetAngle;
	double turnThrottle;
	
	double currentAngle;
	double currentAngularRate;
  public double error = 0;
  
  public Drivetrain() {
    super("drivetrain");
    diffDrive.setSubsystem(this.getName());
    diffDrive.setDeadband(0.1);
    diffDrive.setMaxOutput(1.0);
    diffDrive.setSafetyEnabled(true);
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //Follow Motors
    leftMiddleMotor.follow(leftFrontMotor);
    leftRearMotor.follow(leftFrontMotor);
    rightMiddleMotor.follow(rightFrontMotor);
    rightRearMotor.follow(rightFrontMotor);
  }
	
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void zeroSensors() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    /*NEED TO ZERO PIGEON HERE*/
  }

  public void dashboard(){
    //Add values that need to be updated on the dashboard.
  }
}
