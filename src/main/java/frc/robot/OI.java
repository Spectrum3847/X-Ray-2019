/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.lib.controllers.SpectrumAndNotButton;
import frc.lib.controllers.SpectrumAxisButton;
import frc.lib.controllers.SpectrumIOButton;
import frc.lib.controllers.SpectrumOrButton;
import frc.lib.controllers.SpectrumTwoButton;
import frc.lib.controllers.SpectrumXboxController;
import frc.lib.controllers.SpectrumAxisButton.ThresholdType;
import frc.lib.controllers.SpectrumXboxController.XboxAxis;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimberKicker;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.auto.LvlOneTwoRocketLeft;
import frc.robot.commands.auto.LvlOneTwoRocketRight;
import frc.robot.commands.auto.SmartMotionDrive;
import frc.robot.commands.cargo.*;
import frc.robot.commands.drive.AutoHatchIntake;
import frc.robot.commands.drive.AutoTurn;
import frc.robot.commands.drive.BrakeMode;
import frc.robot.commands.drive.LLDrive;
import frc.robot.commands.elevator.CheckIfZero;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.MMElevator;
import frc.robot.commands.elevator.SimpleElevatorGoToPos;
import frc.robot.commands.hatch.*;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public static SpectrumXboxController driverController;
  public static SpectrumXboxController opController;
  public static SpectrumAxisButton leftTriggerButton;
  public static SpectrumAxisButton rightTriggerButton;
  //public static SpectrumAxisButton rightTriggerCanelIntake;
  public static SpectrumAxisButton leftStickIn;
  public static SpectrumAxisButton leftStickCargoShip;
  public SpectrumOrButton DriverLeftDpad;
  public SpectrumOrButton DriverRightDpad ;

  public OI() {
		driverController = new SpectrumXboxController(0, .17, .05);
    opController = new SpectrumXboxController(1, .15, .15);

    //Driver Buttons
    //A button is aim with camera inside drive() command
    //driverController.yButton.whileHeld(new Climb());
    //driverController.selectButton.whenPressed(new ClimberKicker());
    driverController.bButton.whileHeld(new AutoHatchIntake());
    driverController.xButton.whileHeld(new LLDrive());
    new SpectrumAxisButton(OI.driverController, XboxAxis.RIGHT_X, .3, ThresholdType.DEADBAND).whileHeld(new BrakeMode());//Go to brake mode when doing one side turn thing

    driverController.startButton.whenPressed(new SmartMotionDrive(-7.85, 7.85));

    
    DriverLeftDpad = new SpectrumOrButton(driverController.Dpad.Left, new SpectrumOrButton(driverController.Dpad.UpLeft, driverController.Dpad.DownLeft));
    DriverRightDpad = new SpectrumOrButton(driverController.Dpad.Right, new SpectrumOrButton(driverController.Dpad.UpRight, driverController.Dpad.DownRight));
    DriverLeftDpad.whenPressed(new AutoTurn(70, DriverLeftDpad));
    DriverRightDpad.whenPressed(new AutoTurn(-70, DriverRightDpad));

    //Operator Buttons

    //Cargo and Hatch Controls
    rightTriggerButton = new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.RIGHT_TRIGGER, .5, ThresholdType.GREATER_THAN);
    Button intakeBtn = new SpectrumAndNotButton(rightTriggerButton, opController.Dpad.Left);
    IntakeCargo in = new IntakeCargo(intakeBtn);
    intakeBtn.whenPressed(in);

    leftTriggerButton = new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.LEFT_TRIGGER, .5, ThresholdType.GREATER_THAN);
    new SpectrumAndNotButton(leftTriggerButton, opController.Dpad.Left).whileHeld(new FireCargo());

    new SpectrumAndNotButton(opController.leftBumper, opController.Dpad.Left).whenPressed(new MMElevator((int)Robot.prefs.getNumber("C: ElevatorHeight", 2000)));

    SpectrumOrButton leftDpad = new SpectrumOrButton(opController.Dpad.Left, new SpectrumOrButton(opController.Dpad.UpLeft, opController.Dpad.DownLeft));
    new SpectrumTwoButton(leftDpad, opController.rightBumper).whileHeld(new TiltDown());
    new SpectrumTwoButton(leftDpad, opController.leftBumper).whileHeld(new HatchRelease());
    new SpectrumTwoButton(leftDpad, leftTriggerButton).whileHeld(new HatchFire());
    new SpectrumTwoButton(leftDpad, rightTriggerButton).whileHeld(new HatchReady());

    leftStickIn = new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.LEFT_Y, -.25, ThresholdType.LESS_THAN);
    leftStickIn.whileHeld(new RollerBottomOn(1.0));
    leftStickIn.whileHeld(new RollerTopOn(1.0));

    leftStickCargoShip = new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.LEFT_Y, .25, ThresholdType.GREATER_THAN);
    leftStickCargoShip.whileHeld(new CargoShipDrop());

    new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.RIGHT_X, -.5, ThresholdType.LESS_THAN).whileHeld(new BullDozer());

    //Elevator Controls
    opController.startButton.whileHeld(new ElevatorZero());
    SpectrumIOButton cargoButton = new SpectrumIOButton(Robot.cargoMech.CargoSW);
    SpectrumOrButton rightDpad =  new SpectrumOrButton(opController.Dpad.Right, new SpectrumOrButton(opController.Dpad.UpRight, opController.Dpad.DownRight));
    SpectrumOrButton  cargoOverRideable = new SpectrumOrButton(cargoButton, rightDpad);
    opController.aButton.whenPressed(new CheckIfZero());
    new SpectrumTwoButton(opController.aButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posCargoL1));
    new SpectrumTwoButton(opController.xButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posCargoL2));
    new SpectrumTwoButton(opController.yButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posCargoL3));
    new SpectrumTwoButton(opController.bButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posCargoShip));
    new SpectrumAndNotButton(opController.aButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posDownLimit));
    new SpectrumAndNotButton(opController.bButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posHatchL2));
    new SpectrumAndNotButton(opController.xButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posHatchL2));
    new SpectrumAndNotButton(opController.yButton, cargoOverRideable).whenPressed(new MMElevator(Elevator.posHatchL3));

    new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.RIGHT_Y, -.15, ThresholdType.LESS_THAN).whileHeld(new ManualElevator());
    new SpectrumAxisButton(OI.opController, SpectrumXboxController.XboxAxis.RIGHT_Y, .15, ThresholdType.GREATER_THAN).whileHeld(new ManualElevator());

  }

  public boolean isOperatorButtonPushed(){
    if (opController.aButton.get() 
        || opController.bButton.get() 
        || opController.xButton.get()
        || opController.yButton.get()
        || opController.rightBumper.get()
        || opController.leftBumper.get()
        || opController.startButton.get()
        || opController.selectButton.get()){
          return true;
        }
    else{
      return false;
    }
  }

  public boolean isDriverButtonPushed(){
    if (driverController.aButton.get() 
        || driverController.bButton.get() 
        || driverController.xButton.get()
        || driverController.yButton.get()
        || driverController.rightBumper.get()
        || driverController.leftBumper.get()
        || driverController.startButton.get()
        || driverController.selectButton.get()){
          return true;
        }
    else{
      return false;
    }
  }

  /*
   * Example Buttons
   * 
    //Left Trigger sets the speed of the punch either wheeled outtake, soft punch, full punch in OperatorPuncher() command
		operatorController.rightBumper.toggleWhenPressed(new OperatorEject());

		//Left Bumper reverses these positions in SetArmPos command
		operatorController.aButton.whileHeld(new SetArmPos(Arm.Position.FwdIntake));
		operatorController.xButton.whileHeld(new SetArmPos(Arm.Position.FwdScore));
   */

  

}
