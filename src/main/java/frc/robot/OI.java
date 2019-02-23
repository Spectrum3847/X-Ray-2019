/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.controllers.SpectrumAndNotButton;
import frc.lib.controllers.SpectrumAxisButton;
import frc.lib.controllers.SpectrumIOButton;
import frc.lib.controllers.SpectrumTwoButton;
import frc.lib.controllers.SpectrumXboxController;
import frc.lib.controllers.SpectrumAxisButton.ThresholdType;
import frc.robot.commands.BrakeMode;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimberKicker;
import frc.robot.commands.cargo.*;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.MotionMagicElevator;
import frc.robot.commands.elevator.SimpleElevatorGoToPos;
import frc.robot.commands.hatch.*;
import frc.robot.subsystems.Elevator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public static SpectrumXboxController driverController;
  public static SpectrumXboxController operatorController;
  public static SpectrumAxisButton leftTriggerButton;
  public static SpectrumAxisButton rightTriggerButton;
  //public static SpectrumAxisButton rightTriggerCanelIntake;
  public static SpectrumAxisButton leftStickIn;
  public static SpectrumAxisButton leftStickCargoShip;

  public OI() {
		driverController = new SpectrumXboxController(0, .17, .05);
    operatorController = new SpectrumXboxController(1, .15, .15);

    //Driver Buttons
    //A button is aim with camera inside drive() command
    //driverController.yButton.whileHeld(new Climb());
    //driverController.selectButton.toggleWhenPressed(new ClimberKicker());
    driverController.bButton.whileHeld(new BrakeMode());


    //Operator Buttons
    IntakeCargo in = new IntakeCargo();
    rightTriggerButton = new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.RIGHT_TRIGGER, .5, ThresholdType.GREATER_THAN);
    new SpectrumAndNotButton(rightTriggerButton, operatorController.Dpad.Left).whileHeld(in);

    leftTriggerButton = new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.LEFT_TRIGGER, .5, ThresholdType.GREATER_THAN);
    new SpectrumAndNotButton(leftTriggerButton, operatorController.Dpad.Left).whileHeld(new FireCargo());

    new SpectrumTwoButton(operatorController.Dpad.Left, operatorController.rightBumper).whileHeld(new TiltDown());
    new SpectrumTwoButton(operatorController.Dpad.Left, leftTriggerButton).whileHeld(new HatchFire());
    new SpectrumTwoButton(operatorController.Dpad.Left, rightTriggerButton).whileHeld(new HatchReady());

    operatorController.startButton.whileHeld(new ElevatorZero());
    SpectrumIOButton cargoButton = new SpectrumIOButton(Robot.cargoMech.CargoSW);
    new SpectrumTwoButton(operatorController.aButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posCargoL1));
    new SpectrumTwoButton(operatorController.xButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posCargoL2));
    new SpectrumTwoButton(operatorController.yButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posCargoL3));
    new SpectrumTwoButton(operatorController.bButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posCargoShip));
    new SpectrumAndNotButton(operatorController.aButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posDownLimit));
    new SpectrumAndNotButton(operatorController.bButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posHatchL2));
    new SpectrumAndNotButton(operatorController.xButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posHatchL2));
    new SpectrumAndNotButton(operatorController.yButton, cargoButton).whenPressed(new MotionMagicElevator(Elevator.posHatchL3));

    new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.RIGHT_Y, -.15, ThresholdType.LESS_THAN).whileHeld(new ManualElevator());
    new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.RIGHT_Y, .15, ThresholdType.GREATER_THAN).whileHeld(new ManualElevator());

    leftStickIn = new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.LEFT_Y, -.25, ThresholdType.LESS_THAN);
    leftStickIn.whileHeld(new RollerBottomOn(1.0));
    leftStickIn.whileHeld(new RollerTopOn(1.0));

    leftStickCargoShip = new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.LEFT_Y, .25, ThresholdType.GREATER_THAN);
    leftStickCargoShip.whileHeld(new CargoShipDrop());

  }

  public boolean isOperatorButtonPushed(){
    if (operatorController.aButton.get() 
        || operatorController.bButton.get() 
        || operatorController.xButton.get()
        || operatorController.yButton.get()
        || operatorController.rightBumper.get()
        || operatorController.leftBumper.get()
        || operatorController.startButton.get()
        || operatorController.selectButton.get()){
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
