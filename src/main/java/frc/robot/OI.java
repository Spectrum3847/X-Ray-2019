/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.controllers.SpectrumAxisButton;
import frc.lib.controllers.SpectrumXboxController;
import frc.lib.controllers.SpectrumAxisButton.ThresholdType;
import frc.robot.commands.cargo.*;
import frc.robot.commands.elevator.ElevatorZero;
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
  public static SpectrumAxisButton leftTriggerFire;
  public static SpectrumAxisButton rightTriggerIntake;
  public static SpectrumAxisButton rightTriggerCanelIntake;

  public OI() {
		driverController = new SpectrumXboxController(0, .02, .02);
    operatorController = new SpectrumXboxController(1, .15, .15);

    //Driver Buttons


    //Operator Buttons
    operatorController.leftBumper.whileHeld(new HatchEject());
    operatorController.rightBumper.whileHeld(new HatchFire());
    operatorController.selectButton.whileHeld(new ElevatorZero());
    operatorController.aButton.whileHeld(new SimpleElevatorGoToPos(Elevator.posCargoL1));
    operatorController.bButton.whenPressed(new SimpleElevatorGoToPos(Elevator.posHatchL2));
    operatorController.xButton.whenPressed(new SimpleElevatorGoToPos(Elevator.posCargoL2));
    operatorController.yButton.whenPressed(new SimpleElevatorGoToPos(Elevator.posCargoL3));
    
    IntakeCargo in = new IntakeCargo();
    rightTriggerIntake = new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.RIGHT_TRIGGER, .5, ThresholdType.GREATER_THAN);
    rightTriggerIntake.toggleWhenPressed(in);

    leftTriggerFire = new SpectrumAxisButton(OI.operatorController, SpectrumXboxController.XboxAxis.LEFT_TRIGGER, .5, ThresholdType.GREATER_THAN);
    leftTriggerFire.whileHeld(new FireCargo());

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
