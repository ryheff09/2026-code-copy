// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  //creating objects and initializing variables
  //shooter side object, which again applies to all components of the shooter
  private final ShooterSide side;

  //standard io interface object
  private final HoodIO io;
  //standard autologged class
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  //the default value for whether or not the turret is at its goal is false, as it hasn't moved yet
  private boolean atGoal = false;

  //A debouncer object which controls the boolean output of whether or not the turret is at its goal
  //When the turret reaches its target and continues past it, it must remain as false for 0.2 seconds before the signal is actually read as false
  //This ensures that the turret actually stops at its target and doesn't skip over it
  //This time value is higher than that of the turret; is this because the hood moves faster?
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  /** Creates a new Hood. */
  public Hood(ShooterSide side, HoodIO io) {
    this.side = side;
    this.io = io;
  }

  @Override
  public void periodic() {
    //default update inputs and data storage
    io.updateInputs(inputs);
    Logger.processInputs(("Hood/" + side.getName()), inputs);

    //sets the angle of the left or right turret to the position in radians input depending again on the shooter side
    if (side == ShooterSide.LEFT) {
      RobotVisualizer.getInstance().setLeftHoodAngle(inputs.positionRad);
    } else if (side == ShooterSide.RIGHT) {
      RobotVisualizer.getInstance().setRightHoodAngle(inputs.positionRad);
    }
  }

  /**
   * Sets the hood to the target angle.
   *
   * @param angle The target angle (in radians).
   */
  public void setAngle(double angle) {
    //sets an at goal variable equal to the debounced value of whether or not the absolute value of the target minus the actual position of the hood (referenced using the input), which is how far the hood is from its target

    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(angle - inputs.positionRad) < HoodConstants.kAngleTolerance);
    //sets the angle of the hood using the method defined in the io interface, which happens to be named exactly the same as the current method
    io.setAngle(angle);
  }

  //method which sets the speed of the motor, again referencing a default method defined in the io interface
  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  //The methods below return information about the state of the hood:
  //The position (referencing the relevant input)
  //the velocity (referencing the relevant input)
  //the goal (referencing the at goal variable defined above in the set angle method)
  //the side of the shooter (ultimately defined in a different shooter file)

  public double getPosition() {
    return inputs.positionRad;
  }

  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean atGoal() {
    return atGoal;
  }

  public ShooterSide getSide() {
    return this.side;
  }
}
