// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // initializeing objects and variables
  // standard shooter side object present in all subsystems which make up the
  // shooter
  private final ShooterSide side;

  // standard io object and autologged class
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // default value for atGoal is false because the flywheel has not started
  // spinning yet
  private boolean atGoal = false;

  // A debouncer object which controls the boolean output of whether or not the
  // flywheel is at its velocity goal
  // When the flywheel reaches its target and continues to get faster, the
  // flywheel must stay out of its range of acceptable velocity before the signal
  // is actually read as false, reflecting how the flywheel is no longer in an
  // acceptable velocity range
  // This ensures that the flywheel actually stops accelerating at its target and
  // doesn't skip over it
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  // initalizes the flywheel velocity goal in RPM
  private double goalRPM = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(ShooterSide side, FlywheelIO io) {
    this.side = side;
    this.io = io;
  }

  @Override
  public void periodic() {
    // default update inputs and data storage
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/" + side.getName() + "/Flywheel", inputs);
    // records the additional input of whether the flywheel has reached its target
    // velocity (particularly useful in debugging)
    Logger.recordOutput("Shooter/" + side.getName() + "/Flywheel/AtGoal", atGoal);

    // puts data values in smart dashboard of the current flywheel velocity and the
    // target velocity
    SmartDashboard.putNumber("Flywheel Velo", getVelocity());
    SmartDashboard.putNumber("Flywheel Setpoint", goalRPM);
  }

  // basic command which sets the motor to a specified velocity until interrupted,
  // in which case the motor stops
  // references the set velocity method written below
  public Command runVelocity(double velocityRPM) {
    return Commands.startEnd(
        () -> {
          setVelocity(velocityRPM);
        },
        () -> {
          stop();
        },
        this);
  }

  // method which sets the velocity of the motor
  public void setVelocity(double velocityRPM) {
    // velocity goal variable is set to the parameter of the method
    goalRPM = velocityRPM;

    // at goal variable is updated to the debounced value of the absolute value of
    // the velocity parameter converted to radians per second (what speed the motor
    // is supposed to be at) - the actual speed of the motor (evaluates to how far
    // away the motor is from reaching its target velocity), and whether or not this
    // is less than the speed tolerance defined in constants; in other words, is the motor at an acceptable speed
    atGoal = atGoalDebouncer.calculate(
        Math.abs(
            Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM)
                - inputs.velocityRadPerSec) < FlywheelConstants.kSpeedTolerance);
    // Rotations per minute -> rotations per second
    //references set velocity method (default method
    //defined in io interface, remember that this uses closed loop logic)
    io.setVelocity(velocityRPM / 60.0);
  }

  //method to set the velocity of the motor using simple open loop logic (calls default method defined in the io interface of the same name)
  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  //method to stop the motor (calls default method defined in the io interface of the same name)
  public void stop() {
    io.stop();
  }

  /**
   * Gets the current velocity of the flywheel
   *
   * @return A double representing the speed of the flywheel (in RPM).
   */
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  //returns the overall shooter side, which again is used in all subsystems which make up the shooter
  public ShooterSide getSide() {
    return this.side;
  }
}
