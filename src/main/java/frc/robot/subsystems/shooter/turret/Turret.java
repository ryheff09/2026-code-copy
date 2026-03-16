// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  //creating objects and initializing variables
  //Shooter side object, which applies not only to the turret which consists of 1 motor but the flywheel and hood as well, as all of these components work together to create one shooter)
  private final ShooterSide side;

  //standard turret io object
  private final TurretIO io;
  //standard auto logged class which is automatically generated due to the use of @Autolog in the io interface
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  //An instance of the outputs class created in the io interface
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  //sets the target angle of the turret equal to the zeroed angle
  private Rotation2d targetAngle = Rotation2d.kZero;

  //the default value for at goal is false, as the turret has not moved yet
  private boolean atGoal = false;
  //A debouncer object which controls the boolean output of whether or not the turret is at its goal
  //When the turret reaches its target and continues past it, it must remain as false for 0.1 seconds before the signal is actually read as false
  //This ensures that the turret actually stops at its target and doesn't skip over it
  private Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  //the default value for iszeroed is false, as the turret has not been calibrated yet
  private boolean isZeroed = false;

  /** Creates a new Turret. */
  //standard constructor
  public Turret(ShooterSide side, TurretIO io) {
    this.side = side;
    this.io = io;
  }

  @Override
  public void periodic() {
    //standard data logging
    io.updateInputs(inputs);
    Logger.processInputs(("Turret/" + side.getName()), inputs);

    //if a limit switch is triggered, the turret is zeroed
    if (inputs.limitTriggered) {
      isZeroed = true;
    }

    //if the shooter side is left, the left turret angle is set to the target angle input, and vice versa for the right
    if (side == ShooterSide.LEFT) {
      RobotVisualizer.getInstance().setLeftTurretAngle(Rotation2d.fromRadians(inputs.positionRad));
    } else if (side == ShooterSide.RIGHT) {
      RobotVisualizer.getInstance().setRightTurretAngle(Rotation2d.fromRadians(inputs.positionRad));
    }

    //The logger also stores the values of the turret side and what its target is (particularly useful in debugging)
    Logger.recordOutput(("Turret/" + side.getName() + "/TargetAngle"), targetAngle);
  }

  //The periodic after scheduler method is called periodically after the period method, particularly to apply outputs
  //You could just put everything in this method at the end of the periodic method, but I guess this is just for organization
  @Override
  public void periodicAfterScheduler() {
    //calls the default apply outputs method, which sets the motor to a certain speed to reach a target using closed loop controllers in both the hardware and simulation layers
    io.applyOutputs(outputs);
  }

  //A command to track the motor's progress to reaching the target
  //takes a supplier which returns a translation 2d value, which represents points or vectors on a coordinate plane
  public Command trackTarget(Supplier<Translation2d> targetSupplier) {

    //the command is a run command, meaning it executes the actions periodically until interrupted
    return Commands.run(
        () -> {
          //sets a translation2d target value equal to the target supplier (remember that suppliers can be changed, unlike variables, because they are functions)
          Translation2d target = targetSupplier.get();
          //sets a pose2d object, which is a combination of translation2d and rotation2d, to get the robot's approximate position and direction on the field
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();

          //takes a transform 3d object (point in 3d space, or translation 3d, + rotation2d) defined in constants and gets the translation2d from it
          //if the shooter side is left, it calls the left turret transform 3d object, and vice versa
          Translation2d turretOffset =
              (this.side == ShooterSide.LEFT
                  ? TurretConstants.kRobotToLeftTurret.getTranslation().toTranslation2d()
                  : TurretConstants.kRobotToRightTurret.getTranslation().toTranslation2d());

          // Turret position in field coordinates
          Translation2d turretFieldPos =
              robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

          // Vector from turret -> target (field frame)
          Translation2d deltaField = target.minus(turretFieldPos);

          // Convert to robot frame
          Translation2d deltaRobot = deltaField.rotateBy(robotPose.getRotation().unaryMinus());

          // Angle turret should point (robot-relative)
          Rotation2d targetAngle = new Rotation2d(Math.atan2(deltaRobot.getY(), deltaRobot.getX()));

          //after getting the target angle of the turret based on the robots position and rotation on the field, this command references the method below to actually set the motor to the target
          this.targetAngle = targetAngle;

          setPosition(targetAngle);
        },
        this);
  }

  //automatically zeroes the turret by running it at a certain speed until it hits the limit switch, using the is zeroed method defined above to determine this
  public Command zero() {
    return Commands.startEnd(() -> setOpenLoop(0.2), () -> stop()).until(this::isZeroed);
  }

  /**
   * Set the target angle for the turret.
   *
   * <p>This will clamp the angle to be within the maximum and minimum rotation of the turret.
   *
   * @param position A {@link Rotation2d} object representing the target position of the turret.
   */
  public void setPosition(Rotation2d position) {
    //if the turret is not zeroed the method stops
    if (!isZeroed) return; // safety

    targetAngle = position;

    //clamps the position between the turret's movement limits and converts it to radians
    position =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                position.getRadians(),
                TurretConstants.kMinTurretAngleRad,
                TurretConstants.kMaxTurretAngleRad));

    //again sets the turret mode to closed loop, and updates the value of the closed loop target based on the position parameter
    outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
    outputs.closedLoopTarget = position;

    //sets a variable at goal equal to the debounced value of whether or not the absolute value of the goal position minus the actual position (an input using the encoder) is less than the tolerance, or the acceptable level of error, defined in constants
    //Again, the debouncer prevents the motor from moving too fast and going past the target without the code noticing
    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(position.getRadians() - inputs.positionRad) < TurretConstants.kAngleTolerance);
  }

  //a safety method which clamps the open loop output defined in the outputs class between the standard -1.0 and 1.0 used to set the speed of a motor
  public void setOpenLoop(double output) {
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;
    outputs.openLoopOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  //another safety method which sets the open loop output defined in the outputs class to 0
  public void stop() {
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;
    outputs.openLoopOutput = 0.0;
  }

  //all the methods below return information about the state of the turret: 
  //the position (returns corresponding input)
  //the velocity (returns corresponding input)
  //whether or not the motor has reached its goal (defined in the command above)
  //whether or not the motor is zeroed (defined in the command above)
  //what the side of the shooter is (defined in a different shooter file)
  
  //note that all the commands do is set the change the values of the outputs in the outputs class; the method apply outputs does all the work of actually moving the motors
  public double getPosition() {
    return inputs.positionRad;
  }

  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean isZeroed() {
    return isZeroed;
  }

  public ShooterSide getSide() {
    return this.side;
  }
}
