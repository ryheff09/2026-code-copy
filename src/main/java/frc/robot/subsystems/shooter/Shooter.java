// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.TrajectoryCalculator.ShooterCommand;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Hub;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  //initializing objects and variables
  //initialize shooter side variable 
  private final ShooterSide side;

  //subsystem objects for individual components of the shooter, which will all come together in this subsystem
  private Turret turret;
  private Hood hood;
  private Flywheel flywheel;

  /** Creates a new Shooter. */
  //all individual subsystem objects for the three components of the shooter are required as parameters, as well as the shooter side using the enum at the very bottom of this document
  public Shooter(ShooterSide side, TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.side = side;
    //creates a new turret, hood, and flywheel
    this.turret = new Turret(side, turretIO);
    this.hood = new Hood(side, hoodIO);
    this.flywheel = new Flywheel(side, flywheelIO);
  }

  //creates a different shooter which does not have a turret object (this is because we were not using april tags in minuteman and therefore did not need to rotate the turret)
  public Shooter(ShooterSide side, HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.side = side;
    this.turret = null;
    this.hood = new Hood(side, hoodIO);
    this.flywheel = new Flywheel(side, flywheelIO);
  }

  @Override
  public void periodic() {
    //the purpose of making each subsystem periodic within the periodic method is to create an even faster periodic loop
    hood.periodic();
    //checks if there is a turret or not (depending on which shooter object we use) and sets it to periodic if there is one
    if (turret != null) {
      turret.periodic();
    }
    flywheel.periodic();
  }

  //command to presumably shoot both shooters at once at the hub; references command to calculate trajecory based on a target written below, and provides this target as the center of the hub
  public static Command shootBothAtHub(Shooter leftShooter, Shooter rightShooter) {
    //sets the target for the shooters as the hub center point as a translation 2d (point on the coordinate plane)
    //the allianceFlipUtil class flips the coordinates of the hub automatically depending on what alliance/side of field we are on
    return shootBothAtTarget(
        leftShooter,
        rightShooter,
        () -> AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
  }

  /**
   * Calculate and apply trajectory parameters for both shooters.
   *
   * @param leftShooter The left shooter subsystem.
   * @param rightShooter The right shooter subsystem.
   * @param targetSupplier A supplier for the target.
   * @return A RunCommand applying trajectory parameters to both shooters.
   */
  public static Command shootBothAtTarget(
      Shooter leftShooter, Shooter rightShooter, Supplier<Translation2d> targetSupplier) {
    //returns a command which calculates trajectory parameters and applies it to both shooters
    return Commands.run(
        () -> {
          var cmds = TrajectoryCalculator.calculateBoth(targetSupplier.get());
          leftShooter.applyCommand(cmds.left());
          rightShooter.applyCommand(cmds.right());
        },
        leftShooter,
        rightShooter);
  }

  public static Command shootBothAtTargetNoTurret(
      Shooter leftShooter, Shooter rightShooter, Supplier<Translation2d> targetSupplier) {
    //same as the command above except it takes into account that there is no turret to rotate the shooter
    return Commands.run(
        () -> {
          var cmds = TrajectoryCalculator.calculateBoth(targetSupplier.get());
          leftShooter.applyCommandNoRotation(cmds.left());
          rightShooter.applyCommandNoRotation(cmds.right());
        },
        leftShooter,
        rightShooter);
  }

  //both the above commands reference methods written later

  /**
   * Apply a pre-calculated shooter command to this shooter. This does not require the shooter
   * subsystem - use when combining with other shooters.
   *
   * @param cmd The shot parameters to apply.
   */

  // a method which takes a command as a parameter and sets the velocity of the flywheel and position of the hood based on the information in the command
  public void applyCommand(ShooterCommand cmd) {
    flywheel.setVelocity(cmd.wheelRPM());
    hood.setAngle(cmd.hoodAngle());
    //again checks whether or not there is a turret before attempting to set its angle
    if (turret != null) {
      turret.setPosition(cmd.turretAngle());
    }
  }

  //a similar method which automatically assumes there is no turret
  public void applyCommandNoRotation(ShooterCommand cmd) {
    flywheel.setVelocity(cmd.wheelRPM());
    hood.setAngle(cmd.hoodAngle());
  }

  //more commands which calculate and implement trajectory parameters, both with and without turret rotation
  public Command shootAtTargetRotation(Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          ShooterCommand cmd = TrajectoryCalculator.calculate(side, targetSupplier.get());
          flywheel.setVelocity(cmd.wheelRPM());
          hood.setAngle(cmd.hoodAngle());
          turret.setPosition(cmd.turretAngle());
        },
        this,
        turret,
        hood,
        flywheel);
  }

  public Command shootAtTargetNoRotation(Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          ShooterCommand cmd = TrajectoryCalculator.calculate(side, targetSupplier.get());
          flywheel.setVelocity(cmd.wheelRPM());
          hood.setAngle(cmd.hoodAngle());
        },
        this,
        hood,
        flywheel);
  }

  //The following 3 commands implement methods created in the separate subsystems to track the target of the turret, zero the turret, and set the velocity of the flywheel (implemented in above commands)
  public Command trackTarget(Supplier<Translation2d> targetSupplier) {
    return turret.trackTarget(targetSupplier);
  }

  public Command zeroTurret() {
    return turret.zero();
  }

  public Command setFlywheelVelocity(double velocityRPM) {
    return flywheel.runVelocity(velocityRPM);
  }

  //the following 5 methods implement methods created in the separate subsystems to set the angle of the hood and set the position of the turret, as well as basic open loop logic for all 3 elements of the shooter
  //question: why is the flywheel velocity a command, but the hood and turret positions methods?
  public void setHoodAngle(double angle) {
    hood.setAngle(angle);
  }

  public void setTurretPosition(Rotation2d position) {
    turret.setPosition(position);
  }

  public void setFlywheelOpenLoop(double output) {
    flywheel.setOpenLoop(output);
  }

  public void setHoodOpenLoop(double output) {
    hood.setOpenLoop(output);
  }

  public void setTurretOpenLoop(double output) {
    turret.setOpenLoop(output);
  }

  //returns the shooter side
  public ShooterSide getSide() {
    return side;
  }

  //enum to define the two possible states of the shooter: left or right
  public enum ShooterSide {
    LEFT("Left"),
    RIGHT("Right");

    private String name;

    private ShooterSide(String name) {
      this.name = name;
    }

    //returns the name of the shooter (left or right)
    public String getName() {
      return name;
    }
  }
}
//The whole shooter is structured so that each motor gets a separate subsystem and contains methods to set position and/or velocity in both closed and open loop object
//This subsystem defines how each of the other subsystems should work together in commands to shoot fuel at a target (uses the trajectory calculator)