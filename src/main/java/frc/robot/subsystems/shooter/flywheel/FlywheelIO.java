package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  //standard default update inputs method
  default void updateInputs(FlywheelIOInputs inputs) {}

  @AutoLog
  //standard inputs class which is eventually autologged
  //contains all the standard inputs (connected, velocity, voltage in volts, current in amps) and does not include a position value, because a flywheel just spins
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
  }

  /**
   * Set the shooter motor to a specified velocity.
   *
   * @param velocity The velocity to set the motor to (in RPS).
   */
  default void setVelocity(double velocity) {}

  /** Run motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  //default method to stop the motor
  default void stop() {}
}
