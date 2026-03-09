package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  /**
   * This class contains the following inputs to be logged as data:
   * Whether or not the turret is connected, the turret position in radians, the
   * turret velocity in radians per second, whether or not the limit switch which
   * limits the turret's movement is triggered, and the default inputs of applied
   * volts and amps.
   */
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
    public boolean limitTriggered = false;
  }

  /**
   * This enum specifies two possible states for the turret: closed loop (with
   * PID) or open loop (without PID)
   */
  public static enum TurretIOOutputMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  /**
   * This class sets the state of the turret to closed loop based on the enum. It
   * also initializes the target using the rotation 2d class (to be used with a
   * PID controller to improve accuracy)
   */
  public class TurretIOOutputs {
    public TurretIOOutputMode mode = TurretIOOutputMode.CLOSED_LOOP;

    public double openLoopOutput = 0.0;
    public Rotation2d closedLoopTarget = Rotation2d.kZero;
  }

  void updateInputs(TurretIOInputs inputs);

  void applyOutputs(TurretIOOutputs outputs);
}
