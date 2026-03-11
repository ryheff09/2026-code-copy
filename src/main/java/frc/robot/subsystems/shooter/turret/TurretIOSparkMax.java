package frc.robot.subsystems.shooter.turret;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants.DeviceIDs;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import java.util.function.DoubleSupplier;

public class TurretIOSparkMax implements TurretIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController motorController; //creates an object for closed loop control, in this case pid

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling); 
  //ensures that the button is let go of for 0.5 seconds before reading the signal as false; this eliminates noise or errors from triggering something unwanted.

  public TurretIOSparkMax(ShooterSide side) {
    //takes the shooter side enum as a parameter which is defined in the shooter subsystem and combines the turret, hood, and flywheel subsystems (all of which together make 1 of 2 shooters)
    //creates a NEO motor and sets its ID (defined in the constants file) depending on the shooter side parameter
    motor =
        new SparkMax(
            side == ShooterSide.LEFT ? DeviceIDs.kLeftTurretAzimuth : DeviceIDs.kRightTurretAzimuth,
            MotorType.kBrushless);
    encoder = motor.getEncoder();
    //the closed loop controller is built in
    motorController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    //sets the idle mode of the turret motor (when it recieves 0 power) to coasting (the motor is moveable)
    config.idleMode(IdleMode.kCoast);
    // TODO: Tune
    config.inverted(side == ShooterSide.RIGHT);
    // .smartCurrentLimit(30);

    config
        .encoder
        .positionConversionFactor(
            2 * Math.PI / TurretConstants.kGearRatio) // No absolute encoder...
        .velocityConversionFactor(2 * Math.PI / TurretConstants.kGearRatio / 60.0);

    config.closedLoop.positionWrappingEnabled(false).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config
        .softLimit
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(TurretConstants.kMinTurretAngleRad)
        .forwardSoftLimit(TurretConstants.kMaxTurretAngleRad);

    config.closedLoop.feedForward.kS(0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> encoder.setPosition(0));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentDrawAmps = value);
    inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case CLOSED_LOOP -> {
        double clampedPosition =
            MathUtil.clamp(
                outputs.closedLoopTarget.getRadians(),
                TurretConstants.kMinTurretAngleRad,
                TurretConstants.kMaxTurretAngleRad);

        motorController.setSetpoint(clampedPosition, ControlType.kPosition);
      }
      case OPEN_LOOP -> {
        motor.set(MathUtil.clamp(outputs.openLoopOutput, -1.0, 1.0));
      }
    }
  }
}
