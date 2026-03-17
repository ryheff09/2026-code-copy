package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceIDs;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  //standard initialization of objects: a talon fx motor and corresponding config
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  //initializes status signals (timestamped data points) for the velocity acceleration, voltage, and current of the flywheel
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;

  //creates an object which depends on feedforward to set the voltage of the motor to get it to a specified velocity.
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public FlywheelIOTalonFX(ShooterSide side) {
    //creates the talon fx motor and sets its id (defined in constants) based on the shooter side
    motor =
        new TalonFX(
            side == ShooterSide.LEFT
                ? DeviceIDs.kLeftTurretFlywheel
                : DeviceIDs.kRightTurretFlywheel);
    
    //creates the motor config and sets it to inverted if the shooter side is right
    motorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        side == ShooterSide.RIGHT
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            //updates all the pid gains
            .withSlot0(FlywheelConstants.kGains)
            /**
             * TODO: Update gains Peiwei, Ben: see the FlywheelConstants.kGains above... thats where
             * the values are You also might have to check if the inverted values are correct,
             * positive should spin the right way for shooting (line above that has the
             * withInverted() method)
             */
            //sets the motor outputs to values defined in the constants file, including enabling coasting and setting the inverted direction to clockwise.
            .withMotorOutput(FlywheelConstants.kOutputConfigs);
    //tries to configure the motor by applying the motor config object to the configurator object (works differently than spark maxs and neos) every 0.25 seconds up to 5 times
    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));

    //sets the values of the 4 status signal objects initialized above using the simple motor methods
    velocitySignal = motor.getVelocity();
    accelerationSignal = motor.getAcceleration();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();

    //sets a common frequency for all the 4 status signal objects
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, velocitySignal, accelerationSignal, voltageSignal, currentSignal);
        //optimizes the frequency for the status signal objects by reducing it. 
        motor.optimizeBusUtilization();
  }

  @Override
  //default update inputs mehtod
  public void updateInputs(FlywheelIOInputs inputs) {
    //sets the connected variable to true if the code is able to read all the status signals defined above after they are refreshed (this is what the .isOk() method checks for)
    inputs.connected =
        BaseStatusSignal.refreshAll(
                velocitySignal, accelerationSignal, voltageSignal, currentSignal)
            .isOK();
    
    //updates the values of the inputs using the status signals
    inputs.velocityRadPerSec = velocitySignal.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentDrawAmps = currentSignal.getValueAsDouble();
  }

  @Override
  //default method which sets the motor to a specified velocity using pid control and feedforward (closed loop logic)
  public void setVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  //default method which sets the speed of the motor using open loop object
  public void setOpenLoop(double output) {
    motor.set(output);
  }

  @Override
  //default method which stops the motor
  public void stop() {
    motor.stopMotor();
  }
}
//note: instead of using the spark sticky fault to determine whether the flywheel is connected,
//which both the turret and hood use, this subsystem uses status signals
//this is because the flywheel is a Kraken (talon fx motor controller) rather than a Neo (spark max motor controller)
