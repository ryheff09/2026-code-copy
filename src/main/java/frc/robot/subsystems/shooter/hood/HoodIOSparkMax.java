package frc.robot.subsystems.shooter.hood;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import java.util.function.DoubleSupplier;

public class HoodIOSparkMax implements HoodIO {
  //standard objects: motor, corresponding encoder, a closed loop controller (pidf), and another connected debouncer (for more detail see turret documentation)
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController motorController;
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  //the constructor which takes the universal shooter side as a parameter, which again applies to all elements of the shooter
  public HoodIOSparkMax(ShooterSide side) {
    //sets the id of the motor depending on the shooter side
    motor =
        new SparkMax(
            side == ShooterSide.LEFT ? DeviceIDs.kLeftTurretHood : DeviceIDs.kRightTurretHood,
            MotorType.kBrushless);
    encoder = motor.getEncoder();
    motorController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    //sets the idle mode of the turret motor (when it recieves 0 power) to coasting (the motor is moveable)
    config.idleMode(IdleMode.kCoast);

    //inverts the motor config if the shooter side is on the right, making everything else universal
    config.inverted(side == ShooterSide.RIGHT);

    //automatically converts units from radians into degrees so that whenever 
    //you call the .getPosition method for the encoder, it returns in this unit,
    //without having to do the math every time or call the units class every time
    config
        .encoder
        .positionConversionFactor(2 * Math.PI / HoodConstants.kGearRatio) // No absolute encoder...
        .velocityConversionFactor(2 * Math.PI / HoodConstants.kGearRatio / 60.0);

    //sets the kS gain of the feedforward controller(anticipates motor movement to stop errors before they occur, the opposite of reactive)
    //The kS gain is the minimum voltage in volts required to initiate the motor, in this case 0
    config.closedLoop.feedForward.kS(0);

    //Attempts to configure the motor then set the encoder to 0 rotations for a maximum of 5 times
    //if this produces an error, it will try again until successful, or until it carries out the 5th try

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> encoder.setPosition(0));
  }

  @Override
  //Standard method to update the inputs from the IO layer
  public void updateInputs(HoodIOInputs inputs) {
    //initializes the spark sticky fault variable to false
    //this will change to true if any errors are detected in any methods which control the motor
    sparkStickyFault = false;
     //Only carries out the following 4 methods if a valid values is able to be read from the Spark Max
    //The first parameter tells the method what motor it is working with, in this case there is only 1 motor
    //The second parameter is a supplier (function which returns a value); in the case of the first two methods, this is position and velocity respectively
    //The third parameter is a consumer, which takes the value of the supplier defined previously and sets it equal to the position and velocity inputs from the IO interface respectively
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    //The first parameter is still the same motor
    //The second parameter is a double supplier which is also a list
    //the first value returns the active duty cycle (% voltage) being applied to the motor
    //the second value returns the actual amount of voltage in volts being applied to the motor from the battery
    //The third parameter takes both the values and multiplies them (% voltage * total voltage) and sets that equal to the applied voltage input
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    //The first parameter is still the same motor
    //The second parameter (supplier) returns the Spark Max's output current in amps
    //The third parameter (consumer) sets the current draw amps input and sets it equal to the second parameter
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentDrawAmps = value);
    //sets the connected input (boolean, whether or not the turret is connected) equal to the connected debouncer's interpretation of the sparkstickyfault signal
    //This uses the sparkstickyfault's detection of errors as criteria for the turret being connected
    //If the debouncer reads the value of the spark stick fault to be true, this means there is an error and the turret is not connected
    //If the debounder reads the value of the spark sticky fault to be false for more than a given amount of time (specified when the debounder is created) then the turret is connected, so this value is set to true
    //This is why there is a ! operator
    inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  //default set angle method as defined in the io interface
  public void setAngle(double angle) {
    //clamps the position of the motor between the maximum and minimum angles that the hood can move between
    double clampedPosition =
        MathUtil.clamp(angle, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    //sets the closed loop controller in the spark max to the clamped position defined in the line above
    motorController.setSetpoint(clampedPosition, ControlType.kPosition);
  }

  @Override
  //default set open loop method as defined in the io interface
  public void setOpenLoop(double output) {
    //clamps the speed output between the regular values of -1.0 and 1.0
    motor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  @Override
  //default method to stop the motor
  public void stop() {
    motor.stopMotor();
  }
}

//note: the structure of this hardware layer of the subsystem is VERY SIMILAR to that of the turret
//the only difference is that it does not contain an enum which is used in a universal apply outputs method
//instead, it has separate methods to set the angle of the hood using closed loop logic and to set the speed of the motor using open loop logic; it also requires a separate method to stop the motor
//this improves readability(possibly?) and is simpler, because the only output that is actually being applied is the target angle of the hood
