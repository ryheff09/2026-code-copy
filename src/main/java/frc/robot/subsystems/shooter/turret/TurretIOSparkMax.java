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
  private final SparkClosedLoopController motorController; //creates an object for closed loop control, in this case pidf (Proportional Integral Derivative + feedforward)

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling); 
  //ensures that the turret is disconnected for 0.5 seconds before reading the signal as false; this eliminates noise or errors from triggering a full turret disconnect 
  //if the type was krising, this would apply from false to true, and the type can also be both. Other applications include button pressings or limit switches or any boolean signals.

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
    //automatically inverts the spark max motor if the turret side is on the right (the parameter is a boolean)
    config.inverted(side == ShooterSide.RIGHT);
    // .smartCurrentLimit(30);

    //automatically converts units from radians into degrees so that whenever 
    //you call the .getPosition method for the encoder, it returns in this unit,
    //without having to do the math every time or call the units class every time
    config
        .encoder
        .positionConversionFactor(
            2 * Math.PI / TurretConstants.kGearRatio) // No absolute encoder...
        .velocityConversionFactor(2 * Math.PI / TurretConstants.kGearRatio / 60.0);

    //sets position wrapping to false, meaning the turret cannot move continuously in a full circle, only back and forth
    //sets the motor encoder as the feedback sensor for the closed loop controller (pidf)
    config.closedLoop.positionWrappingEnabled(false).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    //sets a soft limit for the turret, which is the opposite of a hard limit such as a limit switch
    //It stops the turret from turning past certain limits on both sides (reverse and forward)
    //It also sets these limits in radians using the constants
    config
        .softLimit
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(TurretConstants.kMinTurretAngleRad)
        .forwardSoftLimit(TurretConstants.kMaxTurretAngleRad);

    //sets the kS gain of the feedforward controller(anticipates motor movement to stop errors before they occur, the opposite of reactive)
    //The kS gain is the minimum voltage in volts required to initiate the motor, in this case 0
    config.closedLoop.feedForward.kS(0);

    //Note: All of the above things are set using the motor config

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
  public void updateInputs(TurretIOInputs inputs) {
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
    //Question: why does it reverse the value of sparkstickyfault?
    inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  //apply outputs method defined in the IO interface
  public void applyOutputs(TurretIOOutputs outputs) {
    //The switch and case keywords are a cleaner version of if-elseif-else chains
    //it takes the value of the output mode (closed loop or open loop, set to closed loop))
    switch (outputs.mode) {
      //in the case of a closed loop, a clamped position variable is created which takes the target output, gets it in radians, and puts it within the minimum and maximum turret movement angles if it is not already
      case CLOSED_LOOP -> {
        double clampedPosition =
            MathUtil.clamp(
                outputs.closedLoopTarget.getRadians(),
                TurretConstants.kMinTurretAngleRad,
                TurretConstants.kMaxTurretAngleRad);

        //sets the closed loop controller in the spark max to the clamped position which was just created.
        motorController.setSetpoint(clampedPosition, ControlType.kPosition);
      }
      //in the case of an open loop, the speed of the motor is set to the openloopoutput output and clamps it between the minimum value of -1 and the maximum value of 1
      //Why even create an open loop case if closed loop is so much better?
      //Probably for safety, and this can easily be changed in one place, the turretIO outputs class
      case OPEN_LOOP -> {
        motor.set(MathUtil.clamp(outputs.openLoopOutput, -1.0, 1.0));
      }
    }
  }
}
