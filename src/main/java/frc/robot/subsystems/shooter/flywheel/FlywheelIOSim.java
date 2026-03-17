package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  //standard gearbox object: creats a gearbox with 1 Krakenx44 motor
  private final DCMotor gearbox = DCMotor.getKrakenX44(1);
  //default simulation object initialization
  private final DCMotorSim sim;

  //creates a pid controller object with specified gains
  //the gain values show that reaching the flywheel velocity quickly is a priority, rather than accuracy or response to transient errors
  //again, this is because a flywheel doesn't have to worry about position, only velocity
  private final PIDController pid = new PIDController(1, 0, 0, Constants.kLoopPeriodSeconds);

  //standard applied volts variable to be implemented later
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    //pid controller is NOT reset, unlike turret
    //continuous movement is NOT enabled, unlike turret

    //creates the sim object initialized above
    //parameters: 
    //the gearbox it simulates (in this case the 1 gearbox initialized above)
    //its JKgMetersSquared, or moment of inertia (the measure of an object's resistance to changes in its speed of rotation) question: how is this value determined? 
    //the gear ratio of the overall gearbox (in this case it is set to 300, probably based on motor specs)
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, FlywheelConstants.kGearRatio),
            gearbox);
  }

  @Override
  //standard default update inputs method
  public void updateInputs(FlywheelIOInputs inputs) {
    //sets an output variable equal to the pid's calcualted voltage based on the simulated velocity 
    double currentOutput = pid.calculate(sim.getAngularVelocityRPM());
    //clamps the output variable between -12.0 and 12.0, based on the 12 volt battery limit, and sets this to a new applied voltage variable
    appliedVolts = MathUtil.clamp(currentOutput, -12.0, 12.0);

    //sets the voltage of the simulated gearbox to the applied voltage clamped above
    sim.setInputVoltage(appliedVolts);
    //updates the voltage value every 0.02 seconds
    sim.update(0.02);

    //updates all input values using the simulated gearbox object
    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  //default method to set the setpoint, or target, velocity for the flywheel
  @Override
  public void setVelocity(double velocity) {
    pid.setSetpoint(velocity);
  }

  //default method to set the voltage of the motor (multiplies the speed * 12, ex 0.5 speed = 50% speed = 6 out of 12 volts from the battery)
  @Override
  public void setOpenLoop(double output) {
    appliedVolts = 12.0 * output;
  }

  //default method to stop the motor by setting its voltage to 0
  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
