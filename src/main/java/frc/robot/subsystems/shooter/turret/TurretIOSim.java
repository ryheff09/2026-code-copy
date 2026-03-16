package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

public class TurretIOSim implements TurretIO {
  //standard gearbox object, creates a gearbox with the one motor in the turret
  private final DCMotor gearbox = DCMotor.getNEO(1);
  //standard simulated gearbox object
  private final DCMotorSim sim;

  //creates a pid controller with gains (these need to be tuned; see PID documentation for more details)
  private PIDController pid = new PIDController(2, 0, 0.3, Constants.kLoopPeriodSeconds);

  //standard applied volts variable to be implemented later
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    //resets the pid controller by clearing any previous errors that it was storing and the integral term
    pid.reset();
    //enables continuous movement for the pid (past 360 degrees and back to 0)
    //sets the minimum and maximum to radian values
    //these values are considered the same point and the pid calculates the shortest point to the target
    //ex: if the target is 1 degree and the turret is at 350, the turret will only move 11 degrees clockwise rather than 349 degrees counterclockwise
    //question: why set the actual turret not to continuous movement but the pid to continuous movement?
    pid.enableContinuousInput(-Math.PI, Math.PI);
    //default sim object
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, TurretConstants.kGearRatio),
            gearbox);
  }

  //standard default update input method
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    //Keeps the value of the voltage in an acceptable range (-12 to 12)
    double volts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    //Sets the voltage of the motor to the value of volts clamped above
    sim.setInputVoltage(volts);
    //updates the states of the dc motor simulation object every 0.02 seconds to be logged as data points
    sim.update(0.02);

    //updates all the input values using the simulation object
    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  //default apply outputs method
  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    //again, the switch, case, case keywords are a simpler version of conditional statements
    switch (outputs.mode) {
      //in the case that the mode is closed loop (which it is as of now), the pid setpoint is set to the specified target in radians
      //calculates the voltage which needs to be applied to the simulated gearbox object by getting the current position of the turret in radians
      case CLOSED_LOOP -> {
        pid.setSetpoint(outputs.closedLoopTarget.getRadians());
        appliedVolts = pid.calculate(sim.getAngularPositionRad());
      }
      //in the case that the mode is changed to open loop, the applied voltage is set to 12 times the speed of the motor (ex 0.5 speed is 50%, which is equivalent to 6 out of 12 volts from the batter)
      case OPEN_LOOP -> {
        appliedVolts = 12.0 * outputs.openLoopOutput;
      }
    }
  }
}
