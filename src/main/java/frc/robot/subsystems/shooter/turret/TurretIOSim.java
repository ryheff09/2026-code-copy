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

  //creates a pid controller
  private PIDController pid = new PIDController(2, 0, 0.3, Constants.kLoopPeriodSeconds);

  //standard applied volts variable to be implemented later
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    //resets the pid controller by clearing any previous errors and the integral term
    pid.reset();
    //enables continuous movement for the pid (past 360 degrees and back to 0)
    //sets the minimum and maximum to radian values
    //these values are considered the same point and the pid calculates the shortest point to the target
    //ex: if the target is 1 degree and the turret is at 350, the turret will only move 11 degrees clockwise rather than 349 degrees counterclockwise
    //question: why set the actual turret not to continuous movement but the pid to continuous movement?
    pid.enableContinuousInput(-Math.PI, Math.PI);
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, TurretConstants.kGearRatio),
            gearbox);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double volts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(volts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case CLOSED_LOOP -> {
        pid.setSetpoint(outputs.closedLoopTarget.getRadians());
        appliedVolts = pid.calculate(sim.getAngularPositionRad());
      }
      case OPEN_LOOP -> {
        appliedVolts = 12.0 * outputs.openLoopOutput;
      }
    }
  }
}
