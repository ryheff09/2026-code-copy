package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

//default simulation class which implements the io layer
public class HoodIOSim implements HoodIO {
  //standard gearbox object, creates a gearbox with 1 motor in it, in this case a neo 550 (lighter and less powerful than a regular neo)
  private final DCMotor gearbox = DCMotor.getNeo550(1);
  //default simulation object initialization
  private final SingleJointedArmSim sim;
  
  //creates a pid controller with gains (these need to be tuned; see PID documentation for more details)
  //in this case, the ki and kd gains are 0, while the kp gain is one, so a fast response to error is a priority here
  private final PIDController pid = new PIDController(1.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);

  //standard applied volts variable to be implemented later
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    //pid controller is NOT reset, unlike turret
    //continuous movement is NOT enabled, unlike turret

    //creates the initialized sim object
    //this is a single jointed arm sim, which is exactly like it sounds; it simulates the gravity on an arm with only 1 pivot point
    //It takes the additional parameters of the length of the arm, the range of arm movement, whether or not to simulate gravity, and the arm's starting angle in radians
    sim =
        new SingleJointedArmSim(
            gearbox,
            HoodConstants.kGearRatio,
            0.025,
            Units.inchesToMeters(7),
            HoodConstants.kMinAngleRad,
            HoodConstants.kMaxAngleRad,
            true,
            0);
  }

  //standard default update inputs method
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    //clamps a value of volts between an acceptable range (-12.0 to 12.0) based on the fact that the battery is a 12 volt battery
    double volts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    //sets the input value of the simulated arm to the clamped voltage value
    sim.setInputVoltage(volts);
    //updates the states of the dc motor simulation object every 0.02 seconds to be logged as data points
    sim.update(0.02);

    //updates all the input values using the simulated arm object
    inputs.connected = true;
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = volts;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  //default set angle method
  public void setAngle(double angle) {
    //clamps the angle parameter in the acceptable range for hood movement
    angle = MathUtil.clamp(angle, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    //calculates the voltage to apply to the arm to get it to its target angle based off of where it is and where it wants to be
    //the turret uses the pid.setSetpoint method to set the setpoint of the pid controller, so that it doesn't have to pass the second parameter of a target
    //question: is there a difference between these two methods?
    appliedVolts = pid.calculate(sim.getAngleRads(), angle);
  }

  @Override
  //default set open loop method
  public void setOpenLoop(double output) {
    //sets the applied voltage equal to twelve times the output of speed (ex 0.5 speed is 50%, which is equivalent to 6 out of 12 volts from the battery)
    appliedVolts = 12.0 * output;
  }

  @Override
  //default stop motor method
  public void stop() {
    appliedVolts = 0.0;
  }
}
