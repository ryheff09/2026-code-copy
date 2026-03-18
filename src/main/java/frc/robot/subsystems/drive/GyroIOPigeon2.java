// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.DriveConstants.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  //creates a new pigeon2 (different type of gyro) with an id stored in constants
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
      //creates a status signal (timestamped data point) for the yaw of the gyro
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  //initializes queues for the yaw position and timestamps for these positions
  //these store values until they are ready for processing and provide ways to access these values
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  //creates another status signal for the yaw velocity of the gyro
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    //creates and applies a new config object for the pigeon if there is no existing object
    //if there is already a config, it applies the existing config
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    //sets the update frequency for the yaw angle and yaw velocity status signals
    yaw.setUpdateFrequency(DriveConstants.kOdometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);
    //optimizes the update frequencies for the status signals defined above
    pigeon.optimizeBusUtilization();
    //creates the initialized queue objects above
    //question: why clone the status signal yaw angle?
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  //default method to update the inputs defined in the io interface
  public void updateInputs(GyroIOInputs inputs) {
    //updates values of the status signals
    //if there are no errors with these values, the connected input is set to true
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    //gets the rotation2d value of the yaw angle status signal, then adds this to 180 degrees, or pi radians (this would be considered the starting point)
    //converts is value to degrees and sets that equal to the yaw position input
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble()).rotateBy(Rotation2d.kPi);
    //sets the value of the yaw velocity input to the yaw velocity status signal, converting the value to radians per second
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // inputs.odometryYawTimestamps =
    //     yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryYawPositions =
    //     yawPositionQueue.stream()
    //         .map((Double value) -> Rotation2d.fromDegrees(value))
    //         .toArray(Rotation2d[]::new);
    // yawTimestampQueue.clear();
    // yawPositionQueue.clear();
  }

  @Override
  //default method to actually set the yaw of the gyro to a given number of degrees; again uses 180 degrees or pi radians as a default starting point (why?)
  public void setYaw(Rotation2d angle) {
    pigeon.setYaw(angle.rotateBy(Rotation2d.kPi).getDegrees());
  }
}
