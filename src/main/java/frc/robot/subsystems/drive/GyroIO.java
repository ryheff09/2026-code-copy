// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * This interface contains the class which contains the inputs to be logged as
 * data for the gyro, the default method to update these inputs, and a default
 * method to set the yaw (turning around the y axis, or the z axis of the WPILib
 * coordinate system) of a swerve module.
 */
public interface GyroIO {

  /**
   * This class contains the following inputs to be logged as data: whether or not
   * the gyroscope is connected, the yaw position, the yaw velocity, and the
   * timestamps and positions for the robot odemetry (estimating the robot's
   * position on the field, which uses the gyroscope)
   */
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }

  public default void setYaw(Rotation2d angle) {
  }
}
