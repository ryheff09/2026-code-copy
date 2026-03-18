// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  //default method to update the inputs to be stored as data
  public default void updateInputs(CameraIOInputs inputs) {}

  @AutoLog
  public static class CameraIOInputs {
    //initializes connected to false
    public boolean connected = false;
    //initializes camera position relative to the field to default zero value
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    //initializes robot pose (position on the field)
    public PoseObservation[] poseObservations = new PoseObservation[0];
    //initializes a list to contain april tag ids
    public int[] tagIds = new int[0];
  }

  //stores data about the camera's rotation on the x and y axis (WPIlib's coordinate system)
  //the record keyword ensures that this cannot be changed and automatically generates code to get anything stored within it (ex. TargetObservation.tx)
  /** Represents the angle to a target. Not used for pose estimation */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  //creates a record for the robot pose
  //stores the time the pose was recorded, the actual pose in 3 dimensions (takes both translation3d, a point in (x, y, z), and rotation3d (pitch, yaw, roll), the robot's rotation, into account)
  //also stores information about the april tags as well as the method of pose estimation (references enum below)
  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }
}
