// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem {
  public double MAX_VELOCITY_METERS_PER_SECOND = 0;
  
  public void drive(ChassisSpeeds speeds);

  public Rotation2d getGyroscopeRotation();

  public void resetPose(Pose2d pose);

  public void setDisabled();

  public double getMaxVelocity();

  public Pose2d getPose();

  public void updateVision(Pose3d robotPoseApril);

  public Pose2d getEstimatedPose();

  public int pointInBox(ArrayList<Pose3d> name);
}
