// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SimulateModel;


public class Shooter extends SubsystemBase {
  /** Creates a new Arm. */
  public double angle1 = -45;
  public double angle2 = 70;
  double[] m_origin = new double[]{ -.26, 0, .2731};
  double[] elbowPlace = new double[]{0.15, 0, 0.8};

  public Pose3d pose = new Pose3d(0.15, 0, .8, new Rotation3d(Math.toRadians(0), Math.toRadians(angle2), Math.toRadians(-0)));
  
  public Shooter() {
  }

  public void changeAngle2(double num){
    angle2 += num;
  }

  public void updatePose3d(Pose3d pose3d) {
    pose = pose3d;
  }

  public double getAngle2() {
    return angle2;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Array", elbowPlace);
    Logger.recordOutput("Shooter Degrees", angle2);
  }
}