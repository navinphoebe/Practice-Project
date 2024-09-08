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


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new Arm. */
  private double angle1 = -45;
  private double angle2 = 70;
  double[] m_origin = new double[]{ -.26, 0, .2731};
  double[] elbowPlace = new double[]{0.15, 0, 0};
  double armLength = 0.6;
  public ShooterSubsystem m_shooter;
  public SimulateModel m_model = new SimulateModel(m_origin, armLength, elbowPlace);
  public Pose3d poseA = new Pose3d(-0.26, 0, 0.2731, new Rotation3d(Math.toRadians(180), Math.toRadians(angle1), Math.toRadians(0)));
  
  public Pose3d poseB = new Pose3d(0.15, 0, 0, new Rotation3d(Math.toRadians(0), Math.toRadians(angle2), Math.toRadians(-0)));

  public Mechanism2d mech = new Mechanism2d(100, 100);
  public MechanismLigament2d m_wrist;
  public MechanismLigament2d m_wrist2;
  public MechanismLigament2d m_wrist3;
  public MechanismLigament2d m_wrist4;
  public MechanismLigament2d m_wrist5;
  public MechanismRoot2d root;

  public ArmSubsystem(ShooterSubsystem shooter) {
    m_shooter = shooter;
    // angle2 = 100;
    // the main mechanism object
    // the mechanism root node
    root = mech.getRoot("climber", 75, 0);
    m_wrist = root.append(new MechanismLigament2d("base", 40, 180, 6, new Color8Bit(Color.kBlack)));
    m_wrist2 = m_wrist.append(new MechanismLigament2d("base1", 16, -90, 6, new Color8Bit(Color.kBlack)));
    m_wrist3 = m_wrist2.append(new MechanismLigament2d("wrist1", 40.0, -100, 6, new Color8Bit(Color.kRed)));
    m_wrist4 = m_wrist3.append(new MechanismLigament2d("wrist2", 8.0, 130, 6, new Color8Bit(Color.kGreen)));
    m_wrist5 = m_wrist3.append(new MechanismLigament2d("wrist3", 8.0, -50, 6, new Color8Bit(Color.kGreen)));
    SmartDashboard.putData("First Mechanism", mech);
  }


  @AutoLogOutput
  public Mechanism2d getMechanism() {
    return mech;
  }

  public double getAngle1() {
    return angle1;
  }

  public void changeAngle1(double num){
    angle1 += num;
    m_shooter.changeAngle2(num);
  }

  public void changeOnlyAngle1(double num) {
    angle1 += num;
  }

  public void setAngle1(double num) {
    double change = num - angle1;
    angle1 = num;
    m_shooter.changeAngle2(change);
  }

  public void setOnlyAngle1(double num) {
    angle1 = num;
  }

  @Override
  public void periodic() {
    angle2 = m_shooter.getAngle2();
    m_wrist3.setAngle(angle1);

    m_model.getJointDegrees(angle1, elbowPlace);

    Logger.recordOutput("Array", elbowPlace);
    Logger.recordOutput("Arm Degrees", angle1);
    Logger.recordOutput("Shooter Degrees", angle2);

    poseA = new Pose3d(-0.26, 0, 0.2731, new Rotation3d(Math.toRadians(180), Math.toRadians(angle1), Math.toRadians(0)));
    poseB = new Pose3d(elbowPlace[0], elbowPlace[1], elbowPlace[2], new Rotation3d(Math.toRadians(0), Math.toRadians(angle2), Math.toRadians(-0)));
    Logger.recordOutput("MyPoseArray", poseA, poseB);
  }
}