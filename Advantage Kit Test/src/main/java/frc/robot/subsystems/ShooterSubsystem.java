// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.SimulateModel;
import frc.robot.Vision;
import frc.robot.commands.MoveShooterToPosition;



public class ShooterSubsystem extends SubsystemBase {
  
  private double angle2 = 70;
  double[] m_origin = new double[]{ -.26, 0, .2731};

  public Pose3d pose = new Pose3d(0.15, 0, 10, new Rotation3d(Math.toRadians(0), Math.toRadians(angle2), Math.toRadians(-0)));
  private InterpolatingDoubleTreeMap shooterAngleMapDown = new InterpolatingDoubleTreeMap();
  private Vision m_vision;
  
  public ShooterSubsystem() {
    populateShooterAngleMapDown();
  }

  public void addVision(Vision vision) {
    m_vision = vision;
  }
  public void changeAngle2(double num){
    angle2 += num;
  }

  private void populateShooterAngleMapDown(){
    var array = ShooterConstants.SHOOTER_ANGLE_FROM__DOWN;
    for(int i = 0; i < array.length; i++){
        shooterAngleMapDown.put(array[i][0], array[i][1]);
    }
  }

  public void setAngle2(double num) {
    angle2 = num;
  }

  public void updatePose3d(Pose3d pose3d) {
    pose = pose3d;
  }

  public double getAngle2() {
    return angle2;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter Degrees", angle2);
  }

  public double getShooterAngleMapDown(double distance){
    double angle = shooterAngleMapDown.get(Units.metersToInches(distance));
    return -angle + 55;
}

public double getTargetAngle() {
   if (RobotContainer.PICKUP_STATE == RobotContainer.PickupState.HAS_NOTE) {
    if (RobotContainer.ARM_STATE == RobotContainer.ArmState.AMP_STATE) {
      return -40;
    } else {
      double ty = m_vision.getGoalDistance(true);
      return getShooterAngleMapDown(ty);
    }
   } 
  return 50;
  
}

}