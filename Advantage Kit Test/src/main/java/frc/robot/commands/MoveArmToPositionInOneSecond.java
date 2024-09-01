// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.format.SignStyle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToPositionInOneSecond extends Command {
  private final Arm m_arm;
  private final double m_targetPosition;
  private final double sign;

  public MoveArmToPositionInOneSecond(Arm arm, double targetPosition) {
    m_arm = arm;
    m_targetPosition = targetPosition;
    if (targetPosition % 360 < m_arm.angle1 % 360){
      sign = -1;
    } else {
      sign = 1;
    }
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Executing command");
    if (!isFinished()) {
      m_arm.changeAngle1(sign);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double wristDifference = Math.abs(m_arm.angle1 - m_targetPosition) % 360;
    if (wristDifference == 0) { 
      return true;
    }
    return false;
  }
}
