// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class MoveShooter extends Command {
  private final Shooter m_shooter;
  private final double m_targetPosition;
  private final double sign;

  public MoveShooter(Shooter shooter, double targetPosition) {
    m_shooter = shooter;
    m_targetPosition = targetPosition % 360;

    // Calculate the sign based on the current angle and target position
    double currentAngle = m_shooter.getAngle2() % 360;
    sign = Math.copySign(1, m_targetPosition - currentAngle);

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Executing command");
    if (!isFinished()) {
      m_shooter.changeAngle2(sign);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    double currentAngle = m_shooter.getAngle2() % 360;
    double wristDifference = Math.abs(currentAngle - m_targetPosition);
    if (wristDifference == 0) {
      return true;
    }
    return false;
  }
}