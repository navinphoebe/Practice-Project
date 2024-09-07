// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Vision;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAdjustWithAprilTags extends Command {
  /** Creates a new AutoAdjustWithAprilTags. */
  private ShooterSubsystem m_shooter;
  private Vision m_vision;
  private double targetAngle;
  public AutoAdjustWithAprilTags(Vision vision, ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_vision = vision;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("init auto tag");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.hasGoalTarget()) {
      double ty = m_vision.getGoalDistance();
      targetAngle = m_shooter.getShooterAngleMapDown(ty);
      double sign = calculateNegativeOrPositive();
      m_shooter.changeAngle2(sign);
    }
  }

  public double calculateNegativeOrPositive() {
    double m_shooterTarget = targetAngle % 360;
    m_shooterTarget += 360;
    m_shooterTarget = m_shooterTarget % 360;

    double m_shooterAngle = m_shooter.getAngle2() % 360;
    m_shooterAngle += 360;
    m_shooterAngle = m_shooterAngle % 360;
    m_shooter.setAngle2(m_shooterAngle);
    boolean greater = m_shooterTarget - m_shooterAngle > 0 && m_shooterTarget - m_shooterAngle < 180;
    boolean equal = m_shooterTarget - m_shooterAngle == 0;
    if (greater ) {
      return 1;
    } else if (equal) {
      return 0;
    } else {
      return -1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((targetAngle - m_shooter.getAngle2()) % 360 == 0);
  }
}
