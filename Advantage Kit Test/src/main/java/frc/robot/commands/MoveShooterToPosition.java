// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterToPosition extends Command {
  /** Creates a new MoveArmAndShooterToPosition. */
  private ShooterSubsystem m_shooter;
  private double m_shooterTarget;
  private double shooterSign;
  private double m_shooterAngle;
  public MoveShooterToPosition(ShooterSubsystem shooter, double shooterTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_shooterTarget = shooterTarget;

    addRequirements(m_shooter);
  }

  public double calculateNegativeOrPositive(double angle, double target) {
    boolean greater = target - angle > 0 && target - angle < 180;
    if (greater) {
      return 1;
    } else {
      return -1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterTarget = m_shooterTarget % 360;
    m_shooterTarget += 360;
    m_shooterTarget = m_shooterTarget % 360;

    m_shooterAngle = m_shooter.getAngle2() % 360;
    m_shooterAngle += 360;
    m_shooterAngle = m_shooterAngle % 360;
    m_shooter.setAngle2(m_shooterAngle);
    shooterSign = calculateNegativeOrPositive(m_shooterAngle, m_shooterTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_shooterTarget - m_shooter.getAngle2()) % 360 != 0) {
      m_shooter.changeAngle2(shooterSign);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_shooterTarget - m_shooter.getAngle2()) % 360 == 0);
  }
}
