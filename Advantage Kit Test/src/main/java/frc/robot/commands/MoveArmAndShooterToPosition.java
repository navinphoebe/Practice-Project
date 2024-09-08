// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.DrivetrainState;
import frc.robot.Vision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveArmAndShooterToPosition extends Command {
  /** Creates a new MoveArmAndShooterToPosition. */
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;
  private double m_armTarget;
  private double m_shooterTarget;
  private double m_armAngle;
  private double m_shooterAngle;
  private double armSign;
  private double shooterSign;
  private boolean tr;
  public MoveArmAndShooterToPosition(ShooterSubsystem shooter, ArmSubsystem arm, double armTarget, double shooterTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    tr = false;
    m_arm = arm;
    m_armTarget = armTarget;
    m_shooterTarget = shooterTarget;

    addRequirements(m_shooter, m_arm);
  }

  public MoveArmAndShooterToPosition(ShooterSubsystem shooter, ArmSubsystem arm, double armTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_arm = arm;
    tr = true;
    m_armTarget = armTarget;
    m_shooterTarget = m_shooter.getAngle2();
    addRequirements(m_arm);
  }

  public double calculateNegativeOrPositive(double angle, double target) {
    boolean greater = (target - angle < 180 && target - angle > 0) || (target - angle < -180);
    if (!greater) {
      return -1;
    } else {
      return 1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armAngle = m_arm.getAngle1() % 360;
    m_armAngle += 360;
    m_armAngle = m_armAngle % 360;
    m_arm.setOnlyAngle1(m_armAngle);

    m_armTarget = m_armTarget % 360;
    m_armTarget += 360;
    m_armTarget = m_armTarget % 360;

    m_shooterTarget = m_shooterTarget % 360;
    m_shooterTarget += 360;
    m_shooterTarget = m_shooterTarget % 360;

    m_shooterAngle = m_shooter.getAngle2() % 360;
    m_shooterAngle += 360;
    m_shooterAngle = m_shooterAngle % 360;
    armSign = calculateNegativeOrPositive(m_armAngle, m_armTarget);
    shooterSign = calculateNegativeOrPositive(m_shooterAngle, m_shooterTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_armTarget - m_arm.getAngle1()) % 360 != 0) {
      m_arm.changeOnlyAngle1(armSign);
    }
    if ((m_shooterTarget - m_shooter.getAngle2()) % 360 != 0 && tr == false) {
      m_shooter.changeAngle2(shooterSign);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs((m_armTarget - m_arm.getAngle1())) % 360 < 2) && (Math.abs((m_shooterTarget - m_shooter.getAngle2()) % 360) < 2);
  }
}
