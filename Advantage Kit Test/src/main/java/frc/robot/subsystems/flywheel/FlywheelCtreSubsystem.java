// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class FlywheelCtreSubsystem extends SubsystemBase implements FlywheelSubystem {

  public TalonFX m_motor;
  /** Creates a new FlywheelDcSubsystem. */
  public FlywheelCtreSubsystem() {
      m_motor = new TalonFX(0);
   
  }

  @Override 
  public void stop() {
    m_motor.stopMotor();
  }

  @Override 
  public void setVelocity(double velocityRPM) {
  var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
   m_motor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            0,
            0,
            false,
            false,
            false));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   ////Logger.recordOutput("Velocity RPM", m_motor.getVelocity().getValueAsDouble() / 60);
   //Logger.recordOutput("Voltage", m_motor.getSupplyVoltage().getValueAsDouble());
  }

  @Override
  public TalonFX getTalonFX() {
    return m_motor;
  }
}
