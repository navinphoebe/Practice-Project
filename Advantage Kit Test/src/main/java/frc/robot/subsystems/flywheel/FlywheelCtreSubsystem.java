// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

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
    //m_motor.stop();
  }

  @Override 
  public void setVelocity(double velocityRPM) {
   // m_motor.setVelocity(velocityRPM);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ////Logger.recordOutput("Velocity RPM", m_motor.getVelocityRPM());
   // Logger.recordOutput("Voltage", m_motor.getVoltage());
  }
}
