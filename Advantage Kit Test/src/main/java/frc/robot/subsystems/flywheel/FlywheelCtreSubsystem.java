// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class FlywheelCtreSubsystem extends SubsystemBase implements FlywheelSubystem {

  public TalonFX m_motor;
  private double kGearRatio = 1.5;
  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.022);
  private final DCMotorSim m_motorSimModel =
    new DCMotorSim(DCMotor.getNEO(1), kGearRatio, 0.004);
  /** Creates a new FlywheelDcSubsystem. */
  public FlywheelCtreSubsystem() {
      m_motor = new TalonFX(0);
    var config = new Slot0Configs();
    config.kP = .5;
    config.kI = 0;
    config.kD = 0;
    m_motor.getConfigurator().apply(config);
  }

  @Override 
  public void stop() {
    m_motor.stopMotor();
  }

  @Override 
  public void setVelocity(double velocityRPM) {
  var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
  var ffVolts = ffModel.calculate(velocityRadPerSec);
  m_motor.setControl(
    new VelocityVoltage(
        Units.radiansToRotations(velocityRadPerSec),
        0.0,
        true,
        ffVolts,
        0,
        false,
        false,
        false));
        Logger.recordOutput("Set Velocity", velocityRadPerSec);
        Logger.recordOutput("Voltagee", m_motor.getMotorVoltage().getValueAsDouble());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  if (Robot.isSimulation()) {
      TalonFXSimState talonFXSim = m_motor.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      //talonFXSim.setSupplyVoltage(12);

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltage();
      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      m_motorSimModel.setInputVoltage(motorVoltage);
      m_motorSimModel.update(0.020); // assume 20 ms loop time

      // apply the new rotor position and velocity to the TalonFX;
      // note that this is rotor position/velocity (before gear ratio), but
      // DCMotorSim returns mechanism position/velocity (after gear ratio)
      talonFXSim.setRawRotorPosition(
          kGearRatio * m_motorSimModel.getAngularPositionRotations());
      talonFXSim.setRotorVelocity(
          kGearRatio * Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
    }
   Logger.recordOutput("Velocity RPM",  Units.radiansPerSecondToRotationsPerMinute(Units.rotationsToRadians(m_motor.getVelocity().getValueAsDouble()) / kGearRatio));
   Logger.recordOutput("Voltage", m_motor.getSupplyVoltage().getValueAsDouble());
  }

  @Override
  public TalonFX getTalonFX() {
    return m_motor;
  }
}
