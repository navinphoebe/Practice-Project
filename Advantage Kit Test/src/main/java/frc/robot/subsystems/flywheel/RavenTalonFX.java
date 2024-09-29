// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class RavenTalonFX implements IRavenMotor {
    private TalonFX m_motor;

    public RavenTalonFX(int deviceId) {
        m_motor = new TalonFX(deviceId);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
        slot0Configs.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

        m_motor.getConfigurator().apply(slot0Configs);
    }

    public RavenTalonFX(int deviceId, String canbus) {
        m_motor = new TalonFX(deviceId, canbus);
    }

    // ------- AutoCloseable ----- //
    public void close() {
        m_motor.close();
    }

    @Override
    public void setVelocity(double velocityRPM) {
        var request = new VelocityVoltage(0).withSlot(0);
        m_motor.setControl(request.withVelocity(velocityRPM));
    }

    // ------ set/get routines for WPILIB interfaces ------//
    /**
     * Common interface for setting the speed of a motor controller.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void set(double speed) {
        m_motor.set(speed);
    }

    /**
     * Common interface for seting the direct voltage output of a motor controller.
     *
     * @param volts The voltage to output.
     */
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public double getVelocityRPM() {
       return 0;
    }

    public double getVoltage() {
        return 0;
    }

    /**
     * Common interface for getting the current set speed of a motor controller.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double get() {
        return m_motor.get();
    }

    // ---------Intercept CTRE calls for motor safety ---------//
    /*
     * protected StatusCode setControlPrivate(ControlRequest request) {
     * m_motor.setControlPrivate(request);
     * }
     */

    // ----------------------- turn-motor-off routines-------------------//
    /**
     * Common interface for disabling a motor controller.
     */
    public void disable() {
        m_motor.disable();
    }

    /**
     * Common interface to stop motor movement until set is called again.
     */
    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    // ----------------------- Invert routines -------------------//
    /**
     * Common interface for inverting direction of a motor controller.
     * <p>
     * Since invert is a config, this API is blocking. We recommend that
     * users avoid calling this API periodically.
     *
     * @param isInverted The state of inversion, true is inverted.
     */
    public void setInverted(boolean isInverted) {
        m_motor.setInverted(isInverted);
    }

    /**
     * Common interface for returning the inversion state of a motor controller.
     * <p>
     * Since invert is a config, this API is blocking. We recommend that
     * users avoid calling this API periodically.
     *
     * @return The state of the inversion, true is inverted.
     */

    public boolean getInverted() {
        return m_motor.getInverted();
    }

    // -------------------- Neutral mode routines ----------------//
    /**
     * Sets the mode of operation when output is neutral or disabled.
     * <p>
     * Since neutral mode is a config, this API is blocking. We recommend
     * that users avoid calling this API periodically.
     *
     * @param neutralMode The state of the motor controller bridge when output is
     *                    neutral or disabled
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        m_motor.setNeutralMode(neutralMode);
    }

    // ---- Sendable -------//

    public void initSendable(SendableBuilder builder) {
        m_motor.initSendable(builder);
    }

    /**
     * @return Description of motor controller
     */
    public String getDescription() {
        return m_motor.getDescription();
    }

    /**
     * Feed the motor safety object.
     * <p>
     * Resets the timer on this object that is used to do the timeouts.
     */
    public void feed() {
        m_motor.feed();
    }

    public void setExpiration(double expirationTime) {
        m_motor.setExpiration(expirationTime);
    }

    public double getExpiration() {
        return m_motor.getExpiration();
    }

    public boolean isAlive() {
        return m_motor.isAlive();
    }

    public void setSafetyEnabled(boolean enabled) {
        m_motor.setSafetyEnabled(enabled);
    }

    public boolean isSafetyEnabled() {
        return m_motor.isSafetyEnabled();
    }
}
