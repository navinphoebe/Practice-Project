// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SimulateModel {
    public double[] m_origin = new double[3]; //(x,y,z)
    public double m_armLength;
    public double[] m_jointArea = new double[3]; // (x,y,z)
    public SimulateModel(double[] origin, double armLength, double[] jointArea) {
        m_origin = origin;
        m_armLength = armLength;
        m_jointArea = jointArea;
    }

    public void getJointDegrees(double degrees, double[] jointArea) {
        double x = m_origin[0] + m_armLength * Math.cos(Math.toRadians(degrees * -1));
        double z = m_origin[2] + m_armLength * Math.sin(Math.toRadians(degrees * -1));
        jointArea[0] = x;
        jointArea[2] = z;
    }
}
