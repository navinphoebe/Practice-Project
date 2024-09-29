// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/** Add your docs here. */
public class Rectangle {

    private static Pose2d m_pose;
    private static double[] origin = new double[2];
    private static double m_width;
    private static double m_height;
    private static double m_rotation;
    private static double[] topLeftCorner = new double[2];
    private static double[] bottomRightCorner = new double[2];
    private static double armX;
    private static double armZ;

    public Rectangle(Pose2d pose) {
        m_pose = pose;
        origin[0] = m_pose.getX();
        origin[1] = m_pose.getY();
        m_rotation = m_pose.getRotation().getRadians();
        topLeftCorner[0] = -.5 * .4;
        bottomRightCorner[0] = .5 * .4;
        topLeftCorner[1] = .5;
        bottomRightCorner[1] = 0;
    }

    public static void updateOrigin(Pose2d pose) {
        origin[0] = pose.getX();
        origin[1] = pose.getY();
        m_rotation = pose.getRotation().getRadians();
    }

    public static void updateArm(Pose3d pose) {
        armX = pose.getX();
        armZ = pose.getZ();
    }

    public static boolean isPointInBox(Pose3d point) {
        double y = point.getX() - origin[0];
        double x = point.getY() - origin[1];
        double z = point.getZ();
        double[] newPoint = new double[2];
        newPoint[0] = x * Math.cos(m_rotation) - y * Math.sin(m_rotation);
        newPoint[1] = x * Math.sin(m_rotation) + y * Math.cos(m_rotation);
        Logger.recordOutput("failed point", newPoint);
        newPoint[1] -= armX;
        if ((bottomRightCorner[1] <= newPoint[1] && newPoint[1] <= topLeftCorner[1]) &&
                (topLeftCorner[0] <= newPoint[0] && newPoint[0] <= bottomRightCorner[0]) &&
                Math.abs(z - armZ) < .25) {
            Logger.recordOutput("Working point", newPoint);
            return true;
        }
        return false;
    }

}
