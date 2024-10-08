// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Vision {
    VisionSystemSim visionSim;
    PhotonCameraSim cameraSim;
    private Pose3d robotPoseApril;
    private Pose2d robotPoseTrad;
    AprilTagFieldLayout tagLayout;
    Transform3d robotToCamera;
    Transform3d cameraToRobot;
    private Drivetrain m_drivetrain;
    
    public Vision(Drivetrain drivetrain) {
        if (Robot.isSimulation()){
        m_drivetrain = drivetrain;
        visionSim = new VisionSystemSim("Vision Sim Table");
        TargetModel targetModel = new TargetModel(0.5, 0.25);
        // The pose of where the target is on the field.
        // Its rotation determines where "forward" or the target x-axis points.
        // Let's say this target is flat against the far wall center, facing the blue driver stations.
        Pose3d targetPose = new Pose3d(0, 0, 2, new Rotation3d(0, 0, Math.PI));
        // The given target model at the given pose
        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
        // The layout of AprilTags which we want to add to the vision system
        try {
        tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e){
        
        }
        visionSim.addAprilTags(tagLayout);
        // The simulated camera properties
        System.out.println("failure");
        
        SimCameraProperties cameraProp = new SimCameraProperties();
        // The PhotonCamera used in the real robot code.
        PhotonCamera camera = new PhotonCamera("cameraName");

        // The simulation of this camera. Its values used in real robot code will be updated.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(80.3));
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        // Add this vision target to the vision system simulation to make it visible
        visionSim.addVisionTargets(visionTarget);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(-.311, .140, 0.298);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-20), Math.PI);
        robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
        cameraToRobot = robotToCamera.inverse();

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);

        // Get the built-in Field2d used by this VisionSystemSim
        visionSim.getDebugField();
    }
    }
 
    @AutoLogOutput
    public Pose3d getPoseApril() {
        return robotPoseApril;
    }

    @AutoLogOutput
    public int getAutoTag() {
        var result = cameraSim.getCamera().getLatestResult();
        if (result == null) {
        return 0;
        }
        return result.getBestTarget().getFiducialId();
        
    } 

    public void periodic() {
        if (Robot.isSimulation()){
        visionSim.update(m_drivetrain.getPose());
        var debugField = visionSim.getDebugField();
            debugField.getObject("EstimatedRobot").setPose(m_drivetrain.getPose());
        var result = cameraSim.getCamera().getLatestResult();
        if (result.hasTargets()){
        System.out.println("target!" + result.getBestTarget().getFiducialId());
        Logger.recordOutput("Target", result.getBestTarget().getFiducialId());
        var target = result.getBestTarget();
        Optional<Pose3d> aprilTagLayout = tagLayout.getTagPose(target.getFiducialId());
        Pose3d aprilLayout = new Pose3d();
        if (aprilTagLayout.isPresent()){
            aprilLayout = aprilTagLayout.get();
        }
        robotPoseApril = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilLayout, cameraToRobot);
         m_drivetrain.updateVision(robotPoseApril);
        } 
       
    }
    }

    public boolean hasGoalTarget() {
        var result = cameraSim.getCamera().getLatestResult();
        if (result.hasTargets()){
        var target = result.getBestTarget();
        double id = target.getFiducialId();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
        if (id == 7 || id == 8) {
            return true;
        } else {
            return false;
        }
    }
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        if (id == 3 || id == 4) {
            return true;
        } else {
            return false;
        }
    }
    }
    return false;
    }

    public double getAlignDegrees(boolean tr) {
        Pose2d pose = m_drivetrain.getEstimatedPose();
        if (pose != null) {
        if (Units.inchesToMeters(-1.5) - pose.getX() != 0) {
        return pose.getRotation().getDegrees() - Math.toDegrees(Math.atan((Units.inchesToMeters(218.42) - pose.getY())/(Units.inchesToMeters(-1.5) - pose.getX())));
        } 
    }
        return 0;
        
    }

    public double getAlignDegrees() {
            var result = cameraSim.getCamera().getLatestResult();
        

        if (result.hasTargets()) {
            int target = getGoalTarget(result.getTargets());
            if (target != -1) {
            var targetList = result.getTargets();
            PhotonTrackedTarget goal = targetList.get(target);
            double distanceMeters =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            .5,
                            Units.inchesToMeters(53.88),
                            Math.toRadians(15),
                            Units.degreesToRadians(goal.getPitch()));
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
        distanceMeters, Rotation2d.fromDegrees(-goal.getYaw()));
        System.out.println(-translation.getAngle().getDegrees());
        return -translation.getAngle().getDegrees();
        
        
                            
    }
}
    return 0;
    }

    private boolean isGoal(double id) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue && id == 7) {
            return true;
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && (id == 3 || id == 4)){
            return true;
        }
        return false;
    }

    public double getGoalDistance() {
        var result = cameraSim.getCamera().getLatestResult();
        

        if (result.hasTargets()) {
            int target = getGoalTarget(result.getTargets());
            if (target != -1) {
            var targetList = result.getTargets();
            PhotonTrackedTarget goal = targetList.get(target);
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            .5,
                            Units.inchesToMeters(55),
                            Math.toRadians(15),
                            Units.degreesToRadians(goal.getPitch()));
                            Logger.recordOutput("goal distance", range);
                  
            return range - .422;
        }
    }
    return 0;
}

public double getGoalDistance(boolean tr) {
    Pose2d pose = m_drivetrain.getEstimatedPose();
      if (pose != null) {
        return Math.pow(Units.inchesToMeters(218.42) - pose.getY(), 2) + Math.pow(Units.inchesToMeters(-1.5) - pose.getX(), 2) - .422;
        
    }
        return 0;

}

    private int getGoalTarget(List<PhotonTrackedTarget> targets) {
        Object[] array = targets.toArray();
        for (int x = 0; x < array.length; x++) {
            if (isGoal(targets.get(x).getFiducialId())) {
                return x;
            }
        }
        return -1;
    }
}
