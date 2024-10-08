// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.xml.namespace.QName;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.TargetCorner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GyroSim;
import frc.robot.SwerveModuleSim;
import frc.robot.util.Field.FieldSubzone;
import frc.robot.util.Field.FieldZone;
import frc.robot.util.Field.FieldZones;

public class DrivetrainSubsystemSim extends SubsystemBase implements Drivetrain {
  /** Creates a new DrivetrainSubsystem. */
  public final Field2d m_fieldSwerve = new Field2d();
  public final GyroSim m_gyro = new GyroSim();
  public final SwerveModuleSim m_frontLeftModule = new SwerveModuleSim();
  public final SwerveModuleSim m_frontRightModule = new SwerveModuleSim();
  public final SwerveModuleSim m_backLeftModule = new SwerveModuleSim();
  public final SwerveModuleSim m_backRightModule = new SwerveModuleSim();
  public FieldZones m_fieldZones = new FieldZones();
  public static FieldSubzone m_currentZone = new FieldSubzone("", 0, 0, 0, 0);
  public ChassisSpeeds m_speeds = new ChassisSpeeds(0, 0, 0);
  public Pose2d m_pose;
  public double m_x;
  public double m_y;
  public double m_rotationRadians; 
  public double fl_distance;
  public double fr_distance;
  public double bl_distance;
  public double br_distance; 
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 6;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 4;
  public SwerveModuleState frontLeft;
  public SwerveModuleState frontRight;
  public SwerveModuleState backLeft;
  public SwerveModuleState backRight;
  public Pose2d poseEstimated;
  public static final MechanicalConfiguration MK4I_L2_PLUS = new MechanicalConfiguration(
    0.10033,
    (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
    true,
    (16.0 / 50.0) * (10.0 / 60.0),
    false
);
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


   private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
  );

  public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics, new Rotation2d(m_rotationRadians),
      new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0))
      }, new Pose2d(0, 0, new Rotation2d()));

  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DriveIO m_io;

    private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          new Rotation2d((Math.toRadians(m_gyro.getAngle()))),
          new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d()),
      new SwerveModulePosition(0, new Rotation2d()),
      new SwerveModulePosition(0, new Rotation2d()),
      new SwerveModulePosition(0, new Rotation2d())
      },
          new Pose2d(),
          VecBuilder.fill(0, 0, 0),
          VecBuilder.fill(0, 0, 0));

  

  public DrivetrainSubsystemSim(DriveSimIO io) {
    m_io = io;
    //SmartDashboard.putData("Field Swerve", m_fieldSwerve);
    m_rotationRadians = 0;
    fl_distance = 0;
    fr_distance = 0;
    bl_distance = 0;
    br_distance = 0;
    m_x = 0;
    m_y = 0;

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5, 0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

               var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              } 
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

  @Override
  @AutoLogOutput
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public double getMaxVelocity() {
    return Constants.MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public Pose2d getEstimatedPose() {
    return poseEstimated;
  }

  @Override
  public void resetPose(Pose2d pose) {
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(m_speeds);

    m_gyro.update(pose.getRotation().getRadians());
    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
    double frontLeftDistance = m_frontLeftModule.getValue(frontLeft.speedMetersPerSecond);
    double frontRightDistance = m_frontRightModule.getValue(frontRight.speedMetersPerSecond);
    double backLeftDistance = m_backLeftModule.getValue(backLeft.speedMetersPerSecond);
    double backRightDistance = m_backRightModule.getValue(backRight.speedMetersPerSecond);
    
    m_rotationRadians = m_gyro.getGyroValueAdded(m_speeds.omegaRadiansPerSecond);
    // update gyro and distance
    m_odometry.resetPosition(pose.getRotation(),
    new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftDistance, frontLeft.angle),
      new SwerveModulePosition(frontRightDistance, frontRight.angle),
      new SwerveModulePosition(backLeftDistance, backLeft.angle),
      new SwerveModulePosition(backRightDistance, backRight.angle)
      }, pose);
  }


  public double netSpeed() {
    double x = m_speeds.vxMetersPerSecond * m_speeds.vxMetersPerSecond;
    double y = m_speeds.vyMetersPerSecond * m_speeds.vyMetersPerSecond;
    double squared  = x + y;
    return Math.sqrt(squared);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_speeds;
  }

  @Override 
  public void updateVision(Pose3d pose) {
    m_poseEstimator.addVisionMeasurement(pose.toPose2d(), Timer.getFPGATimestamp());
  }

  @Override
  public void drive(ChassisSpeeds speeds) {
    m_speeds = speeds;
  }

  @Override
  @AutoLogOutput
  public Rotation2d getGyroscopeRotation() {
    return new Rotation2d(m_gyro.getAngle());
  }

  @Override
  public void setDisabled() {
    m_speeds = new ChassisSpeeds(0, 0 , 0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("net", netSpeed());
    m_io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(m_speeds);

    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
    double frontLeftDistance = m_frontLeftModule.getValue(frontLeft.speedMetersPerSecond);
    double frontRightDistance = m_frontRightModule.getValue(frontRight.speedMetersPerSecond);
    double backLeftDistance = m_backLeftModule.getValue(backLeft.speedMetersPerSecond);
    double backRightDistance = m_backRightModule.getValue(backRight.speedMetersPerSecond);
    m_rotationRadians = m_gyro.getGyroValueAdded(m_speeds.omegaRadiansPerSecond);
    // update gyro and distance
    m_pose = m_odometry.update(new Rotation2d(m_rotationRadians),
    new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftDistance, frontLeft.angle),
      new SwerveModulePosition(frontRightDistance, frontRight.angle),
      new SwerveModulePosition(backLeftDistance, backLeft.angle),
      new SwerveModulePosition(backRightDistance, backRight.angle)
      });


      m_poseEstimator.update(new Rotation2d(m_rotationRadians), new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftDistance, frontLeft.angle),
      new SwerveModulePosition(frontRightDistance, frontRight.angle),
      new SwerveModulePosition(backLeftDistance, backLeft.angle),
      new SwerveModulePosition(backRightDistance, backRight.angle)
      });

    publisher.set(new SwerveModuleState[] {
      frontLeft,
      frontRight,
      backLeft,
      backRight
    });
    m_fieldSwerve.setRobotPose(m_pose); 
    poseEstimated = m_poseEstimator.getEstimatedPosition();
    Logger.recordOutput("estimated pose", poseEstimated);
    Rectangle.updateOrigin(getPose());
    updateFieldSection(new Point2D.Double(m_pose.getX(), m_pose.getY()));
  }

  public void updateFieldSection(Point2D point) {
    m_currentZone = m_fieldZones.getPointFieldZone(point);
    Logger.recordOutput("current zone", m_currentZone.getName());
  }

  @Override
  public int pointInBox(ArrayList<Pose3d> point) {
    for (int i = 0; i < point.size(); i++){
      Logger.recordOutput("point" + i, point.get(i));
      if (Rectangle.isPointInBox(point.get(i))){
        return i;
      }
    }
    return -1;
  }
}
