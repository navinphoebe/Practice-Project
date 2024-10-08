// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PickupState;
import frc.robot.subsystems.DrivetrainSubsystemSim;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double shotSpeed = 9.0; // Meters per sec
  private static final double ejectSpeed = 2.0; // Meters per sec
  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private static Supplier<Pose2d> robotPoseSupplier;
  private static Supplier<Rotation2d> armAngleSupplier;
  private static Supplier<Pose3d> armPoseSupplier;
  public static final List<Translation2d> autoNotes = new ArrayList<>();
  private static boolean hasNote = false;
  

  public static void setPoseSuppliers(Supplier<Pose2d> pose1, Supplier<Rotation2d> pose2, Supplier<Pose3d> armPose) {
      robotPoseSupplier = pose1;
      armAngleSupplier = pose2;
      armPoseSupplier = armPose;
  }

  /** Show all staged notes for alliance */
  public static void showAutoNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void clearAutoNotes() {
    autoNotes.clear();
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetAutoNotes() {
    clearAutoNotes();
    for (int i = FieldConstants.StagingLocations.spikeTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[i]));
    }
    for (int i = FieldConstants.StagingLocations.centerlineTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(
          AllianceFlipUtil.apply(FieldConstants.StagingLocations.centerlineTranslations[i]));
    }
  }

  /**
   * Take note from staged note
   *
   * @param note Number of note starting with 0 - 2 being spike notes going from amp to source side
   *     <br>
   *     and 3 - 7 being centerline notes going from amp to source side.
   */
  public static void takeAutoNote(int note) {
    if (autoNotes.get(note) != null) {
    autoNotes.set(note, null);
    hasNote = true;
    }
  }

  /** Shows the currently held note if there is one */
  public static void showHeldNotes() {
    if (hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {getIndexerPose3d()});
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {});
    }
  }

  /** Shoots note from middle of arm to speaker */
  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  if (!hasNote()) {
                    return Commands.run(() -> {
                      System.out.println("shoot not hasNote");
                    }).until(() -> true);
                
                  }  
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  final Pose3d endPose =
                      new Pose3d(
                          AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening),
                          startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
              }
              

  public static Command eject() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  System.out.println(getIndexerPose3d().getZ());
                  final Pose3d endPose =
                      startPose.transformBy(
                          new Transform3d(2, 0, -1 + startPose.getZ(), new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/EjectNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  private static Pose3d getIndexerPose3d() {
    Transform3d indexerTransform =
        new Transform3d(armPoseSupplier.get().getX() + .075, armPoseSupplier.get().getY(), armPoseSupplier.get().getZ() - .09,  new Rotation3d(0.0, armAngleSupplier.get().getRadians() + Math.toRadians(-15), 0.0));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }

public static boolean hasNote() {
    return hasNote;
}
}