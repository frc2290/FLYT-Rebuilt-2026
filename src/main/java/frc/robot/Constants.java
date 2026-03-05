// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // BEGIN advantagekit stuff
    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final double boundBuffer = inchesToMeters(15.0);
    public static final double baseBumpBuffer = inchesToMeters(0);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
    // END advantagekit stuff

    public static final boolean debugMode = false;

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * Shared geometry describing the robot's vision sensors and AprilTag layout.
     */
    public static final class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = 0.314325;
        public static final double CAMERA_PITCH_RADIANS = degreesToRadians(45);
        public static final double TARGET_HEIGHT_METERS = 1.4351;

        /**
         * Physical location of the apriltag camera on the robot, relative to the center
         * of the robot.
         */
        // -11in X -11in y
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(-0.2794, 0.2794, 0.5334),
                new Rotation3d(0.0, 0, degreesToRadians(90)));

        public static final Transform3d APRILTAG_CAMERA2_TO_ROBOT = new Transform3d(
                new Translation3d(0.0015, -0.3279, (0.9473 - 0.102)),
                new Rotation3d(degreesToRadians(180), degreesToRadians(-45), degreesToRadians(-200)));

        // Multi-camera AprilTag rig configuration (robot-to-camera transforms).
        public static final String kForwardCamName = "forward";
        public static final Transform3d kForwardCamTransform = new Transform3d(
                new Translation3d(0.24765, -0.1524, 0.5334),
                new Rotation3d(0.0, degreesToRadians(-42.0), 0.0));

        public static final String kBackwardCamName = "backward";
        public static final Transform3d kBackwardCamTransform = new Transform3d(
                new Translation3d(inchesToMeters(12.25), inchesToMeters(5.375), inchesToMeters(21)),
                new Rotation3d(0.0, degreesToRadians(-42.0), degreesToRadians(180.0)));

        public static final String kLeftCamName = "left";
        public static final Transform3d kLeftCamTransform = new Transform3d(
                new Translation3d(-0.3175, 0.2921, 0.523875),
                new Rotation3d(0.0, degreesToRadians(-10.0), degreesToRadians(90.0)));

        public static final String kRightCamName = "right";
        public static final Transform3d kRightCamTransform = new Transform3d(
                new Translation3d(-0.3175, -0.2921, 0.523875),
                new Rotation3d(0.0, degreesToRadians(-10.0), degreesToRadians(-90.0)));

        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS = 8.069;

        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose
        // to the opposite alliance
        public static final Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        /*
         * April Tag IDs:
         * - Red Alliance:
         * - Left Trench: 6, 7
         * - Tower: 15, 16
         * - Outpost: 13, 14
         * - Right Trench: 12, 1
         * - Hub:
         * - Front: 9, 10
         * - Back: 3, 4
         * - Left: 5, 8
         * - Right: 11, 2
         * - Blue Alliance:
         * - Left Trench: 22, 23
         * - Tower: 31, 32
         * - Outpost: 29, 30
         * - Right Trench: 17, 28
         * - Hub:
         * - Front: 25, 26
         * - Back: 19, 20
         * - Left: 21, 24
         * - Right: 18, 27
         */

        public static final double inToM = 1 / 39.37;
        public final double mToIn = 39.37;

        public static final Pose3d hubCenterPose = new Pose3d(4.626, 4.035, 1.829, new Rotation3d(0, 0, 0));
        public static final Pose2d outpostPose = new Pose2d(0, 0.665988, new Rotation2d(0)); // human player station
                                                                                             // (change x pos)
        public static final Pose2d depotPose = new Pose2d(0.6858, 5.963158, new Rotation2d(0));
        public static final Pose2d towerPose = new Pose2d(1.055624, 3.745484, new Rotation2d(0));

        // R2OC Red and Blue average Pose
        public static final List<Pose2d> leftBranches = List.of(
                new Pose2d(3.182, 4.192, new Rotation2d(Math.toRadians(-1.4))), // 18_LEFT
                new Pose2d(3.682, 2.984, new Rotation2d(Math.toRadians(58.0))), // 17_LEFT
                new Pose2d(4.994, 2.810, new Rotation2d(Math.toRadians(118.1))), // 22_LEFT
                new Pose2d(5.795, 3.849, new Rotation2d(Math.toRadians(178.1))), // 21_LEFT
                new Pose2d(5.287, 5.073, new Rotation2d(Math.toRadians(-121.5))), // 20_LEFT
                new Pose2d(3.991, 5.245, new Rotation2d(Math.toRadians(-62.1)))); // 19_LEFT
        public static final List<Pose2d> rightBranches = List.of(
                new Pose2d(3.181, 3.851, new Rotation2d(Math.toRadians(-0.8))), // 18_RIGHT
                new Pose2d(3.985, 2.808, new Rotation2d(Math.toRadians(59.1))), // 17_RIGHT
                new Pose2d(5.288, 2.977, new Rotation2d(Math.toRadians(118.7))), // 22_RIGHT
                new Pose2d(5.796, 4.206, new Rotation2d(Math.toRadians(179.7))), // 21_RIGHT
                new Pose2d(4.989, 5.246, new Rotation2d(Math.toRadians(-121.0))), // 20_RIGHT
                new Pose2d(3.691, 5.073, new Rotation2d(Math.toRadians(-61.4)))); // 19_RIGHT

        public static final Translation2d reefCenter = new Translation2d(176 * inToM, 158.5 * inToM);
        public static final Translation2d processor = new Translation2d(6, 0);

        /** Pose used when pointing the drivetrain toward the center of the reef. */
        public static final Pose2d REEF_CENTER_AIM_POSE = new Pose2d(reefCenter, new Rotation2d());

        /** Pose used when aiming the drivetrain at the processor. */
        public static final Pose2d PROCESSOR_AIM_POSE = new Pose2d(processor, new Rotation2d());

        public static final double halfwayAcrossFieldY = (317 / 2) * inToM;
        public static final double coralStationLeftHeading = -55;
        public static final double coralStationRightHeading = 55;

        // X value of the translation is irrelevant
        public static final Translation2d netScore = new Translation2d(295 * inToM, 295 * inToM);

        public static final int ATPipelineIndex = 0;

        // Vision fusion and std-dev tuning constants.
        public static final double kVisionMaxTimeSkewSeconds = 0.100;
        public static final double kVisionInvalidStdDev = 1000.0;
        public static final double kVisionRejectVarianceThreshold = 90000.0;
        public static final double kVisionRotationVarianceFusionCutoff = 1000.0;

        public static final double kVisionXyStdDevCoefficient = 0.01;
        public static final double kVisionThetaStdDevCoefficient = 0.03;
        public static final double kVisionCameraStdDevFactor = 1.0;
        public static final double kVisionStdDevDistanceExponent = 2.0;
        public static final double kVisionStdDevTagCountExponent = 2.0;
        public static final double kVisionStdDevMin = 0.01;

        public static final double kVisionFinalXyStdDevFloor = 0.05;
        public static final double kVisionFinalThetaStdDevFallback = 0.03;
        public static final double kVisionFinalThetaStdDevFloor = 0.03;

        // Reject vision frames with physically impossible field Z estimates.
        public static final double kVisionMaxPoseZMeters = 1.0;

        public static final double xTolerance = Units.inchesToMeters(1);
        public static final double xToleranceHasDistance = Units.inchesToMeters(11.5);
        public static final double yTolerance = Units.inchesToMeters(2);
        public static final double yToleranceHasDistance = Units.inchesToMeters(10);
        public static final double thetaTolerance = 2;
    }
}
