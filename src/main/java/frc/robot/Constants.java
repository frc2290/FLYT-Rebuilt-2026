// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final boolean debugMode = false;

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 7.6;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 40;
        public static final int kRearLeftDrivingCanId = 42;
        public static final int kFrontRightDrivingCanId = 46;
        public static final int kRearRightDrivingCanId = 44;

        public static final int kFrontLeftTurningCanId = 41;
        public static final int kRearLeftTurningCanId = 43;
        public static final int kFrontRightTurningCanId = 47;
        public static final int kRearRightTurningCanId = 45;

        public static final boolean kGyroReversed = true;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 16;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRPM / 60;
        public static final double kWheelDiameterMeters = 0.0736;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

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

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRPM = 6734;
    }

    /**
     * Shared geometry describing the robot's vision sensors and AprilTag layout.
     */
    public static final class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = 0.9144;
        public static final double CAMERA_PITCH_RADIANS = degreesToRadians(45);
        public static final double TARGET_HEIGHT_METERS = 1.4351;

        /**
         * Physical location of the apriltag camera on the robot, relative to the center
         * of the robot.
         */
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(0.239191, -0.344616, 0.324842),
                new Rotation3d(0.0, 0, degreesToRadians(30)));

        public static final Transform3d APRILTAG_CAMERA2_TO_ROBOT = new Transform3d(
                new Translation3d(0.0015, -0.3279, (0.9473 - 0.102)),
                new Rotation3d(degreesToRadians(180), degreesToRadians(-45), degreesToRadians(-200)));

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
         *  - Left Trench: 6, 7
         *  - Tower: 15, 16
         *  - Outpost: 13, 14
         *  - Right Trench: 12, 1
         *  - Hub:
         *   - Front: 9, 10
         *   - Back: 3, 4
         *   - Left: 5, 8
         *   - Right: 11, 2
         * - Blue Alliance:
         *  - Left Trench: 22, 23
         *  - Tower: 31, 32
         *  - Outpost: 29, 30
         *  - Right Trench: 17, 28
         *  - Hub:
         *   - Front: 25, 26
         *   - Back: 19, 20
         *   - Left: 21, 24
         *   - Right: 18, 27 
         */

        public static final double inToM = 1 / 39.37;
        public final double mToIn = 39.37;

        public static final Pose3d hubCenterPose = new Pose3d(4.626, 4.035, 1.829, new Rotation3d(0, 0, 0));
        public static final Pose2d outpostPose = new Pose2d(0, 0.665988, new Rotation2d(0)); // human player station (change x pos)
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

        public static final double xTolerance = Units.inchesToMeters(1);
        public static final double xToleranceHasDistance = Units.inchesToMeters(11.5);
        public static final double yTolerance = Units.inchesToMeters(2);
        public static final double yToleranceHasDistance = Units.inchesToMeters(10);
        public static final double thetaTolerance = 2;
    }
}
