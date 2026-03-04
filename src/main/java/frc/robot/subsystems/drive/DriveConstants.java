// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Sole source of truth for drivetrain hardware, geometry, and control constants. */
public final class DriveConstants {
  private DriveConstants() {}

  public enum MAXSwerveRatio {
    LOW(12.0, 22.0), // 5.50:1
    MEDIUM(13.0, 22.0), // 5.08:1
    HIGH(14.0, 22.0), // 4.71:1
    EXTRA_HIGH_1(15.0, 22.0), // 4.40:1
    EXTRA_HIGH_2(16.0, 22.0), // 4.125:1
    EXTRA_HIGH_3(16.0, 21.0), // 3.94:1
    EXTRA_HIGH_4(16.0, 20.0), // 3.75:1
    EXTRA_HIGH_5(16.0, 19.0); // 3.56:1

    private final double pinion;
    private final double spur;

    MAXSwerveRatio(double pinion, double spur) {
      this.pinion = pinion;
      this.spur = spur;
    }

    public double getReduction() {
      // (BevelDriven * SpurDriven) / (PinionDriving * BevelDriving)
      return (45.0 * spur) / (pinion * 15.0);
    }
  }

  public static final boolean gyroReversed = true;
  // Fixed yaw offset to align NavX zero with field-forward for autos.
  public static final double gyroYawOffsetDeg = 180.0;

  public static final double maxAngularSpeed = 4 * Math.PI;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(20.5); // Y-axis (width between wheels)
  public static final double wheelBase = Units.inchesToMeters(27.0); // X-axis (length between wheels)
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  // WPILib Coordinate System: X is forward/backward, Y is left/right
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // Front Left (+X, +Y)
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right (+X, -Y)
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back Left (-X, +Y)
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // Back Right (-X, -Y)
      };
  public static final SwerveDriveKinematics driveKinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2.0);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 46;
  public static final int backLeftDriveCanId = 40;
  public static final int frontRightDriveCanId = 44;
  public static final int backRightDriveCanId = 42;

  public static final int frontLeftTurnCanId = 47;
  public static final int backLeftTurnCanId = 41;
  public static final int frontRightTurnCanId = 45;
  public static final int backRightTurnCanId = 43;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = 0.0736 / 2.0;
  public static final double wheelDiameterMeters = wheelRadiusMeters * 2.0;
  // Select your active MAXSwerve ratio here.
  public static final MAXSwerveRatio activeDriveRatio = MAXSwerveRatio.EXTRA_HIGH_5;
  public static final double driveMotorReduction = activeDriveRatio.getReduction();
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
  public static final double driveFreeSpeedRPM =
      Units.radiansPerSecondToRotationsPerMinute(driveGearbox.freeSpeedRadPerSec);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec
  public static final double maxSpeedMetersPerSec =
      driveFreeSpeedRPM * driveEncoderVelocityFactor * wheelRadiusMeters;

  // Drive PID configuration
  public static final double driveKp = 0.017;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 12.0 / (driveFreeSpeedRPM * driveEncoderVelocityFactor);
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 12.0 / (driveGearbox.freeSpeedRadPerSec / driveMotorReduction);

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
