// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.frontLeftDriveCanId,
            DriveConstants.frontLeftTurnCanId,
            DriveConstants.frontLeftZeroRotation.getRadians());

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.frontRightDriveCanId,
            DriveConstants.frontRightTurnCanId,
            DriveConstants.frontRightZeroRotation.getRadians());

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.backLeftDriveCanId,
            DriveConstants.backLeftTurnCanId,
            DriveConstants.backLeftZeroRotation.getRadians());

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.backRightDriveCanId,
            DriveConstants.backRightTurnCanId,
            DriveConstants.backRightZeroRotation.getRadians());

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    /**
     * Scaling factor used when the robot is in "slow" mode for precision driving.
     */
    private double slowSpeed = 1.0;

    /** PID used for heading control when commands request a specific angle. */
    private PIDController rotPid = new PIDController(0.001, 0.0, 0.0); // 0.015 0 0

    /** PID used to correct X position errors during auto-alignment routines. */
    private PIDController xPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5

    /** PID used to correct Y position errors during auto-alignment routines. */
    private PIDController yPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.driveKinematics,
            Rotation2d.fromDegrees(getAngle()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        m_gyro.setAngleAdjustment(180);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.maxSpeedMetersPerSec;
        double ySpeedDelivered = ySpeed * DriveConstants.maxSpeedMetersPerSec;
        double rotDelivered = rot * DriveConstants.maxAngularSpeed;

        var swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.maxSpeedMetersPerSec);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.maxSpeedMetersPerSec);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        // return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
        return (m_gyro.getYaw() * (DriveConstants.gyroReversed ? -1.0 : 1.0));
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
    }

    public double getAngle() {
        return m_gyro.getAngle() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
    }

    public Rotation2d newHeading() {
        return m_gyro.getRotation2d();
        // return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
        return states;
    }

    public void setSlowSpeed() {
        slowSpeed = 0.5;
    }

    public void setRegularSpeed() {
        slowSpeed = 1;
    }

    public boolean isSlowSpeed() {
        return slowSpeed < 1;
    }

    public PIDController getRotPidController() {
        return rotPid;
    }

    public PIDController getXPidController() {
        return xPid;
    }

    public PIDController getYPidController() {
        return yPid;
    }
}
