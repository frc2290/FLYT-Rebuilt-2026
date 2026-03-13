// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.turret.TurretConstants.turretHoodData;
import static frc.robot.subsystems.turret.TurretConstants.turretRPMData;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.FullShooterParams;
import frc.utils.FieldConstants.Hub;
import frc.utils.ShootOnTheFly.SOTFResult;

public class Turret extends SubsystemBase {
    private static final double shooterVelocityScale = 1.45;

    public enum ControlMode {
        VELOCITY,
        VOLTAGE
    }

    private final TurretIO io; // input outs puts
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speeds;
    private boolean stopShoot = false;
    private ControlMode shooterControlMode = ControlMode.VELOCITY;
    private double shooterCommandedVoltage = 0.0;
    private ShootOnTheFly sotf = ShootOnTheFly.getInstance();
    private Translation2d targetTranslation = Hub.topCenterPoint.toTranslation2d();
    private boolean sotfEnabled = true;

    private double sotfYaw = 0.0;
    private boolean turretPointedAtTarget = false;
    private double previousLoopTimestampSec = Timer.getFPGATimestamp();

    public Turret(TurretIO turretIO, Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.io = turretIO;
        this.pose = pose;
        this.speeds = speeds;
        sotf.addShootInterpData(TurretConstants.SHOOTER_MAP);
        sotf.addShootSpeedInterpData(turretRPMData);
        sotf.addShootAngleInterpData(turretHoodData);

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.4).per(Second),
                        Volts.of(6.0),
                        Seconds.of(30),
                        (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> shooterCommandedVoltage = voltage.in(Volts),
                        null,
                        this));
    }

    @Override
    public void periodic() {
        // turret logging
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Pose2d currentPose = pose.get();
        ChassisSpeeds currentSpeeds = speeds.get();
        Rotation2d currentTurretAngle = Rotation2d.fromDegrees(getTurretPos());

        double timestampSec = Timer.getFPGATimestamp();
        double dt = timestampSec - previousLoopTimestampSec;
        previousLoopTimestampSec = timestampSec;
        if (dt <= TurretConstants.SotfConstants.minLoopDtSeconds || dt > TurretConstants.SotfConstants.maxLoopDtSeconds) {
            dt = TurretConstants.SotfConstants.defaultLoopDtSeconds;
        }

        double turretOmegaRadPerSecond = Math.toRadians(inputs.turretVelocity);
        SOTFResult result = sotf.calculateNewtonTOF(
                targetTranslation,
                currentPose,
                currentSpeeds,
                currentTurretAngle,
                turretOmegaRadPerSecond,
                dt);
        if (result.isValid) {
            sotfYaw = result.yaw;
        }

        Rotation2d turretPointedAt = currentTurretAngle.rotateBy(currentPose.getRotation());
        Rotation2d targetYaw = Rotation2d.fromDegrees(result.yaw);
        Rotation2d error = turretPointedAt.minus(targetYaw);
        turretPointedAtTarget = result.isValid
                && Math.abs(error.getDegrees()) < TurretConstants.SotfConstants.pointAtTargetToleranceDeg;

        if (sotfEnabled && result.isValid) {
            Rotation2d turretTargetRelative = targetYaw.minus(currentPose.getRotation());
            io.setTurnPosition(turretTargetRelative);
        }

        double activeShooterVelocitySetpointMps = result.isValid ? result.vel * shooterVelocityScale : 0.0;
        double activeShotAngleSetpointDeg = result.isValid ? result.pitch : 0.0;

        if (sotfEnabled) {
            boolean canShoot = !stopShoot && result.isValid;
            switch (shooterControlMode) {
                case VELOCITY:
                    io.setShooterSpeed(canShoot ? activeShooterVelocitySetpointMps : 0.0);
                    break;
                case VOLTAGE:
                    io.setShooterVoltage(canShoot ? shooterCommandedVoltage : 0.0);
                    break;
            }
            if (canShoot) {
                io.setShotAngle(activeShotAngleSetpointDeg);
            } else {
                io.setHoodAngle(0);
            }
        }
        Logger.recordOutput("Turret/SOTFYaw", result.yaw);
        Logger.recordOutput("Turret/SOTFVel", result.vel);
        Logger.recordOutput("Turret/SOTFPitch", result.pitch);
        Logger.recordOutput("Turret/SOTFTarget", targetTranslation);
        Logger.recordOutput("Turret/SOTFDist", result.dist);
        Logger.recordOutput("Turret/SOTFValid", result.isValid);
        Logger.recordOutput("Turret/SOTFLoopDtSec", dt);
        Logger.recordOutput("Turret/SOTFEnabled", sotfEnabled);
        Logger.recordOutput("Turret/PointedAtHub", turretPointedAtTarget);
        Logger.recordOutput("Turret/turretPointedAt", error.getDegrees());
        Logger.recordOutput("Turret/ShooterControlMode", shooterControlMode.toString());
        Logger.recordOutput("Turret/FlywheelVelocitySetpointMps", activeShooterVelocitySetpointMps);
        Logger.recordOutput("Turret/ShotAngleSetpointDeg", activeShotAngleSetpointDeg);
    }

    /**
     * set the translation to point at for the turret
     * 
     * @param targetPose the translation to point at
     */
    public void setTargetTranslation(Translation2d targetTranslation) {
        this.targetTranslation = targetTranslation;
    }

    /**
     * manually set the hood angle
     * 
     * @param angle angle in degrees to set the hood
     */
    public void setHoodAngle(double angle) {
        io.setHoodAngle(angle);
    }

    /**
     * manually set the turn position
     * 
     * @param rotation turn position as a rotation2d
     */
    public void setTurnPosition(Rotation2d rotation) {
        io.setTurnPosition(rotation);
    }

    /**
     * manually set the shooter speed
     * 
     * @param speed speed in m/s to set the shooter
     */
    public void setShooterSpeed(double speed) {
        setShooterControlMode(ControlMode.VELOCITY);
        io.setShooterSpeed(speed);
    }

    /** Applies fixed shooter/hood setpoints directly. */
    public void runManualShot(double flywheelVelocityMps, double hoodAngleDeg) {
        setStopShoot(false);
        if (shooterControlMode != ControlMode.VELOCITY) {
            setShooterControlMode(ControlMode.VELOCITY);
        }
        io.setShooterSpeed(flywheelVelocityMps);
        io.setShotAngle(hoodAngleDeg);
    }

    /** Command factory to run a fixed manual shot. */
    public Command manualShotCommand(double flywheelVelocityMps, double hoodAngleDeg) {
        return run(() -> runManualShot(flywheelVelocityMps, hoodAngleDeg))
                .until(() -> manualShotReady(hoodAngleDeg));
    }

    /** Enables/disables SOTF setpoint generation in periodic(). */
    public void setSotfEnabled(boolean enabled) {
        sotfEnabled = enabled;
    }

    public boolean isSotfEnabled() {
        return sotfEnabled;
    }

    /** True when the measured hood angle is within tolerance of the requested shot angle. */
    public boolean hoodAtShotSetpoint(double hoodAngleDeg) {
        if (!inputs.hoodConnected) {
            return true;
        }
        double measuredShotAngleDeg = TurretConstants.hoodShotAngleOffset - inputs.hoodPositionDeg;
        return Math.abs(measuredShotAngleDeg - hoodAngleDeg) <= TurretConstants.hoodShotAngleToleranceDeg;
    }

    /** Backwards-compatible alias for manual-shot checks. */
    public boolean hoodAtManualShotSetpoint(double hoodAngleDeg) {
        return hoodAtShotSetpoint(hoodAngleDeg);
    }

    /** True when both flywheel and hood have reached manual-shot readiness. */
    public boolean manualShotReady(double hoodAngleDeg) {
        return flywheelAtSpeed() && hoodAtShotSetpoint(hoodAngleDeg);
    }

    /** Current measured flywheel linear velocity (m/s). */
    public double getFlywheelVelocityMetersPerSecond() {
        return inputs.flywheelVelocity;
    }

    /** Returns the nominal shot speed command used by this turret for a given distance. */
    public double getShotVelocityForDistanceMeters(double distanceMeters) {
        FullShooterParams params = TurretConstants.SHOOTER_MAP.get(distanceMeters);
        if (params == null) {
            return 0.0;
        }
        return params.speedMetersPerSecond() * shooterVelocityScale;
    }

    /** Returns the nominal shot hood-angle command for a given distance. */
    public double getShotAngleForDistanceMeters(double distanceMeters) {
        FullShooterParams params = TurretConstants.SHOOTER_MAP.get(distanceMeters);
        if (params == null) {
            return 0.0;
        }
        return params.hoodAngle();
    }

    /** Get the current flywheel control mode. */
    public ControlMode getShooterControlMode() {
        return shooterControlMode;
    }

    /** Set the flywheel control mode. */
    public void setShooterControlMode(ControlMode controlMode) {
        shooterControlMode = controlMode == null ? ControlMode.VELOCITY : controlMode;
        if (shooterControlMode == ControlMode.VELOCITY) {
            shooterCommandedVoltage = 0.0;
            io.setShooterVoltage(0.0);
        }
    }

    /** Alias getter for flywheel control mode. */
    public ControlMode getFlywheelControlMode() {
        return getShooterControlMode();
    }

    /** Alias setter for flywheel control mode. */
    public void setFlywheelControlMode(ControlMode controlMode) {
        setShooterControlMode(controlMode);
    }

    /** Runs a quasistatic SysId test on the flywheels in the specified direction. */
    public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setShooterControlMode(ControlMode.VOLTAGE);
            shooterCommandedVoltage = 0.0;
        }).andThen(sysId.quasistatic(direction)).finallyDo(() -> {
            setShooterControlMode(ControlMode.VELOCITY);
            shooterCommandedVoltage = 0.0;
        });
    }

    /** Runs a dynamic SysId test on the flywheels in the specified direction. */
    public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setShooterControlMode(ControlMode.VOLTAGE);
            shooterCommandedVoltage = 0.0;
        }).andThen(sysId.dynamic(direction)).finallyDo(() -> {
            setShooterControlMode(ControlMode.VELOCITY);
            shooterCommandedVoltage = 0.0;
        });
    }

    /**
     * stop/unstop shooting
     * 
     * @param stop stopped?
     */
    public void setStopShoot(boolean stop) {
        stopShoot = stop;
    }

    public boolean isStopShootEnabled() {
        return stopShoot;
    }

    /**
     * shoot command for sim
     */
    public Command shoot() {
        return runOnce(() -> io.shootFuel());
    }

    public boolean flywheelAtSpeed() {
        return io.flywheelAtSpeed();
    }

    public double getSotfYaw() {
        return sotfYaw;
    }

    public double getTurretPos() {
        return inputs.turretPosition;
    }

    public boolean isTurretPointedAtTarget() {
        return turretPointedAtTarget;
    }
}
