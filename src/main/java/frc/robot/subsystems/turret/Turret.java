// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.FullShooterParams;
import frc.robot.Robot;
import frc.utils.FieldConstants.Hub;
import frc.utils.ShootOnTheFly.SOTFResult;
import frc.utils.ShootOnTheFly.TargetTable;

public class Turret extends SubsystemBase {
    private static final String shooterVelocityScaleKey = "Turret_Cal/Speed_Scale";
    private static final String shotAngleOffsetDegKey = "Turret_Cal/Angle_Offset_Deg";
    private static final double defaultShooterVelocityScale = 1.0;
    private static final double defaultShotAngleOffsetDeg = 0.0;

    public enum ControlMode {
        NORMAL, // Velocity for shooter, position for turn
        SHOOTER_VOLTAGE,
        TURN_VOLTAGE
    }

    private final TurretIO io; // input outs puts
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine shooterSysId;
    private final SysIdRoutine turnSysId;
    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speeds;
    private boolean stopShoot = false;
    private ControlMode currentControlMode = ControlMode.NORMAL;
    private double sysIdVoltage = 0.0;
    private ShootOnTheFly sotf = ShootOnTheFly.getInstance();
    private Translation2d targetTranslation = Hub.topCenterPoint.toTranslation2d();
    private boolean sotfEnabled = true;

    private double driveAngleCorrection = 0.0;
    private boolean turretPointedAtTarget = false;
    private double currentTof = 0.0;
    private double previousLoopTimestampSec = Timer.getFPGATimestamp();
    private double activeShooterVelocitySetpointMps = 0.0;
    private double activeShotAngleSetpointDeg = 0.0;
    private double currentShooterVelocityScale = defaultShooterVelocityScale;
    private double currentShotAngleOffsetDeg = defaultShotAngleOffsetDeg;

    public Turret(TurretIO turretIO, Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.io = turretIO;
        this.pose = pose;
        this.speeds = speeds;
        sotf.addShootInterpData(TurretConstants.HUB_MAP, TargetTable.HUB);
        sotf.addShootInterpData(TurretConstants.SHUTTLE_MAP, TargetTable.SHUTTLE);

        shooterSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.4).per(Second),
                        Volts.of(6.0),
                        Seconds.of(30),
                        (state) -> Logger.recordOutput("Turret/ShooterSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> sysIdVoltage = voltage.in(Volts),
                        null,
                        this));

        turnSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(2.5 / 30.0).per(Second),
                        Volts.of(2.5),
                        Seconds.of(30),
                        (state) -> Logger.recordOutput("Turret/TurnSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> sysIdVoltage = voltage.in(Volts),
                        null,
                        this));

        SmartDashboard.putNumber(shooterVelocityScaleKey, defaultShooterVelocityScale);
        SmartDashboard.putNumber(shotAngleOffsetDegKey, defaultShotAngleOffsetDeg);
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

        Rotation2d turretPointedAt = currentTurretAngle.rotateBy(currentPose.getRotation());
        Rotation2d targetYaw = Rotation2d.fromDegrees(result.yaw);
        Rotation2d error = turretPointedAt.minus(targetYaw);
        turretPointedAtTarget = result.isValid
                && Math.abs(error.getDegrees()) < TurretConstants.SotfConstants.pointAtTargetToleranceDeg;
        if (result.isValid) {
            driveAngleCorrection = targetYaw.minus(turretPointedAt).getDegrees();
            currentTof = result.tof;
        } else {
            currentTof = 0.0;
        }

        currentShooterVelocityScale = SmartDashboard.getNumber(shooterVelocityScaleKey, defaultShooterVelocityScale);
        currentShotAngleOffsetDeg = SmartDashboard.getNumber(shotAngleOffsetDegKey, defaultShotAngleOffsetDeg);

        if (result.isValid) {
            double tunedShotVelocityMps = result.vel * currentShooterVelocityScale;
            double tunedShotAngleDeg = result.pitch + currentShotAngleOffsetDeg;

            // Convert desired SOTF projectile behavior into mechanism commands using
            // inverse linear-fit calibration.
            activeShooterVelocitySetpointMps = (tunedShotVelocityMps - TurretConstants.flywheelSpeedCalibrationOffset)
                    / TurretConstants.flywheelSpeedCalibrationGain;
            activeShotAngleSetpointDeg = (tunedShotAngleDeg - TurretConstants.hoodPitchCalibrationOffset)
                    / TurretConstants.hoodPitchCalibrationGain;
        }



        // TURN CONTROL
        if (currentControlMode == ControlMode.TURN_VOLTAGE) {
            //io.setTurnVoltage(sysIdVoltage);
        } else if (sotfEnabled && result.isValid) {
            Rotation2d turretTargetRelative = targetYaw.minus(currentPose.getRotation());
            //io.setTurnPosition(turretTargetRelative);
        }

        // SHOOTER CONTROL
        if (currentControlMode == ControlMode.SHOOTER_VOLTAGE) {
            io.setShooterVoltage(sysIdVoltage);
        } else if (sotfEnabled) {
            boolean canShoot = !stopShoot && result.isValid;
            io.setShooterSpeed(activeShooterVelocitySetpointMps);
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
        Logger.recordOutput("Turret/TOFSeconds", currentTof);
        //Logger.recordOutput("Turret/SOTFYawVelocityRadPerSec", result.yawVelocityRadPerSec);
        //Logger.recordOutput("Turret/SOTFYawAccelerationRadPerSec2", result.yawAccelerationRadPerSec2);
        //Logger.recordOutput("Turret/SOTFPitchVelocityDegPerSec", result.pitchVelocityDegPerSec);
        //Logger.recordOutput("Turret/SOTFFlywheelAccelerationMps2", result.flywheelAccelerationMetersPerSec2);
        Logger.recordOutput("Turret/SOTFValid", result.isValid);
        Logger.recordOutput("Turret/SOTFLoopDtSec", dt);
        Logger.recordOutput("Turret/SOTFEnabled", sotfEnabled);
        Logger.recordOutput("Turret/PointedAtHub", turretPointedAtTarget);
        //Logger.recordOutput("Turret/turretPointedAt", error.getDegrees());
        Logger.recordOutput("Turret/ControlMode", currentControlMode.toString());
        Logger.recordOutput("Turret/SysIdVoltage", sysIdVoltage);
        Logger.recordOutput("Turret/FlywheelVelocitySetpointMps", activeShooterVelocitySetpointMps);
        Logger.recordOutput("Turret/ShotAngleSetpointDeg", activeShotAngleSetpointDeg);
        Logger.recordOutput("Turret/CalibratedSpeedScale", currentShooterVelocityScale);
        Logger.recordOutput("Turret/CalibratedShotAngleOffsetDeg", currentShotAngleOffsetDeg);
        Logger.recordOutput("Turret/CurrentTOFTable", sotf.getCurrentTofTable());

        Robot.batteryLogger.reportCurrentUsage("Turret/Turn", inputs.turretConnected ? inputs.turretCurrentAmps : 0.0, inputs.turretAppliedVolts);
        Robot.batteryLogger.reportCurrentUsage("Turret/Hood", inputs.hoodConnected ? inputs.hoodCurrentAmps : 0.0, inputs.hoodAppliedVolts);
        Robot.batteryLogger.reportCurrentUsage("Turret/Flywheel", inputs.flywheelConnected ? inputs.flywheelCurrentAmps : 0.0, inputs.flywheelAppliedVolts);
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
        angle = MathUtil.clamp(angle, 0, 30);
        io.setHoodAngle(angle);
    }

    public double getHoodAngle() {
        return io.getHoodAngle();
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
        if (currentControlMode != ControlMode.NORMAL) {
            setControlMode(ControlMode.NORMAL);
        }
        io.setShooterSpeed(speed);
    }

    /** Applies fixed shooter/hood setpoints directly. */
    public void runManualShot(double flywheelVelocityMps, double hoodAngleDeg) {
        setStopShoot(false);
        if (currentControlMode != ControlMode.NORMAL) {
            setControlMode(ControlMode.NORMAL);
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

    public double getTof() {
        return currentTof;
    }

    /** True when the measured hood angle is within tolerance of the requested shot angle. */
    public boolean hoodAtShotSetpoint(double hoodAngleDeg) {
        if (!inputs.hoodConnected) {
            return true;
        }
        double measuredHoodAngleDeg = inputs.hoodPositionDeg;
        return Math.abs(measuredHoodAngleDeg - hoodAngleDeg) <= TurretConstants.hoodShotAngleToleranceDeg;
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

    /** Returns the raw, uncalibrated shot speed command for a given distance. */
    public double getShotVelocityForDistanceMeters(double distanceMeters) {
        FullShooterParams params = TurretConstants.HUB_MAP.get(distanceMeters);
        if (params == null) {
            return 0.0;
        }
        return params.speedMetersPerSecond();
    }

    /** Returns the nominal shot hood-angle command for a given distance. */
    public double getShotAngleForDistanceMeters(double distanceMeters) {
        FullShooterParams params = TurretConstants.HUB_MAP.get(distanceMeters);
        if (params == null) {
            return 0.0;
        }
        return params.hoodAngle();
    }

    /** Get the current turret control mode. */
    public ControlMode getControlMode() {
        return currentControlMode;
    }

    /** Set the turret control mode. */
    public void setControlMode(ControlMode mode) {
        currentControlMode = mode == null ? ControlMode.NORMAL : mode;
        if (currentControlMode == ControlMode.NORMAL) {
            sysIdVoltage = 0.0;
            io.setShooterVoltage(0.0);
            io.setTurnVoltage(0.0);
        }
    }

    /** Backwards-compatible alias for flywheel-specific mode checks. */
    public ControlMode getShooterControlMode() {
        return currentControlMode;
    }

    /** Backwards-compatible alias for flywheel-specific mode control. */
    public void setShooterControlMode(ControlMode controlMode) {
        setControlMode(controlMode == ControlMode.SHOOTER_VOLTAGE ? ControlMode.SHOOTER_VOLTAGE : ControlMode.NORMAL);
    }

    /** Backwards-compatible alias getter for flywheel mode. */
    public ControlMode getFlywheelControlMode() {
        return getShooterControlMode();
    }

    /** Backwards-compatible alias setter for flywheel mode. */
    public void setFlywheelControlMode(ControlMode controlMode) {
        setShooterControlMode(controlMode);
    }

    /** Runs a quasistatic SysId test on the flywheels in the specified direction. */
    public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(ControlMode.SHOOTER_VOLTAGE);
            sysIdVoltage = 0.0;
        }).andThen(shooterSysId.quasistatic(direction)).finallyDo(() -> {
            setControlMode(ControlMode.NORMAL);
        });
    }

    /** Runs a dynamic SysId test on the flywheels in the specified direction. */
    public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(ControlMode.SHOOTER_VOLTAGE);
            sysIdVoltage = 0.0;
        }).andThen(shooterSysId.dynamic(direction)).finallyDo(() -> {
            setControlMode(ControlMode.NORMAL);
        });
    }

    /** Runs a quasistatic SysId test on turret azimuth in the specified direction. */
    public Command sysIdQuasistaticTurn(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(ControlMode.TURN_VOLTAGE);
            sysIdVoltage = 0.0;
        }).andThen(turnSysId.quasistatic(direction)).finallyDo(() -> {
            setControlMode(ControlMode.NORMAL);
        });
    }

    /** Runs a dynamic SysId test on turret azimuth in the specified direction. */
    public Command sysIdDynamicTurn(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(ControlMode.TURN_VOLTAGE);
            sysIdVoltage = 0.0;
        }).andThen(turnSysId.dynamic(direction)).finallyDo(() -> {
            setControlMode(ControlMode.NORMAL);
        });
    }

    /**
     * Sweeps the hood back and forth using a sinusoidal wave for characterization
     * and tuning.
     * Operates at 1Hz between a safe minimum and maximum angle.
     */
    public Command characterizeHood() {
        Timer agitateTimer = new Timer();

        double minAngle = 5.0;
        double maxAngle = 50.0;
        double frequencyHz = 1.0;
        double DurationSec = 10.0 / frequencyHz;

        return startRun(
                () -> {
                    agitateTimer.restart();
                    setSotfEnabled(false);
                },
                () -> {
                    double wave = 0.5 + 0.5 * Math.cos(2 * Math.PI * frequencyHz * agitateTimer.get());
                    double targetAngle = minAngle + (wave * (maxAngle - minAngle));
                    setHoodAngle(targetAngle);
                    Logger.recordOutput("Turret/HoodCharacterizeTargetAngle", targetAngle);
                }).until(() -> agitateTimer.hasElapsed(DurationSec)).finallyDo(() -> {
                    agitateTimer.stop();
                    setHoodAngle(0.0);
                    setSotfEnabled(true);
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

    public double getDriveAngleCorrection() {
        return driveAngleCorrection;
    }

    public double getTurretPos() {
        return inputs.turretPosition;
    }

    public boolean isTurretPointedAtTarget() {
        return turretPointedAtTarget;
    }

    public boolean turretReadyToShoot() {
        return isTurretPointedAtTarget() && flywheelAtSpeed();
    }

    public Command resetTurretPosition() {
        return Commands.runOnce(() -> io.setTurretHomed(false));
    }
}
