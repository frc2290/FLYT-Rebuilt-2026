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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.ShootOnTheFly;
import frc.utils.FieldConstants.Hub;
import frc.utils.ShootOnTheFly.SOTFResult;

public class Turret extends SubsystemBase {
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

    private double sotfYaw = 0.0;
    private boolean turretPointedAtTarget = false;

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


        SOTFResult result = sotf.calculateRecursiveTOF(targetTranslation, pose.get(), speeds.get());
        sotfYaw = result.yaw;
        //io.setTurnPosition(Rotation2d.fromDegrees(result.yaw).rotateBy(pose.get().getRotation().times(-1)));
        io.setTurnPosition(Rotation2d.fromDegrees(0));
        double turretCurPos = getTurretPos();
        Rotation2d turretPointedAt = Rotation2d.fromDegrees(turretCurPos).rotateBy(pose.get().getRotation());
        Rotation2d targetYaw = Rotation2d.fromDegrees(result.yaw);
        Rotation2d error = turretPointedAt.minus(targetYaw);
        if (Math.abs(error.getDegrees()) < 10) {
            turretPointedAtTarget = true;
        } else {
            turretPointedAtTarget = false;
        }
        // if (result.yaw - 5 < turretPointedAt && turretPointedAt < result.yaw + 5) {
        //     turretPointedAtTarget = true;
        // } else {
        //     turretPointedAtTarget = false;
        // }

        switch (shooterControlMode) {
            case VELOCITY:
                io.setShooterSpeed(result.vel * 1.375);
                break;
            case VOLTAGE:
                io.setShooterVoltage(shooterCommandedVoltage);
                break;
        }
        if (!stopShoot) {
            io.setShotAngle(result.pitch);
        } else {
            io.setHoodAngle(0);
        }
        Logger.recordOutput("Turret/SOTFYaw", result.yaw);
        Logger.recordOutput("Turret/SOTFVel", result.vel);
        Logger.recordOutput("Turret/SOTFPitch", result.pitch);
        Logger.recordOutput("Turret/SOTFTarget", targetTranslation);
        Logger.recordOutput("Turret/SOTFDist", result.dist);
        Logger.recordOutput("Turret/PointedAtHub", turretPointedAtTarget);
        Logger.recordOutput("Turret/turretPointedAt", error.getDegrees());
        Logger.recordOutput("Turret/ShooterControlMode", shooterControlMode.toString());
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
