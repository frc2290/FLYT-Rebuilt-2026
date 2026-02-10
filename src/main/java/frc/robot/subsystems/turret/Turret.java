// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.turretHoodData;
import static frc.robot.subsystems.turret.TurretConstants.turretRPMData;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.SOTFResult;

public class Turret extends SubsystemBase {
    private final TurretIO io; //input outs puts
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speeds;
    private boolean stopShoot = false;
    private ShootOnTheFly sotf = ShootOnTheFly.getInstance();
    private Pose2d targetPose = VisionConstants.hubCenterPose.toPose2d();

    public Turret(TurretIO turretIO, Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.io = turretIO;
        this.pose = pose;
        this.speeds = speeds;
        sotf.addShootInterpData(TurretConstants.SHOOTER_MAP);
        sotf.addShootSpeedInterpData(turretRPMData);
        sotf.addShootAngleInterpData(turretHoodData);
    }

    @Override
    public void periodic() {
        // turret logging
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (!stopShoot) {
            SOTFResult result = sotf.calculateRecursiveTOF(targetPose.getTranslation(), pose.get(), speeds.get());
            Logger.recordOutput("SOTF Pitch", result.pitch);
            io.setHoodAngle(result.pitch);
            io.setTurnPosition(Rotation2d.fromDegrees(result.yaw));
            io.setShooterSpeed(result.vel);
        }
    }

    /**
     * set the pose to point at for the turret
     * @param targetPose the pose to point at
     */
    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    /**
     * manually set the hood angle
     * @param angle angle in degrees to set the hood
     */
    public void setHoodAngle(double angle) {
        io.setHoodAngle(angle);
    }

    /**
     * manually set the turn position
     * @param rotation turn position as a rotation2d
     */
    public void setTurnPosition(Rotation2d rotation) {
        io.setTurnPosition(rotation);
    }

    /**
     * manually set the shooter speed
     * @param speed speed in rpm to set the shooter
     */
    public void setShooterSpeed(double speed) {
        io.setShooterSpeed(speed);
    }

    /**
     * stop/unstop shooting
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
}
