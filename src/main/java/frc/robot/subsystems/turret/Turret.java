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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.SOTFResult;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speeds;
    private boolean stopShoot = false;
    private ShootOnTheFly sotf = ShootOnTheFly.getInstance();

    /** Creates a new Turret. */
    public Turret(TurretIO turretIO, Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.io = turretIO;
        this.pose = pose;
        this.speeds = speeds;
        sotf.addShootSpeedInterpData(turretRPMData);
        sotf.addShootAngleInterpData(turretHoodData);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (!stopShoot) {
            SOTFResult result = sotf.calculate(VisionConstants.hubCenterPose.toPose2d().getTranslation(), pose.get(), speeds.get());
            Logger.recordOutput("SOTF Pitch", result.pitch);
            io.setHoodAngle(result.pitch);
            io.setTurnPosition(Rotation2d.fromDegrees(result.yaw));
            io.setShooterSpeed(result.vel);
        }
    }

    public void turnTurretToPose(Pose2d targetPose) {
        io.setTurnPosition(Rotation2d.fromDegrees(turnToTarget(targetPose.getTranslation(), pose.get().getTranslation())));
    }

    public void setStopShoot(boolean stop) {
        stopShoot = stop;
    }

    public Command shoot() {
        return runOnce(() -> io.shootFuel());
    }

    private double turnToTarget(Translation2d target, Translation2d current) {
        double offsetX = target.getX() - current.getX();
        double offsetY = target.getY() - current.getY();
        // return (360 - Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
        return Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360;
    }
}
