// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private boolean stopShoot = false;

    /** Creates a new Turret. */
    public Turret(TurretIO turretIO) {
        this.turretIO = turretIO;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        turretIO.updateInputs(turretInputs);
        Logger.processInputs("Turret", turretInputs);

        if (!stopShoot) {
            
        }
    }

    public void turnTurretToPose(Pose2d targetPose) {

    }

    public void setStopShoot(boolean stop) {
        stopShoot = stop;
    }

    public Command shoot() {
        return runOnce(() -> turretIO.shootFuel());
    }
}
