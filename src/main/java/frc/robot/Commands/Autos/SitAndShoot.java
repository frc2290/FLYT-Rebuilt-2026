// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitAndShoot extends FlytSequentialAuto {
    private StateMachine stateMachine;
    private PoseEstimatorSubsystem pose;
    private boolean right = true;

    /** Creates a new SitAndShoot. */
    public SitAndShoot(StateMachine stateMachine, PoseEstimatorSubsystem pose) {
        this.stateMachine = stateMachine;
        this.pose = pose;
    }

    @Override
    public void setup() {
        addCommands(pose.setCurrentPoseCommand((this.right ? new Pose2d(3.8, 0.65, Rotation2d.fromDegrees(0)) : new Pose2d(3.8, 7.45, Rotation2d.fromDegrees(180)))), 
                    this.stateMachine.setShooterOverrideCommand(true));
    }

    @Override
    public void setRight(boolean right) {
        this.right = right;
    }
}
