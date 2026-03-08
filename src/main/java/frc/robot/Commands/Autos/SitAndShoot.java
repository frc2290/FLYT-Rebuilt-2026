// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.robot.subsystems.StateMachines.DriveStateMachine.DriveState;
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitAndShoot extends FlytSequentialAuto {
    private StateMachine stateMachine;
    private PoseEstimatorSubsystem pose;
    private DriveStateMachine driveState;
    private boolean right = true;
    private Pose2d rightPose = new Pose2d(3.66, 0.72, Rotation2d.fromDegrees(90));
    private Pose2d leftPose = new Pose2d(3.6, 7.28, Rotation2d.fromDegrees(-90));

    /** Creates a new SitAndShoot. */
    public SitAndShoot(StateMachine stateMachine, PoseEstimatorSubsystem pose, DriveStateMachine driveState) {
        this.stateMachine = stateMachine;
        this.pose = pose;
        this.driveState = driveState;
    }

    @Override
    public void setup() {
        //addCommands(pose.setCurrentPoseCommand((this.right ? rightPose : leftPose)),
        //            Commands.runOnce(() -> pose.setTargetPose((this.right ? rightPose : leftPose))),
        //            this.stateMachine.setShooterOverrideCommand(true));
        addCommands(driveState.changeState(DriveState.CANCELLED),
                    new WaitCommand(2),
                    this.stateMachine.setShooterOverrideCommand(true));
    }

    @Override
    public void setRight(boolean right) {
        this.right = right;
    }
}
