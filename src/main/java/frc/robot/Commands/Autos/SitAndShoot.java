// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.robot.subsystems.StateMachines.DriveStateMachine.DriveState;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.FieldConstants.LinesHorizontal;
import frc.utils.FieldConstants.LinesVertical;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitAndShoot extends FlytSequentialAuto {
    private PoseEstimatorSubsystem pose;
    private DriveStateMachine driveState;

    /** Creates a new SitAndShoot. */
    public SitAndShoot(PoseEstimatorSubsystem pose, DriveStateMachine driveState) {
        this.pose = pose;
        this.driveState = driveState;
    }

    @Override
    public void setup() {
        // directly in front of the hub
        addCommands(pose.setCurrentPoseCommand(new Pose2d(LinesVertical.starting - inchesToMeters(37.0 / 2.0), LinesHorizontal.center, Rotation2d.kZero)),
                    driveState.changeState(DriveState.CANCELLED));
    }
}
