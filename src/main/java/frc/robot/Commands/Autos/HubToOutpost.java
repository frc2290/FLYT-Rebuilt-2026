// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.SwerveAutoStep;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HubToOutpost extends FlytSequentialAuto {
    private PoseEstimatorSubsystem pose;
    private Intake intake;
    private Pose2d startPose;

    private PathPlannerPath hubToOutpost;

    /** Creates a new HubToOutpost. */
    public HubToOutpost(PoseEstimatorSubsystem pose, Intake intake) {
        this.pose = pose;
        this.intake = intake;

        try {
            this.hubToOutpost = PathPlannerPath.fromPathFile("HubToOutpost");
            this.startPose = hubToOutpost.getStartingHolonomicPose().get();
            Logger.recordOutput("StartPose", startPose);
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build Hub to Outpost auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }

    @Override
    public void setup() {
        addCommands(
            pose.setCurrentPoseCommand(this.hubToOutpost.getStartingHolonomicPose().get()),
            new SwerveAutoStep(this.hubToOutpost, pose),
            intake.intakeOut(IntakeSide.LEFT),
            intake.startIntakeCommand());
    }

    @Override
    public Pose2d getPose() {
        return startPose;
    }
}
