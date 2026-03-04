// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.SwerveAutoStep;
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchToNeutralAuto extends SequentialCommandGroup {
    /** Creates a new TrenchToNeutralAuto. */
    public TrenchToNeutralAuto(PoseEstimatorSubsystem pose, StateMachine stateMachine, Intake intake, boolean right) {
        try {
            PathPlannerPath trenchToNeutral = PathPlannerPath.fromPathFile("TrenchNeutralRight");
            PathPlannerPath neutralToTrench = PathPlannerPath.fromPathFile("NeutralTrenchRight");
            PathPlannerPath trenchToNeutral2 = PathPlannerPath.fromPathFile("TrenchNeutralRight2");
            PathPlannerPath neutralToTrench2 = PathPlannerPath.fromPathFile("NeutralTrenchRight2");
            Pose2d startPose = trenchToNeutral.getStartingHolonomicPose().get();
            Logger.recordOutput("StartPose", startPose);
            // if (!right) {
            //     trenchToNeutral = trenchToNeutral.mirrorPath();
            //     neutralToTrench = neutralToTrench.mirrorPath();
            //     trenchToNeutral2 = trenchToNeutral2.mirrorPath();
            //     neutralToTrench2 = neutralToTrench2.mirrorPath();
            // }
            addCommands(
                //stateMachine.setShooterOverrideCommand(false),
                pose.setCurrentPoseCommand(trenchToNeutral.getStartingHolonomicPose().get()),
                new ParallelCommandGroup(
                    new SwerveAutoStep(trenchToNeutral, pose),
                    new WaitCommand(1.5).andThen(intake.intakeOut(right ? IntakeSide.RIGHT : IntakeSide.LEFT)).andThen(intake.startIntakeCommand())),
                new SwerveAutoStep(neutralToTrench, pose),
                stateMachine.setShooterOverrideCommand(true),
                new WaitCommand(3.5),
                stateMachine.setShooterOverrideCommand(false),
                new SwerveAutoStep(trenchToNeutral2, pose),
                new SwerveAutoStep(neutralToTrench2, pose),
                stateMachine.setShooterOverrideCommand(true),
                new WaitCommand(3.5),
                stateMachine.setShooterOverrideCommand(false)
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build Trench to Neutral auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
