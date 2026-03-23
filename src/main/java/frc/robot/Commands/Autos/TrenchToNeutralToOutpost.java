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
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchToNeutralToOutpost extends FlytSequentialAuto {
    private boolean right = true;
    private PoseEstimatorSubsystem pose;
    private StateMachine stateMachine;
    private Intake intake;

    /** Creates a new TrenchToNeutralAuto. */
    public TrenchToNeutralToOutpost(PoseEstimatorSubsystem pose, StateMachine stateMachine, Intake intake) {
        this.pose = pose;
        this.stateMachine = stateMachine;
        this.intake = intake;
    }

    @Override
    public void setup() {
        try {
            PathPlannerPath trenchToNeutral = PathPlannerPath.fromPathFile("TrenchNeutralRight");
            PathPlannerPath neutralToTrench = PathPlannerPath.fromPathFile("NeutralTrenchRight");
            PathPlannerPath trenchToOutpost = PathPlannerPath.fromPathFile("TrenchRightOutpost");
            Pose2d startPose = trenchToNeutral.getStartingHolonomicPose().get();
            Logger.recordOutput("StartPose", startPose);
            if (!this.right) {
                trenchToNeutral = trenchToNeutral.mirrorPath();
                neutralToTrench = neutralToTrench.mirrorPath();
                trenchToOutpost = PathPlannerPath.fromPathFile("TrenchLeftDepot");
            }
            addCommands(
                //stateMachine.setShooterOverrideCommand(false),
                pose.setCurrentPoseCommand(trenchToNeutral.getStartingHolonomicPose().get()),
                new ParallelCommandGroup(
                    new SwerveAutoStep(trenchToNeutral, pose),
                    new WaitCommand(1.5).andThen(intake.intakeOut(right ? IntakeSide.RIGHT : IntakeSide.LEFT)).andThen(intake.startIntakeCommand())),
                new SwerveAutoStep(neutralToTrench, pose),
                //stateMachine.setShooterOverrideCommand(true),
                new WaitCommand(3.5),
                new ParallelCommandGroup(
                    new SwerveAutoStep(trenchToOutpost, pose),
                    intake.intakeOut((right ? IntakeSide.LEFT : IntakeSide.RIGHT)))//,
                //stateMachine.setShooterOverrideCommand(false)
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build Trench to Neutral auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }

    @Override
    public void setRight(boolean right) {
        this.right = right;
    }
}
