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
public class TrenchToNeutralAuto extends FlytSequentialAuto {
    private boolean right = true;
    private PoseEstimatorSubsystem pose;
    private StateMachine stateMachine;
    private Intake intake;
    private Pose2d startPose = new Pose2d();
    private PathPlannerPath trenchToNeutral;
    private PathPlannerPath neutralToTrench;
    private PathPlannerPath trenchToNeutral2;
    private PathPlannerPath neutralToTrench2;

    private PathPlannerPath trenchToNeutralFRONT;
    private PathPlannerPath neutralToTrenchFRONT;
    private PathPlannerPath trenchToNeutral2FRONT;
    private PathPlannerPath neutralToTrench2FRONT;
    private boolean forward = true;

    /** Creates a new TrenchToNeutralAuto. */
    public TrenchToNeutralAuto(PoseEstimatorSubsystem pose, StateMachine stateMachine, Intake intake) {
        this.pose = pose;
        this.stateMachine = stateMachine;
        this.intake = intake;

        try {
            this.trenchToNeutral = PathPlannerPath.fromPathFile("TrenchNeutralRight");
            this.neutralToTrench = PathPlannerPath.fromPathFile("NeutralTrenchRight");
            this.trenchToNeutral2 = PathPlannerPath.fromPathFile("TrenchNeutralRight2");
            this.neutralToTrench2 = PathPlannerPath.fromPathFile("NeutralTrenchRight2");
            this.trenchToNeutralFRONT = PathPlannerPath.fromPathFile("TrenchNeutralRightFront");
            this.neutralToTrenchFRONT = PathPlannerPath.fromPathFile("NeutralTrenchRightFront");
            this.trenchToNeutral2FRONT = PathPlannerPath.fromPathFile("TrenchNeutralRight2Front");
            this.neutralToTrench2FRONT = PathPlannerPath.fromPathFile("NeutralTrenchRight2Front");
            this.startPose = trenchToNeutral.getStartingHolonomicPose().get();
            Logger.recordOutput("StartPose", startPose);
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build Trench to Neutral auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }

    @Override
    public void setup() {
            if (this.forward) {
                this.trenchToNeutral = trenchToNeutralFRONT;
                this.neutralToTrench = neutralToTrenchFRONT;
                this.trenchToNeutral2 = trenchToNeutral2FRONT;
                this.neutralToTrench2 = neutralToTrench2FRONT;
            }
            if (!this.right) {
                this.trenchToNeutral = trenchToNeutral.mirrorPath();
                this.neutralToTrench = neutralToTrench.mirrorPath();
                this.trenchToNeutral2 = trenchToNeutral2.mirrorPath();
                this.neutralToTrench2 = neutralToTrench2.mirrorPath();
                this.startPose = trenchToNeutral.getStartingHolonomicPose().get();
            }
            addCommands(
                pose.setCurrentPoseCommand(this.trenchToNeutral.getStartingHolonomicPose().get()),
                new ParallelCommandGroup(
                    new SwerveAutoStep(this.trenchToNeutral, pose),
                    new WaitCommand(1.5).andThen(new ParallelCommandGroup(intake.intakeOut(whichSide()),
                                                                    intake.startIntakeCommand()))),
                new SwerveAutoStep(this.neutralToTrench, pose),
                new WaitCommand(3.5),
                new SwerveAutoStep(this.trenchToNeutral2, pose),
                new SwerveAutoStep(this.neutralToTrench2, pose),
                intake.agitateIntake(whichSide())
            );
    }

    private IntakeSide whichSide() {
        if (right && forward) {
            return IntakeSide.LEFT;
        } else if (right && !forward) {
            return IntakeSide.RIGHT;
        } else if(!right && forward) {
            return IntakeSide.RIGHT;
        } else if (!right && !forward) {
            return IntakeSide.LEFT;
        } else {
            return IntakeSide.RIGHT;
        }
    }

    @Override
    public void setRight(boolean right) {
        this.right = right;
    }

    @Override
    public Pose2d getPose() {
        return startPose;
    }

    @Override
    public void setForward(boolean forward) {
        this.forward = forward;
    }
}
