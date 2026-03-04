// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.SwerveAutoStep;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchToNeutralAuto extends SequentialCommandGroup {
    /** Creates a new TrenchToNeutralAuto. */
    public TrenchToNeutralAuto(PoseEstimatorSubsystem pose, Intake intake, boolean right) {
        try {
            PathPlannerPath trenchToNeutral = PathPlannerPath.fromPathFile("TrenchNeutralRight");
            // Add your commands in the addCommands() call, e.g.
            // addCommands(new FooCommand(), new BarCommand());
            addCommands(
                pose.setCurrentPoseCommand(trenchToNeutral.getStartingHolonomicPose().get()),
                new SwerveAutoStep(trenchToNeutral, pose),
                new ParallelCommandGroup(intake.driveIntake(), intake.intakeOut((right ? IntakeSide.LEFT : IntakeSide.RIGHT)))
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build Trench to Neutral auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
