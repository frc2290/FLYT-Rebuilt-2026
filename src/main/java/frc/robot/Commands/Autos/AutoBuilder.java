// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.SwerveAutoStep;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.robot.subsystems.StateMachines.DriveStateMachine.DriveState;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBuilder extends SequentialCommandGroup {
    /** Creates a new AutoBuilder. */
    public AutoBuilder(String start_pos, String end_pos, DriveStateMachine drive, PoseEstimatorSubsystem pose) {
        try {
            PathPlannerPath start_path = PathPlannerPath.fromPathFile(start_pos + " " + "Neutral" + " " + "Intake");
            PathPlannerPath end_path = PathPlannerPath.fromPathFile("Neutral" + " " + end_pos);

            // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            //     start_path = start_path.mirrorPath();
            //     end_path = end_path.mirrorPath();
            // }

            // Add your commands in the addCommands() call, e.g.
            // addCommands(new FooCommand(), new BarCommand());
            addCommands(
                pose.setCurrentPoseCommand(start_path.getStartingHolonomicPose().get()),
                new SwerveAutoStep(start_path, pose),
                new SwerveAutoStep(end_path, pose)
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
