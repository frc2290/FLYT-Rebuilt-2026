// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.SwerveAutoStep;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.utils.FieldConstants;
import frc.utils.LocalADStarAK;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.FieldConstants.Depot;
import frc.utils.FieldConstants.LinesHorizontal;
import frc.utils.FieldConstants.LinesVertical;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBuilder extends SequentialCommandGroup {
    // trench, collect, outpost
    public enum AutoStart {
        TrenchLeft(new Pose2d()),
        TrenchRight(new Pose2d(LinesVertical.allianceZone, LinesHorizontal.rightTrenchMiddle, new Rotation2d())),
        BumpLeft(new Pose2d()),
        BumpRight(new Pose2d());

        private Pose2d pose;

        AutoStart(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public enum AutoActivity {
        Collect(new Rotation2d(), new Pose2d(1, 1, new Rotation2d())),
        Outpost(new Rotation2d(), new Pose2d()),
        Depot(new Rotation2d(), new Pose2d(FieldConstants.Depot.depotCenter.toTranslation2d(), new Rotation2d())),
        Climb(new Rotation2d(), new Pose2d()),
        None(new Rotation2d(), new Pose2d());

        private Pose2d[] poses;
        private Rotation2d end;

        AutoActivity(Rotation2d end, Pose2d... poses) {
            this.end = end;
            this.poses = poses;
        }

        public Pose2d[] getPoses() {
            return poses;
        }

        public PathPlannerPath getPathFrom(Pose2d from) {
            // if (this == AutoActivity.None) {
            //     return null;
            // }
            List<Pose2d> listposes = new ArrayList<>(Arrays.asList(poses));
            listposes.add(0, from);
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(listposes);
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(3, 6, degreesToRadians(540), degreesToRadians(720)),
                null,
                new GoalEndState(0, end));
            return path;
        }
    }

    /*public enum AutoActivity {
        CollectShootNeutral,
        ShuttleNeutral,
        Chaos, // chaos :)
    }

    public enum AutoEnd {
        Outpost,
        Depot,
        Climb,
    }*/

    // private static Pathfinder pathfinder = new LocalADStarAK();

    /** Creates a new AutoBuilder. */
    public AutoBuilder(AutoStart start, AutoActivity activity, DriveStateMachine drive, PoseEstimatorSubsystem pose) {
        try {
            // PathPlannerPath path = activity.getPathFrom(start.getPose());
            // com.pathplanner.lib.auto.AutoBuilder.pathfindToPose(null, null, null)
            Pathfinding.setStartPosition(start.getPose().getTranslation());
            Pathfinding.setGoalPosition(activity.getPoses()[0].getTranslation());
            for (;!Pathfinding.isNewPathAvailable(););
            var path = Pathfinding.getCurrentPath(new PathConstraints(3.0, 3.0, degreesToRadians(540.0), degreesToRadians(720.0)), new GoalEndState(0, new Rotation2d()));

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                // start_path = start_path.flipPath();
                // end_path = end_path.flipPath();
            }

            // Add your commands in the addCommands() call, e.g.
            // addCommands(new FooCommand(), new BarCommand());
            addCommands(
                pose.setCurrentPoseCommand(start.getPose()),
                new SwerveAutoStep(path, pose)
                // pose.setCurrentPoseCommand(start_path.getStartingHolonomicPose().get()),
                // new SwerveAutoStep(start_path, pose),
                // new SwerveAutoStep(end_path, pose)
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to build auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
