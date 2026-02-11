// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.subsystems.StateMachines;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Commands.DriveCommandFactory;
import frc.robot.Commands.GraphCommand;
import frc.robot.Commands.GraphCommand.GraphCommandNode;
import frc.utils.FlytDashboard;
import frc.utils.PoseEstimatorSubsystem;

public class DriveStateMachine extends SubsystemBase {

    private GraphCommand m_graphCommand = new GraphCommand();
    private FlytDashboard dashboard = new FlytDashboard("DriveStateMachine");
    private Drive drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;
    private final DriveCommandFactory driveCommandFactory;

    /** DriveTrain states - drive state machine */
    public enum DriveState {
        MANUAL, // Field oriented freeroam
        FOLLOW_PATH, // Auto path following
        CLIMB_RELATIVE,
        FIELD_ORIENTED,
        HUB_ORIENTED, // point at hub
        POINT_AT_FUEL,
        CANCELLED; // Drive system cancelled
    }

    /*
     * Graph Command Nodes for Drive State Machine
     */
    GraphCommandNode manualNode;
    GraphCommandNode followPathNode;
    GraphCommandNode pointAtHubNode;
    GraphCommandNode pointAtFuelNode;
    GraphCommandNode cancelledNode;

    /**
     * Constructor for the DriveStateMachine
     *
     * @param m_drive
     * @param m_pose
     * @param m_driverController
     */
    public DriveStateMachine(
            Drive m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;
        driveCommandFactory = new DriveCommandFactory(drive, pose, driverController);
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;

        // Initialize graph command
        initializeGraphCommand();

        // Set as default command so it runs all the time
        m_graphCommand.addRequirements(this);
        this.setDefaultCommand(m_graphCommand);
        m_graphCommand.setCurrentNode(cancelledNode);
    }

    /** Initialize the graph command with all nodes and connections */
    private void initializeGraphCommand() {
        // Create all graph command nodes
        manualNode = m_graphCommand.new GraphCommandNode(
                "Manual",
                Commands.none(),
                Commands.none(),
                driveCommandFactory.createManualDriveCommand());

        pointAtHubNode = m_graphCommand.new GraphCommandNode(
                "PointAtHub",
                Commands.none(),
                Commands.none(),
                driveCommandFactory.createPointingAtPoseCommand(() -> VisionConstants.hubCenterPose.toPose2d(), false));
        
        pointAtFuelNode = m_graphCommand.new GraphCommandNode(
                "PointAtFuel",
                Commands.none(),
                Commands.none(),
                driveCommandFactory.createHeadingLockCommand(() -> {
                    System.out.println(pose.getDegrees() - pose.getTargetYaw());
                    System.out.println(pose.getTargetYaw());
                    return pose.getDegrees() - pose.getTargetYaw();
                }));

        followPathNode = m_graphCommand.new GraphCommandNode(
                "FollowPath",
                Commands.none(),
                new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0, true)),
                driveCommandFactory.createFollowPathCommand());

        cancelledNode = m_graphCommand.new GraphCommandNode(
                "Cancelled", driveCommandFactory.createCancelledCommand(), null, null);

        // Graph Command setup
        m_graphCommand.setGraphRootNode(cancelledNode); // shouldn't drive

        // Define transitions between drive states
        cancelledNode.AddNode(manualNode, 1.0);
        manualNode.AddNode(cancelledNode, 1.0);
        manualNode.AddNode(pointAtHubNode,1.0);
        manualNode.AddNode(pointAtFuelNode,1.0);
        followPathNode.AddNode(cancelledNode, 1.0);
        followPathNode.AddNode(manualNode, 1.0);

    }

    /** ----- Branch Selection ----- */

    /** ----- State Transition Commands ----- */

    /**
     * Set Goal DriveState for DrimeStateMachine
     *
     * @return
     */
    public void setDriveCommand(DriveState m_driveState) {
        switch (m_driveState) {
            case MANUAL:
                m_graphCommand.setTargetNode(manualNode);
                break;
            case FOLLOW_PATH:
                m_graphCommand.setTargetNode(followPathNode);
                break;
            case HUB_ORIENTED:
                m_graphCommand.setTargetNode(pointAtHubNode);
                break;
            case POINT_AT_FUEL:
                m_graphCommand.setTargetNode(pointAtFuelNode);
                break;
            case CANCELLED:
                m_graphCommand.setTargetNode(cancelledNode);
                break;
            default:
                m_graphCommand.setTargetNode(manualNode);
                break;
        }
    }

    /**
     * Check if graph command still reaching goal state
     *
     * @return
     */
    public boolean isTransitioning() {
        return m_graphCommand.isTransitioning();
    }

    public boolean atPosition() {
        return pose.atTargetPose();
    }

    /** ----- State Getters ----- */

    /**
     * Get Current state
     *
     * @return
     */
    public DriveState getCurrentState() {
        GraphCommandNode currentNode = m_graphCommand.getCurrentNode();
        if (currentNode == manualNode)
            return DriveState.MANUAL;
        if (currentNode == followPathNode)
            return DriveState.FOLLOW_PATH;
        if (currentNode == pointAtHubNode)
            return DriveState.HUB_ORIENTED;
        if (currentNode == pointAtFuelNode)
            return DriveState.POINT_AT_FUEL;
        if (currentNode == cancelledNode)
            return DriveState.CANCELLED;
        return DriveState.MANUAL; // Default
    }

    /** ----- Periodic ----- */
    @Override
    public void periodic() {
        // Update dashboard
        dashboard.putString("Current State", getCurrentState().toString());
        dashboard.putBoolean("At State", !isTransitioning());
        dashboard.putBoolean("At Drive Position", atPosition());
    }
}
