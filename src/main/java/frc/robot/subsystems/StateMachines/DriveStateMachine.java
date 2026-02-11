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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Commands.DriveCommandFactory;
import frc.utils.FlytDashboard;
import frc.utils.PoseEstimatorSubsystem;

public class DriveStateMachine extends SubsystemBase {
    public enum DriveState {
        CANCELLED,      // drive system cancelled
        MANUAL,         // field oriented
        SNAKE,          // point towards heading
        CLIMB_RELATIVE, // align with tower
        FOLLOW_PATH,    // auto path following
    }

    private FlytDashboard dashboard = new FlytDashboard("DriveStateMachine");
    private Drive drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;
    private final DriveCommandFactory driveCommandFactory;

    private DriveState driveState = DriveState.CANCELLED;
    private Command currentCommand = null;

    public DriveStateMachine(
            Drive m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;
        driveCommandFactory = new DriveCommandFactory(drive, pose, driverController);
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;
    }

    @Override
    public void periodic() {
        // Update dashboard
        dashboard.putString("Current State", getCurrentState().toString());
    }

    public void setDriveCommand(DriveState driveState) {
        this.driveState = driveState;
        if (currentCommand != null) currentCommand.cancel();
        currentCommand = switch (driveState) {
            case CANCELLED      -> driveCommandFactory.createCancelledCommand();
            case MANUAL         -> driveCommandFactory.createManualDriveCommand();
            case SNAKE          -> driveCommandFactory.createHeadingLockCommand(() -> {
                double forward = driveCommandFactory.sampleForwardInput();
                double strafe = -driveCommandFactory.sampleStrafeInput();
                if (forward != 0 || strafe != 0)
                    return Math.toDegrees(Math.atan2(forward, strafe));
                return pose.getDegrees();
            });
            case CLIMB_RELATIVE -> driveCommandFactory.createHeadingLockCommand(() -> 0.0);
            case FOLLOW_PATH    -> driveCommandFactory.createFollowPathCommand();
        };
        currentCommand.schedule();
    }

    public DriveState getCurrentState() {
        return driveState;
    }
}