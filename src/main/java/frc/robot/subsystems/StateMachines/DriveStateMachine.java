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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.Commands.DriveCommandFactory;
import frc.robot.Commands.DriveCommandFactory.DriverInputs;
import frc.utils.FlytDashboard;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.FieldConstants.LinesHorizontal;
import frc.utils.FieldConstants.LinesVertical;

public class DriveStateMachine extends SubsystemBase {
    public enum DriveState {
        CANCELLED,      // drive system cancelled
        MANUAL,         // field oriented
        ASSIST,         // driver asisst
        SNAKE,          // point towards heading
        TRENCH,         // turns to the closest 90deg
        BUMP,           // turns to the closest 45deg
        CLIMB_RELATIVE, // align with tower
        FOLLOW_PATH,    // auto path following
        SHOOT_LOCK      // lock to sotf heading
    }

    private FlytDashboard dashboard = new FlytDashboard("DriveStateMachine");
    private Drive drive;
    private PoseEstimatorSubsystem pose;
    private CommandXboxController driverController;
    private final DriveCommandFactory driveCommandFactory;

    private DriveState driveState = DriveState.CANCELLED;
    private DriveState prevDriveState = DriveState.CANCELLED;
    private IntakeSide snakeDirection = IntakeSide.LEFT;
    private Command currentCommand = null;
    private SlewRateLimiter snakeLimiter = new SlewRateLimiter(10);

    private double sotfHeading = 0.0;

    public DriveStateMachine(
            Drive m_drive, PoseEstimatorSubsystem m_pose, CommandXboxController m_driverController) {
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
        Logger.recordOutput("DriveStateMachine/CurrentState", driveState);
        Logger.recordOutput("DriveStateMachine/PrevState", prevDriveState);
        Logger.recordOutput("DriveStateMachine/SnakeDirection", snakeDirection);
    }

    /**
     * sets the drivestate. if it is null, it reverts to the previous one
     * @param driveState the drivestate to change to
     */
    public void setDriveCommand(DriveState driveState) {
        if (driveState == null) driveState = prevDriveState;
        prevDriveState = this.driveState;
        this.driveState = driveState;
        if (currentCommand != null) currentCommand.cancel();
        currentCommand = switch (driveState) {
            case CANCELLED      -> driveCommandFactory.createCancelledCommand();
            case MANUAL         -> driveCommandFactory.createManualDriveCommand();
            case ASSIST         -> driveCommandFactory.createTrenchBumpCommand();
            case SNAKE          -> driveCommandFactory.createHeadingLockCommand(() -> {
                DriverInputs inputs = driveCommandFactory.sampleDriverInputs();
                double forward = inputs.xSpeed;
                double strafe = -inputs.ySpeed;
                if (forward != 0 || strafe != 0) {
                    // s stands for sign
                    float s = snakeDirection == IntakeSide.LEFT ? -1 : 1;
                    return Math.toDegrees(Math.atan2(s * forward, s * strafe)) % 360;
                }
                return snakeLimiter.calculate(pose.getDegrees());
            });
            case TRENCH         -> driveCommandFactory.createHeadingLockCommand(() -> Math.round(pose.getDegrees() / 90.0) * 90.0);
            case BUMP           -> driveCommandFactory.createHeadingLockCommand(() -> (Math.round(pose.getDegrees() / 90.0 - 0.5) * 90.0 + 45) % 360);
            case CLIMB_RELATIVE -> driveCommandFactory.createHeadingLockCommand(() -> 0.0);
            case FOLLOW_PATH    -> driveCommandFactory.createFollowPathCommand();
            case SHOOT_LOCK     -> driveCommandFactory.createHeadingLockCommand(() -> sotfHeading);
        };
        currentCommand.schedule();
    }

    public DriveState getCurrentState() {
        return driveState;
    }

    /**
     * makes a command to change the state
     * @param driveState the drivestate to change to
     * @return the command
     */
    public Command changeState(DriveState driveState) {
        return runOnce(() -> setDriveCommand(driveState));
    }

    /**
     * temporarily changes state while true and reverts back once ended
     * @param driveState the drivestate to change to
     * @return the command
     */
    public Command tempChangeState(DriveState driveState) {
        return startEnd(() -> setDriveCommand(driveState),
                        () -> setDriveCommand(null));
    }

    public Command changeSnakeDirection(IntakeSide snakeDirection) {
        return runOnce(() -> this.snakeDirection = snakeDirection);
    }

    public void setSotfHeading(double heading) {
        this.sotfHeading = heading;
    }
}
