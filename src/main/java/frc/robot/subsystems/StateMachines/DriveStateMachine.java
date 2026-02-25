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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        SNAKE,          // point towards heading
        TRENCH,         // turns to the closest 90deg
        BUMP,           // turns to the closest 45deg
        CLIMB_RELATIVE, // align with tower
        FOLLOW_PATH,    // auto path following
    }

    private FlytDashboard dashboard = new FlytDashboard("DriveStateMachine");
    private Drive drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;
    private final DriveCommandFactory driveCommandFactory;

    private DriveState driveState = DriveState.CANCELLED;
    private DriveState prevDriveState = DriveState.CANCELLED;
    private IntakeSide snakeDirection = IntakeSide.LEFT;
    private Command currentCommand = null;
    private boolean isOnLeftSide = false;

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
        Logger.recordOutput("DriveStateMachine/CurrentState", driveState);
        Logger.recordOutput("DriveStateMachine/PrevState", prevDriveState);
        Logger.recordOutput("DriveStateMachine/SnakeDirection", snakeDirection);

        updateVelocity();
    }

    private void updateVelocity() {
        Pose2d currentPose = pose.getCurrentPose();
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d velocity = robotVelocity.rotateBy(currentPose.getRotation());

        Translation2d target = new Translation2d(LinesVertical.allianceZone, LinesHorizontal.rightTrenchOpenStart / 2.0);
        Translation2d error = target.minus(currentPose.getTranslation());
        double errorAngle = error.getAngle().getRadians();
        double errorDist = error.getNorm();
        
        DriverInputs inputs = driveCommandFactory.sampleDriverInputs();
        double inputAngle = Math.atan2(inputs.ySpeed, inputs.xSpeed);
        double magnitude = Math.hypot(inputs.xSpeed, inputs.ySpeed);

        double weight = 1.0 / (0.5 * errorDist + 1.0);
        // double weight = 0.5;
        Logger.recordOutput("DriveStateMachine/errorDist", errorDist);
        Logger.recordOutput("DriveStateMachine/weight", weight);

        double driveAngle;
        if (Math.abs(inputAngle - errorAngle) < Math.PI / 2.0) {
            driveAngle = errorAngle * weight + inputAngle * (1.0 - weight);
        } else {
            driveAngle = inputAngle;
        }

        drive.drive(Math.cos(driveAngle) * magnitude, Math.sin(driveAngle) * magnitude, inputs.rotSpeed, true);
        
        // position 0.25 seconds in the future
        Translation2d soon = currentPose.getTranslation().plus(velocity.times(0.25));
        Logger.recordOutput("DriveStateMachine/Soon", soon);
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
            case SNAKE          -> driveCommandFactory.createHeadingLockCommand(() -> {
                double forward = driveCommandFactory.sampleForwardInput();
                double strafe = -driveCommandFactory.sampleStrafeInput();
                if (forward != 0 || strafe != 0) {
                    // s stands for sign
                    float s = snakeDirection == IntakeSide.LEFT ? -1 : 1;
                    return Math.toDegrees(Math.atan2(s * forward, s * strafe)) % 360;
                }
                return pose.getDegrees();
            });
            case TRENCH         -> driveCommandFactory.createHeadingLockCommand(() -> Math.round(pose.getDegrees() / 90.0) * 90.0);
            case BUMP           -> driveCommandFactory.createHeadingLockCommand(() -> (Math.round(pose.getDegrees() / 90.0 - 0.5) * 90.0 + 45) % 360);
            case CLIMB_RELATIVE -> driveCommandFactory.createHeadingLockCommand(() -> 0.0);
            case FOLLOW_PATH    -> driveCommandFactory.createFollowPathCommand();
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

    /**
     * tells the drivestatemachine what side of the field the robot is on
     * @param isOnLeftSide true if it's on the left side
     * @return the command
     */
    public Command changeFieldSide(boolean isOnLeftSide) {
        return runOnce(() -> this.isOnLeftSide = isOnLeftSide);
    }

    public Command changeSnakeDirection(IntakeSide snakeDirection) {
        return runOnce(() -> this.snakeDirection = snakeDirection);
    }
}