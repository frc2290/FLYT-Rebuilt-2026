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
package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.utils.FieldConstants.LinesHorizontal;
import frc.utils.FieldConstants.LinesVertical;
import frc.utils.FieldConstants;
import frc.utils.PoseEstimatorSubsystem;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Factory for building the common drive commands used throughout the robot code.
 *
 * <p>The goal of this class is to give students a single place to look when they want to understand
 * how the driver sticks get translated into drivetrain motions. Every public method returns a
 * ready-to-run command that mirrors the legacy behaviors (manual drive, heading locks, pose holds,
 * etc.) so the rest of the robot code can request the right behavior without duplicating logic. The
 * helpers intentionally stay lightweight—each command is built with a small lambda that calls
 * {@link DriveSubsystem#drive} directly—so it is easy to extend with future drive modes.
 */
public final class DriveCommandFactory {

  private final Drive drive;
  private final PoseEstimatorSubsystem poseEstimator;
  private final CommandXboxController driverController;
  private final PIDController rotPid;
  private final PIDController xPid;
  private final PIDController yPid;

  /**
   * Constructs a drive command factory that can create commands using the shared drivetrain, pose
   * estimator, and driver controller references.
   */
  public DriveCommandFactory(
      Drive drive, PoseEstimatorSubsystem poseEstimator, CommandXboxController driverController) {
    this.drive = Objects.requireNonNull(drive);
    this.poseEstimator = Objects.requireNonNull(poseEstimator);
    this.driverController = Objects.requireNonNull(driverController);
    // this.rotPid = drive.getRotPidController();
    // this.xPid = drive.getXPidController();
    // this.yPid = drive.getYPidController();
    this.rotPid = new PIDController(0.015, 0.0, 0.0); // 0.015 0 0
    this.rotPid.enableContinuousInput(0, 360);
    this.xPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5
    this.yPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5


  }

  /** Small helper that bundles the driver stick inputs for a single loop of command execution. */
  public static final class DriverInputs {
    public final double xSpeed;
    public final double ySpeed;
    public final double rotSpeed;

    DriverInputs(double xSpeed, double ySpeed, double rotSpeed) {
      this.xSpeed = xSpeed;
      this.ySpeed = ySpeed;
      this.rotSpeed = rotSpeed;
    }
  }

  /**
   * Samples the driver's left Y stick, applies the configured deadband, and flips the axis so
   * forward stick pushes produce positive field-relative X speeds.
   */
  public double sampleForwardInput() {
    return -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
  }

  /**
   * Samples the driver's left X stick with the same shaping as the original manual drive command so
   * small stick noise is ignored.
   */
  public double sampleStrafeInput() {
    return -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
  }

  /** Samples the driver's right X stick to determine the desired rotational velocity. */
  private double sampleRotationInput() {
    return -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);
  }

  /** Grabs a snapshot of the driver inputs so each command can reason about the same numbers. */
  public DriverInputs sampleDriverInputs() {
    return new DriverInputs(sampleForwardInput(), sampleStrafeInput(), sampleRotationInput());
  }

  /** Runs the provided controller each loop while reserving the drivetrain requirement. */
  private Command runDriveCommand(Consumer<DriverInputs> controller) {
    Objects.requireNonNull(controller);

    return Commands.run(() -> controller.accept(sampleDriverInputs()), drive);
  }

  /**
   * Applies manual rotation if the driver moves the right stick. Returns true when the manual
   * override has been applied so the caller can exit early.
   */
  private boolean applyManualRotationOverride(DriverInputs inputs) {
    if (inputs.rotSpeed != 0.0) {
      drive.drive(inputs.xSpeed, inputs.ySpeed, inputs.rotSpeed, true);
      return true;
    }
    return false;
  }

  /**
   * Creates a command that zeros all drivetrain outputs while scheduled.
   *
   * <p>This matches the previous {@code CancelledDrive} behavior and is used whenever the driver
   * state machine needs the drivetrain to coast.
   */
  public Command createCancelledCommand() {
    return Commands.run(() -> drive.drive(0.0, 0.0, 0.0, true), drive);
  }

  /**
   * Creates a command that simply passes driver input through to the drivetrain.
   *
   * <p>The stick shaping intentionally mirrors the legacy {@code ManualDrive} class so the student
   * controls feel identical after this refactor. This is the baseline mode used during teleop.
   */
  public Command createManualDriveCommand() {
    return runDriveCommand(
        inputs ->
            // Pass the field-relative speeds straight to the drivetrain.
            drive.drive(inputs.xSpeed, inputs.ySpeed, inputs.rotSpeed, true));
  }

  /**
   * Creates a command that mirrors the autonomous follower by steering toward the pose estimator's
   * target pose while still respecting manual rotation overrides.
   *
   * <p>This helper is used by the path-following state so students can compare the teleop follower
   * to the autonomous PathPlanner routine—it reuses the exact same PID controllers that the autos
   * rely on.
   */
  public Command createFollowPathCommand() {
    return runDriveCommand(
        inputs -> {
          // Give the driver full control of heading if they request it.
          if (applyManualRotationOverride(inputs)) {
            return;
          }

          // Otherwise steer toward the pose estimator's active trajectory target.
          Pose2d targetPose =
              Objects.requireNonNull(
                  poseEstimator.getTargetPose(), "Follow path target pose was null");

          // These PID calculations intentionally mirror the autonomous follower.
          double rotSpeed =
              rotPid.calculate(poseEstimator.getDegrees(), targetPose.getRotation().getDegrees());
          double xCommand =
              xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
          double yCommand =
              yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

          // Drive toward the path planner target while still using field-relative speeds.
          drive.drive(xCommand, yCommand, rotSpeed, true);
        });
  }

  /**
   * Creates a command that keeps the robot pointed toward a heading unless overridden.
   *
   * <p>The caller supplies a heading provider so this helper can be used both for static targets
   * (like the stored barge heading) and dynamic targets (like pointing at a pose).
   */
  public Command createHeadingLockCommand(DoubleSupplier headingSupplier) {
    Objects.requireNonNull(headingSupplier);

    return runDriveCommand(
        inputs -> {
          // The student can override heading at any time with the right stick.
          if (applyManualRotationOverride(inputs)) {
            return;
          }

          double headingTarget = headingSupplier.getAsDouble();
          // System.out.println(headingTarget);
          double rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), headingTarget);
          drive.drive(inputs.xSpeed, inputs.ySpeed, rotSpeed, true);
        });
  }

  /**
   * Creates a command that aligns the robot to a pose while optionally blending in manual control.
   *
   * <p>The {@code manualBlend} parameter determines how much of the driver's translation input is
   * preserved while the PID controllers pull the robot onto the target pose. A blend of 0 keeps the
   * drivetrain fully locked to the target, while higher values allow slow creeping adjustments.
   */
  public Command createHoldPoseCommand(
      Supplier<Pose2d> targetPoseSupplier, double manualBlend, boolean updateTargetPose) {
    Objects.requireNonNull(targetPoseSupplier);

    return runDriveCommand(
        inputs -> {
          // Let the driver fully override heading control whenever they move the stick.
          if (applyManualRotationOverride(inputs)) {
            return;
          }

          Pose2d targetPose =
              Objects.requireNonNull(
                  targetPoseSupplier.get(), "Target pose supplier returned null");
          if (updateTargetPose) {
            // Synchronize the estimator's target pose so autonomous code still sees the same goal.
            poseEstimator.setTargetPose(targetPose);
          }

          // Drive the robot's X/Y toward the pose using the same PID controllers the autos use.
          double rotSpeed =
              rotPid.calculate(poseEstimator.getDegrees(), targetPose.getRotation().getDegrees());
          double xCorrection =
              xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
          double yCorrection =
              yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

          // Blend in a fraction of the driver's manual input so they can creep around the target.
          double xCommand = xCorrection + inputs.xSpeed * manualBlend;
          double yCommand = yCorrection + inputs.ySpeed * manualBlend;
          drive.drive(xCommand, yCommand, rotSpeed, true);
        });
  }

  /**
   * Creates a command that keeps the robot aimed toward a target pose's translation while still
   * letting the driver translate.
   *
   * <p>This is essentially a heading lock where the heading is recomputed every cycle based on the
   * target pose. The drivetrain can still strafe manually, but its nose stays pointed at the
   * supplied pose—perfect for lining up with the reef or processor. When {@code updateTargetPose}
   * is {@code true} the pose estimator's public target is refreshed each loop so dashboards and
   * autos see the same heading the driver experiences.
   */
  public Command createPointingAtPoseCommand(
      Supplier<Pose2d> targetPoseSupplier, boolean updateTargetPose) {
    Objects.requireNonNull(targetPoseSupplier);

    // Delegate to the heading-lock helper so we keep all of the same manual override rules.
    return runDriveCommand(
        inputs -> {
          if (applyManualRotationOverride(inputs)) {
            return;
          }

          Pose2d targetPose =
              Objects.requireNonNull(
                  targetPoseSupplier.get(), "Target pose supplier returned null");
          double headingTarget = poseEstimator.turnToTarget(targetPose.getTranslation());
          if (updateTargetPose) {
            // Keep the estimator's public target pose synchronized for dashboards and autos.
            poseEstimator.setTargetPose(
                new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(headingTarget)));
          }

          double rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), headingTarget);
          drive.drive(inputs.xSpeed, inputs.ySpeed, rotSpeed, true);
        });
  }

  public Command createTrenchBumpCommand() {
    return runDriveCommand(
        inputs -> {
          /*{
            Translation2d trenchBumpDivide = new Translation2d(
              isNear ?
                LinesVertical.allianceZone : LinesVertical.neutralZoneNear,
              isOnLeft ?
                LinesHorizontal.leftMiddle : LinesHorizontal.rightMiddle);
            Translation2d error = trenchBumpDivide.minus(currentTranslation);
            Rotation2d errorAngle = error.getAngle();
            
            double targetY;
            Logger.recordOutput("DriveAssist/Rel", inputAngle.relativeTo(errorAngle));
            if ((inputAngle.relativeTo(errorAngle).getRadians() > 0) ^ isOnLeft ^ !isNear) {
              targetY = (LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2.0;
            } else {
              targetY = LinesHorizontal.rightTrenchOpenStart / 2.0;
            }
            target = new Translation2d(LinesVertical.trenchCenter, isOnLeft ? FieldConstants.fieldWidth - targetY : targetY);
            Logger.recordOutput("DriveAssist/Target", target);
          }*/
          Pose2d currentPose = poseEstimator.getCurrentPose();
          ChassisSpeeds speeds = drive.getChassisSpeeds();
          Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
          Translation2d velocity = robotVelocity.rotateBy(currentPose.getRotation());
          Translation2d soon = currentPose.getTranslation().plus(velocity.times(0.25));
          double soonY = soon.getY();

          OptionalDouble ySpeed = OptionalDouble.empty();
          OptionalDouble rotSpeed = OptionalDouble.empty();
          double[] targets = {LinesHorizontal.rightTrenchMiddle, LinesHorizontal.rightBumpMiddle, LinesHorizontal.leftBumpMiddle, LinesHorizontal.leftTrenchMiddle};
          // double weight = 1.0 / (1.0 * Math.abs(LinesVertical.allianceZone - currentPose.getX()) + 1.0);
          double weight = 0.8;
          double targetY;
          targetY = Arrays.stream(targets).reduce((x, y) -> Math.abs(x - soonY) < Math.abs(y - soonY) ? x : y).orElse(targets[0]);
          ySpeed = OptionalDouble.of(MathUtil.interpolate(inputs.ySpeed, yPid.calculate(currentPose.getY(), targetY), weight));
          rotSpeed = OptionalDouble.of(MathUtil.interpolate(inputs.rotSpeed, rotPid.calculate(poseEstimator.getDegrees(), Math.round(poseEstimator.getDegrees() / 90.0) * 90.0), weight));

          drive.drive(inputs.xSpeed, ySpeed.orElse(inputs.ySpeed), rotSpeed.orElse(inputs.rotSpeed), true);
        }
    );
  }

  public Command createDriveAssistCommand() {
    return runDriveCommand(
        inputs -> {
          // input stuff
          Rotation2d inputAngle = new Rotation2d(inputs.xSpeed, inputs.ySpeed);
          double inputMagnitude = Math.hypot(inputs.xSpeed, inputs.ySpeed);
          Logger.recordOutput("DriveAssist/InputAngle", inputAngle);

          // position and velocity
          Pose2d currentPose = poseEstimator.getCurrentPose();
          Translation2d currentTranslation = currentPose.getTranslation();

          boolean isNear = currentPose.getX() < LinesVertical.trenchCenter;
          boolean isOnLeft = currentPose.getY() > FieldConstants.fieldWidth / 2.0;
          Logger.recordOutput("DriveAssist/IsNear", isNear);
          Logger.recordOutput("DriveAssist/IsOnLeft", isOnLeft);

          ChassisSpeeds speeds = drive.getChassisSpeeds();
          Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
          Translation2d velocity = robotVelocity.rotateBy(currentPose.getRotation());
          Logger.recordOutput("DriveAssist/Velocity", velocity);
          
          Translation2d soon = currentTranslation.plus(velocity.times(0.25));
          Logger.recordOutput("DriveAssist/Soon", soon);

          // target calculation
          Translation2d target = new Translation2d();

          // positive corrective steering
          Translation2d error = target.minus(currentTranslation);
          Rotation2d errorAngle = error.getAngle();
          double errorDist = error.getNorm();
          Logger.recordOutput("DriveAssist/ErrorAngle", errorAngle);
          Logger.recordOutput("DriveAssist/ErrorDist", errorDist);

          // double weight = 1.0 / (errorDist / 4.0 + 1.0);
          double weight = 1.0;
          Logger.recordOutput("DriveAssist/PositiveCorrectiveWeight", weight);

          Rotation2d driveAngle;
          double relativeAngle = inputAngle.relativeTo(errorAngle).getDegrees();
          if (relativeAngle > -30 && relativeAngle < 30) {
              driveAngle = inputAngle.interpolate(errorAngle, weight);
          } else {
              driveAngle = inputAngle;
          }

          // the driving
          drive.drive(driveAngle.getCos() * inputMagnitude, driveAngle.getSin() * inputMagnitude, inputs.rotSpeed, true);
        });
  }
}
