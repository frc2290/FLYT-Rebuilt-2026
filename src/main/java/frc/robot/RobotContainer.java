// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.Intake;
import frc.robot.Commands.Shoot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Coordinator;
import frc.robot.subsystems.Coordinator.ControllerProfile;
import frc.robot.subsystems.Coordinator.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooter;
import frc.utils.FuelSim;
import frc.utils.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drive m_robotDrive;
    private final IntakeShooter m_intakeShooter;
    private final PoseEstimatorSubsystem m_PoseEstimator;
    private final DriveStateMachine m_drive_state;
    private final Coordinator m_coordinator;
    private final Turret m_turret;

    // The driver's controller
    XboxController m_driverController;

    SendableChooser<Command> auto_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(
            Drive _drive,
            IntakeShooter _intake,
            PoseEstimatorSubsystem _PoseEstimator,
            DriveStateMachine _driveStateMachine,
            Coordinator _coordinator,
            Turret _turret,
            XboxController _driverController) {

        m_robotDrive = _drive;
        m_intakeShooter = _intake;
        m_PoseEstimator = _PoseEstimator;
        m_drive_state = _driveStateMachine;
        m_coordinator = _coordinator;
        m_turret = _turret;
        m_driverController = _driverController;

        // Configure the button bindings
        configureButtonBindings();

        // Auto poses place here
        SmartDashboard.putData(auto_chooser);

        // // Configure default commands
        // m_robotDrive.setDefaultCommand(
        // // The left stick controls translation of the robot.
        // // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        // () -> m_robotDrive.drive(
        // -MathUtil.applyDeadband(m_driverController.getLeftY(),
        // OIConstants.kDriveDeadband),
        // -MathUtil.applyDeadband(m_driverController.getLeftX(),
        // OIConstants.kDriveDeadband),
        // -MathUtil.applyDeadband(m_driverController.getRightX(),
        // OIConstants.kDriveDeadband),
        // true),
        // m_robotDrive));

        if (Robot.isSimulation()) {
            FuelSim instance = FuelSim.getInstance();
            instance.spawnStartingFuel();
            instance.registerRobot(inchesToMeters(30.0), inchesToMeters(30.0), inchesToMeters(5.0), _drive::getPose, _drive::getChassisSpeeds);
            instance.start();
        }
    }

    // private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kX.value)
    // .whileTrue(new RunCommand(
    // () -> m_robotDrive.setX(),
    // m_robotDrive));

    // new JoystickButton(m_driverController, XboxController.Button.kStart.value)
    // .onTrue(new InstantCommand(
    // () -> m_robotDrive.zeroHeading(),
    // m_robotDrive));
    // }

    private void configureButtonBindings() {
        // Button definitions.
        // Map the raw controller buttons to descriptive names for readability.
        JoystickButton start_button = new JoystickButton(m_driverController, Button.kStart.value);

        JoystickButton a_button = new JoystickButton(m_driverController, Button.kA.value);
        JoystickButton b_button = new JoystickButton(m_driverController, Button.kB.value);
        JoystickButton x_button = new JoystickButton(m_driverController, Button.kX.value);
        JoystickButton y_button = new JoystickButton(m_driverController, Button.kY.value);

        JoystickButton left_stick = new JoystickButton(m_driverController, Button.kLeftStick.value);
        JoystickButton right_stick = new JoystickButton(m_driverController, Button.kRightStick.value);

        JoystickButton left_bumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
        JoystickButton right_bumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
        Trigger left_trigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
        Trigger right_trigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);

        POVButton dpad_up = new POVButton(m_driverController, 0);
        POVButton dpad_down = new POVButton(m_driverController, 180);
        POVButton dpad_left = new POVButton(m_driverController, 270);
        POVButton dpad_right = new POVButton(m_driverController, 90);

        a_button.onTrue(m_turret.shoot());

        // Manual controls.
        dpad_left.toggleOnTrue(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> m_coordinator.setControllerProfile(ControllerProfile.MANUAL)),
                        new InstantCommand(
                                () -> m_coordinator.setRobotGoal(
                                        RobotState.MANUAL)))); // Manual

        dpad_up.toggleOnTrue(
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.MANUAL)),
                        new InstantCommand(
                                () -> m_coordinator.setRobotGoal(
                                        RobotState.SHOOT)))); // Algae profile with safe travel goal.

        // TODO: FIX THIS
        // Other controls.
        // right_stick
        //         .and(dpad_right)
        //         .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading())); // Manual heading reset.

        left_trigger.whileTrue(new Intake(m_intakeShooter));
        right_trigger.whileTrue(new Shoot(m_intakeShooter, 1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return auto_chooser.getSelected();
    }
}
