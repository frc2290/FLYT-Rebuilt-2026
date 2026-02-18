// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.utils.PoseEstimatorSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.robot.subsystems.StateMachines.DriveStateMachine.DriveState;
import frc.robot.subsystems.StateMachines.StateMachine.SpecialZone;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.turret.Turret;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.utils.FuelSim;

import static edu.wpi.first.math.util.Units.inchesToMeters;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    // The robot's subsystems
    private final Drive m_drive;
    private final PoseEstimatorSubsystem m_poseEstimator;
    private final StateMachine m_stateMachine;
    private final DriveStateMachine m_driveStateMachine;
    private final Intake m_intake;
    private final DyeRotor m_dyeRotor;
    private final Turret m_turret;

    // The driver's controller
    XboxController m_driverController;

    SendableChooser<Command> auto_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(
            Drive _drive,
            PoseEstimatorSubsystem _poseEstimator,
            StateMachine _stateMachine,
            DriveStateMachine _driveStateMachine,
            Intake _intake,
            DyeRotor _dyeRotor,
            Turret _turret,
            XboxController _driverController) {

        m_drive = _drive;
        m_poseEstimator = _poseEstimator;
        m_stateMachine = _stateMachine;
        m_driveStateMachine = _driveStateMachine;
        m_intake = _intake;
        m_dyeRotor = _dyeRotor;
        m_turret = _turret;
        m_driverController = _driverController;

        // Configure the button bindings
        configureButtonBindings();

        // Auto poses place here
        SmartDashboard.putData(auto_chooser);

        if (Robot.isSimulation()) {
            FuelSim instance = FuelSim.getInstance();
            // instance.spawnStartingFuel();
            instance.registerRobot(inchesToMeters(30), inchesToMeters(37), inchesToMeters(5.0), _poseEstimator::getCurrentPose, _drive::getChassisSpeeds);
            instance.start();
        }
    }

    private void configureButtonBindings() {
        Trigger isOnBump = new Trigger(() -> m_stateMachine.getSpecialZone() == SpecialZone.BUMP);
        Trigger isUnderTrench = new Trigger(() -> m_stateMachine.getSpecialZone() == SpecialZone.TRENCH);
        isOnBump.whileTrue(m_driveStateMachine.tempChangeState(DriveState.BUMP));
        isUnderTrench.whileTrue(m_driveStateMachine.tempChangeState(DriveState.TRENCH));

        Trigger leftNotIn = new Trigger(() -> !m_intake.isIn(IntakeSide.LEFT));
        Trigger rightNotIn = new Trigger(() -> !m_intake.isIn(IntakeSide.RIGHT));
        leftNotIn.onTrue(m_driveStateMachine.changeSnakeDirection(IntakeSide.LEFT));
        rightNotIn.onTrue(m_driveStateMachine.changeSnakeDirection(IntakeSide.RIGHT));

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
        b_button.whileTrue(
                new ParallelCommandGroup(
                        m_intake.driveIntake(),
                        m_driveStateMachine.tempChangeState(DriveState.SNAKE)));

        x_button.onTrue(m_intake.intakeOut(IntakeSide.LEFT));
        y_button.onTrue(m_intake.intakeOut(IntakeSide.RIGHT));

        dpad_left.onTrue(m_driveStateMachine.changeState(DriveState.MANUAL));
        dpad_up.onTrue(m_driveStateMachine.changeState(DriveState.SNAKE));

        // Manual controls.
        // dpad_left.toggleOnTrue(
        //         new ParallelCommandGroup(
        //                 new InstantCommand(
        //                         () -> m_coordinator.setControllerProfile(ControllerProfile.MANUAL)),
        //                 new InstantCommand(
        //                         () -> m_coordinator.setRobotGoal(
        //                                 RobotState.MANUAL)))); // Manual

        // dpad_up.toggleOnTrue(
        //         new ParallelCommandGroup(
        //                 new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.MANUAL)),
        //                 new InstantCommand(
        //                         () -> m_coordinator.setRobotGoal(
        //                                 RobotState.SHOOT)))); // Algae profile with safe travel goal.

        // maybe fix this perhaps
        // Other controls.
        // right_stick
        //         .and(dpad_right)
        //         .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading())); // Manual heading reset.

        // left_trigger.whileTrue(new Intake(m_intakeShooter));
        // right_trigger.whileTrue(new Shoot(m_intakeShooter, 1));
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
