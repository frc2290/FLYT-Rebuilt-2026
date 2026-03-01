// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.utils.PoseEstimatorSubsystem;
import frc.robot.Commands.Autos.AutoBuilder;
import frc.robot.Commands.Autos.AutoBuilder.AutoActivity;
import frc.robot.Commands.Autos.AutoBuilder.AutoEnd;
import frc.robot.Commands.Autos.AutoBuilder.AutoStart;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.robot.subsystems.StateMachines.DriveStateMachine.DriveState;
import frc.robot.subsystems.StateMachines.StateMachine.FieldZone;
import frc.robot.subsystems.StateMachines.StateMachine.SpecialZone;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.utils.FuelSim;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.lang.runtime.ObjectMethods;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
    CommandXboxController m_driverController;
    CommandXboxController m_operatorController = new CommandXboxController(1);

    SendableChooser<Command> auto_chooser = new SendableChooser<>();
    LoggedDashboardChooser<AutoStart> auto_start = new LoggedDashboardChooser<>("Auto Start");
    LoggedDashboardChooser<AutoActivity> auto_activity = new LoggedDashboardChooser<>("Auto Activity");
    LoggedDashboardChooser<AutoEnd> auto_end = new LoggedDashboardChooser<>("Auto End");

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
            CommandXboxController _driverController) {

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
        // this function is TERRIBLE but it's funny and it works so i am keeping it
        BiConsumer<Object, Object[]> enum_chooser = (perhaps_chooser, values) -> {
            LoggedDashboardChooser<Object> chooser = (LoggedDashboardChooser<Object>)perhaps_chooser;
            chooser.addDefaultOption(values[0].toString(), values[0]);
            for (Object value : values) {
                chooser.addOption(value.toString(), value);
            }
        };
        enum_chooser.accept(auto_start, AutoStart.values());
        enum_chooser.accept(auto_activity, AutoActivity.values());
        enum_chooser.accept(auto_end, AutoEnd.values());
        // auto_start_pos.addDefaultOption("Trench", "Trench");
        // auto_start_pos.addOption("Bump", "Bump");
        // auto_end_pos.addDefaultOption("Depot", "Depot");
        // auto_end_pos.addOption("Outpost", "Outpost");

        if (Robot.isSimulation()) {
            FuelSim instance = FuelSim.getInstance();
            // instance.spawnStartingFuel();
            instance.registerRobot(inchesToMeters(30), inchesToMeters(37), inchesToMeters(5.0), _poseEstimator::getCurrentPose, _drive::getChassisSpeeds);
            instance.start();
        }
    }

    private void configureButtonBindings() {
        // TRIGGERS
        Trigger isAuto = new Trigger(() -> m_stateMachine.isAuto());
        Trigger notAuto = isAuto.negate();
        Trigger isOnBump = new Trigger(() -> m_stateMachine.getSpecialZone() == SpecialZone.BUMP);
        Trigger isUnderTrench = new Trigger(() -> m_stateMachine.getSpecialZone() == SpecialZone.TRENCH);
        Trigger isInNeutral = new Trigger(() -> m_stateMachine.getFieldZone() == FieldZone.NEUTRAL);
        Trigger isLeft = new Trigger(() -> m_stateMachine.getLeftSide());

        isOnBump.and(notAuto).whileTrue(m_driveStateMachine.tempChangeState(DriveState.BUMP));
        isUnderTrench.and(notAuto).whileTrue(m_driveStateMachine.tempChangeState(DriveState.TRENCH));

        Trigger leftNotIn = new Trigger(() -> !m_intake.isIn(IntakeSide.LEFT));
        Trigger rightNotIn = new Trigger(() -> !m_intake.isIn(IntakeSide.RIGHT));
        leftNotIn.onTrue(m_driveStateMachine.changeSnakeDirection(IntakeSide.LEFT));
        rightNotIn.onTrue(m_driveStateMachine.changeSnakeDirection(IntakeSide.RIGHT));

        // END TRIGGERS

        // DRIVER Control definitions

        m_driverController.a().onTrue(m_dyeRotor.runDyeRotorCommand(true)).onFalse(m_dyeRotor.runDyeRotorCommand(false));
        m_driverController.b().whileTrue(
                new ParallelCommandGroup(
                        m_intake.driveIntake(),
                        m_driveStateMachine.tempChangeState(DriveState.SNAKE)));

        m_driverController.x().onTrue(m_intake.intakeOut(IntakeSide.LEFT));
        m_driverController.y().onTrue(m_intake.intakeOut(IntakeSide.RIGHT));

        m_driverController.axisGreaterThan(3, 0.5).whileTrue(
                                new ParallelCommandGroup(
                                    m_intake.driveIntake(),
                                    m_driveStateMachine.tempChangeState(DriveState.SNAKE)));

        m_driverController.povLeft().onTrue(m_driveStateMachine.changeState(DriveState.MANUAL));
        m_driverController.povUp().onTrue(m_driveStateMachine.changeState(DriveState.SHOOT_LOCK));
        m_driverController.povRight().onTrue(Commands.runOnce(() -> m_drive.resetGyro())); // Reset Gyro Heading

        // END DRIVER BUTTONS

        // OPERATOR BUTTONS

        // Override the auto shooting, this same button overrides in both neutral and alliance zone
        // In alliance zone it will stop shooting and in neutral it will start shuttling fuel
        m_operatorController.a().onTrue(m_stateMachine.setShooterOverrideCommand(true))
                                .onFalse(m_stateMachine.setShooterOverrideCommand(false));

        // END OPERATOR BUTTONS
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new AutoBuilder(auto_start.get(), auto_activity.get(), auto_end.get(), m_driveStateMachine, m_poseEstimator);
    }
}
