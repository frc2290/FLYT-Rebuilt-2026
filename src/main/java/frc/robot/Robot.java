// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.intake.IntakeConstants.leftZeroOffset;
import static frc.robot.subsystems.intake.IntakeConstants.rightZeroOffset;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.StateMachines.DriveStateMachine;
import frc.robot.subsystems.StateMachines.StateMachine;
import frc.robot.subsystems.StateMachines.DriveStateMachine.DriveState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;

import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.dyerotor.DyeRotorIO;
import frc.robot.subsystems.dyerotor.DyeRotorIOSim;
import frc.robot.subsystems.dyerotor.DyeRotorIOSpark;
import frc.robot.subsystems.energy.BatteryIOInputsAutoLogged;
import frc.robot.subsystems.energy.BatteryLogger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSpark;
import frc.utils.FuelSim;
import frc.utils.LEDEffects;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.LEDEffects.LEDEffect;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    /**
     * Cached handle to the active autonomous routine so we can cancel or reschedule
     * if needed.
     */
    private Command m_autonomousCommand;

    /** Container that wires up all subsystems, commands, and driver controls. */
    private RobotContainer m_robotContainer;

    private static final BatteryIOInputsAutoLogged batteryInputs = new BatteryIOInputsAutoLogged();
    public static final BatteryLogger batteryLogger = new BatteryLogger(batteryInputs);

    /**
     * Cached reference to the primary driver controller so subsystems can read
     * axes.
     */
    private CommandXboxController m_driver = new CommandXboxController(0);

    // The robot's subsystems.
    /** Owns all hardware for swerve driving and exposes the drive commands. */
    private final Drive m_robotDrive;
    private final PoseEstimatorSubsystem m_poseEstimator;
    private final Intake m_intake;
    private final DyeRotor m_dyeRotor;
    private final Turret m_turret;

    /** Coordinates all autonomous and teleop driving modes. */
    private final StateMachine m_stateMachine;
    private final DriveStateMachine m_driveStateMachine;

    private final LEDUtility _leds = new LEDUtility(9);

    LoggedDashboardChooser<Command> sys_id_commands = new LoggedDashboardChooser<>("SysID");

    public Robot() {
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("ProjectName", "FLYT-Rebuilt-2026"); // Set a metadata value

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
        case REAL:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case SIM:
            // Running a physics simulator, log to NT
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case REPLAY:
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                        // be added.

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_robotDrive = new Drive(
                               new GyroIONavX(),
                               new ModuleIOSpark(0),
                               new ModuleIOSpark(1),
                               new ModuleIOSpark(2),
                               new ModuleIOSpark(3));
                m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);
                m_intake = new Intake(new IntakeIOSpark(IntakeSide.LEFT, false, leftZeroOffset), new IntakeIOSpark(IntakeSide.RIGHT, true, rightZeroOffset));
                // m_dyeRotor = new DyeRotor(new DyeRotorIOSpark());
                m_dyeRotor = new DyeRotor(new DyeRotorIOSpark());
                m_turret = new Turret(new TurretIOSpark(),
                                      m_poseEstimator::getCurrentPose,
                                      m_robotDrive::getChassisSpeeds);
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_robotDrive = new Drive(
                               new GyroIO() {},
                               new ModuleIOSim(),
                               new ModuleIOSim(),
                               new ModuleIOSim(),
                               new ModuleIOSim());
                m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);
                var turretIO = new TurretIOSim(m_poseEstimator::getCurrentPose,
                                      m_poseEstimator::getChassisSpeeds);
                m_intake = new Intake(new IntakeIOSim(IntakeSide.LEFT, turretIO), new IntakeIOSim(IntakeSide.RIGHT, turretIO));
                m_dyeRotor = new DyeRotor(new DyeRotorIOSim());
                m_turret = new Turret(turretIO,
                                      m_poseEstimator::getCurrentPose,
                                      m_robotDrive::getChassisSpeeds);
                break;
            default:
                // Replayed robot, disable IO implementations
                m_robotDrive = new Drive(
                               new GyroIO() {},
                               new ModuleIO() {},
                               new ModuleIO() {},
                               new ModuleIO() {},
                               new ModuleIO() {});
                m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);
                m_intake = new Intake(new IntakeIO() {}, new IntakeIO() {});
                m_dyeRotor = new DyeRotor(new DyeRotorIO() {});
                m_turret = new Turret(new TurretIO() {},
                                        m_poseEstimator::getCurrentPose,
                                        m_robotDrive::getChassisSpeeds);
                break;
        }

        m_stateMachine = new StateMachine(m_poseEstimator::getCurrentPose, m_poseEstimator::getChassisSpeeds, m_intake, m_turret, m_dyeRotor);
        m_driveStateMachine = new DriveStateMachine(m_robotDrive, m_poseEstimator, m_intake, m_driver);

        _leds.addStrip("Front", 0, 65); // 66
        _leds.addStrip("Right", 66, 136); //71
        _leds.addStrip("Back", 137, 166); // 30
        _leds.addStrip("Left", 167, 237); //71
        _leds.setDefault();
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer(
                m_robotDrive,
                m_poseEstimator,
                m_stateMachine,
                m_driveStateMachine,
                m_intake,
                m_dyeRotor,
                m_turret,
                m_driver,
                _leds);

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        // Start the logging framework so we can view graphs after a match or practice
        // run.
        // Display the Command Scheduler Status

        // URCL.start();

        // If logging only to DataLog.
        URCL.start(DataLogManager.getLog());

        SmartDashboard.putData(CommandScheduler.getInstance());

        // Display subystem satatus
        SmartDashboard.putData(m_robotDrive);

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ConduitApi conduit = ConduitApi.getInstance();
        batteryInputs.batteryVoltage = conduit.getVoltageVin();
        batteryInputs.rioCurrent = conduit.getPDPChannelCurrent(20);
        batteryInputs.radioCurrent = conduit.getPDPChannelCurrent(21);
        batteryInputs.cameraCurrent = conduit.getPDPChannelCurrent(22);

        batteryInputs.cameraCurrent = RobotController.getInputCurrent();

        DriverStation.getAlliance().ifPresent(m_poseEstimator::setAlliance);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
        batteryLogger.periodicAfterScheduler();
    }

    @Override
    public void disabledInit() {
        m_driveStateMachine.setDriveCommand(DriveState.CANCELLED);
        _leds.setAll(LEDEffect.CHASING, LEDEffects.flytBlue);
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_stateMachine.setIsAuto(true);
        m_driveStateMachine.setDriveCommand(DriveState.FOLLOW_PATH);
        //m_autonomousCommand = new PathPlannerAuto("New Auto");

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_driveStateMachine.setDriveCommand(DriveState.CANCELLED);
        m_stateMachine.setIsAuto(false);
        m_stateMachine.setShootOverride(false);
    }

    @Override
    public void teleopInit() {
        m_driveStateMachine.setDriveCommand(DriveState.MANUAL);
        m_stateMachine.startHubTimer();

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // CODE AT THE TOP WAS DISABLED LAST SEASON WITH COARDINATOR
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // TEMP POINT UPDATE DRIVE TARGET HEADING TO TURRET HEADING FROM SOTF
        m_driveStateMachine.setDriveAngleCorrection(m_turret.getDriveAngleCorrection());
    }

    @Override
    public void teleopExit() {
        m_stateMachine.stopHubTimer();
    }

    @Override
    public void testInit() {

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        sys_id_commands.addDefaultOption("Drive Quasi Forward", m_robotDrive.sysIdQuasistatic(Direction.kForward));
        sys_id_commands.addOption("Drive Quasi Reverse", m_robotDrive.sysIdQuasistatic(Direction.kReverse));
        sys_id_commands.addOption("Drive Dynamic Forward", m_robotDrive.sysIdDynamic(Direction.kForward));
        sys_id_commands.addOption("Drive Dynamic Reverse", m_robotDrive.sysIdDynamic(Direction.kReverse));
        sys_id_commands.addOption("Turret Flywheel Quasi Forward", m_turret.sysIdQuasistaticFlywheel(Direction.kForward));
        sys_id_commands.addOption("Turret Flywheel Quasi Reverse", m_turret.sysIdQuasistaticFlywheel(Direction.kReverse));
        sys_id_commands.addOption("Turret Flywheel Dynamic Forward", m_turret.sysIdDynamicFlywheel(Direction.kForward));
        sys_id_commands.addOption("Turret Flywheel Dynamic Reverse", m_turret.sysIdDynamicFlywheel(Direction.kReverse));
        sys_id_commands.addOption("Turret Turn Quasi Forward", m_turret.sysIdQuasistaticTurn(Direction.kForward));
        sys_id_commands.addOption("Turret Turn Quasi Reverse", m_turret.sysIdQuasistaticTurn(Direction.kReverse));
        sys_id_commands.addOption("Turret Turn Dynamic Forward", m_turret.sysIdDynamicTurn(Direction.kForward));
        sys_id_commands.addOption("Turret Turn Dynamic Reverse", m_turret.sysIdDynamicTurn(Direction.kReverse));
        sys_id_commands.addOption("Turret Hood Characterize", m_turret.characterizeHood());
        sys_id_commands.addOption("Intake Left Quasi Forward", m_intake.sysIdQuasistaticLeftIntake(Direction.kForward));
        sys_id_commands.addOption("Intake Left Quasi Reverse", m_intake.sysIdQuasistaticLeftIntake(Direction.kReverse));
        sys_id_commands.addOption("Intake Left Dynamic Forward", m_intake.sysIdDynamicLeftIntake(Direction.kForward));
        sys_id_commands.addOption("Intake Left Dynamic Reverse", m_intake.sysIdDynamicLeftIntake(Direction.kReverse));
        sys_id_commands.addOption("Intake Left Deploy Characterize", m_intake.agitateIntakeLeft());
        sys_id_commands.addOption("Intake Right Quasi Forward", m_intake.sysIdQuasistaticRightIntake(Direction.kForward));
        sys_id_commands.addOption("Intake Right Quasi Reverse", m_intake.sysIdQuasistaticRightIntake(Direction.kReverse));
        sys_id_commands.addOption("Intake Right Dynamic Forward", m_intake.sysIdDynamicRightIntake(Direction.kForward));
        sys_id_commands.addOption("Intake Right Dynamic Reverse", m_intake.sysIdDynamicRightIntake(Direction.kReverse));
        sys_id_commands.addOption("Intake Right Deploy Characterize", m_intake.agitateIntakeRight());
        sys_id_commands.addOption("Dye Rotor Rotor Quasi Forward", m_dyeRotor.sysIdQuasistaticRotor(Direction.kForward));
        sys_id_commands.addOption("Dye Rotor Rotor Quasi Reverse", m_dyeRotor.sysIdQuasistaticRotor(Direction.kReverse));
        sys_id_commands.addOption("Dye Rotor Rotor Dynamic Forward", m_dyeRotor.sysIdDynamicRotor(Direction.kForward));
        sys_id_commands.addOption("Dye Rotor Rotor Dynamic Reverse", m_dyeRotor.sysIdDynamicRotor(Direction.kReverse));
        sys_id_commands.addOption("Dye Rotor Feeder Quasi Forward", m_dyeRotor.sysIdQuasistaticFeeder(Direction.kForward));
        sys_id_commands.addOption("Dye Rotor Feeder Quasi Reverse", m_dyeRotor.sysIdQuasistaticFeeder(Direction.kReverse));
        sys_id_commands.addOption("Dye Rotor Feeder Dynamic Forward", m_dyeRotor.sysIdDynamicFeeder(Direction.kForward));
        sys_id_commands.addOption("Dye Rotor Feeder Dynamic Reverse", m_dyeRotor.sysIdDynamicFeeder(Direction.kReverse));
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        Command test_command = sys_id_commands.get();
        if (test_command != null) {
            m_driver.start().onTrue(test_command);
        }
    }

    @Override
    public void simulationPeriodic() {
        FuelSim.getInstance().updateSim();
        Logger.recordOutput("Fuel Simulation/Blue Scored", FuelSim.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("Fuel Simulation/Red Scored", FuelSim.Hub.RED_HUB.getScore());
    }
}
