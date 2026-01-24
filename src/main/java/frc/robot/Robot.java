// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Coordinator;
import frc.robot.subsystems.Coordinator.RobotState;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.utils.PoseEstimatorSubsystem;

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

    /**
     * Cached reference to the primary driver controller so subsystems can read
     * axes.
     */
    private XboxController m_driver = new XboxController(0);

    // The robot's subsystems.
    /** Owns all hardware for swerve driving and exposes the drive commands. */
    // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Drive m_robotDrive;
    private final IntakeShooter m_intakeShooter = new IntakeShooter();
    private final PoseEstimatorSubsystem m_poseEstimator;

    /** Coordinates all autonomous and teleop driving modes. */
    private final DriveStateMachine m_driveStateMachine;

    /**
     * Central coordinator that keeps drive and manipulator state machines in sync.
     */
    private final Coordinator m_coordinator;

    public Robot() {
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
        case REAL:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
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
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_robotDrive = new Drive(
                               new GyroIO() {},
                               new ModuleIOSim(),
                               new ModuleIOSim(),
                               new ModuleIOSim(),
                               new ModuleIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                m_robotDrive = new Drive(
                               new GyroIO() {},
                               new ModuleIO() {},
                               new ModuleIO() {},
                               new ModuleIO() {},
                               new ModuleIO() {});
                break;
        }

        m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);
        m_driveStateMachine = new DriveStateMachine(m_robotDrive, m_poseEstimator,
            m_driver);
        m_coordinator = new Coordinator(m_driveStateMachine);
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
                m_intakeShooter,
                m_poseEstimator,
                m_driveStateMachine,
                m_coordinator,
                m_driver);

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
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // m_coordinator.robotDisabled(true);
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        m_coordinator.robotAuto(true);
        m_coordinator.robotDisabled(false);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    public void teleopInit() {

        m_coordinator.robotAuto(false);
        m_coordinator.robotDisabled(false);

        m_coordinator.setRobotGoal(RobotState.START_POSITION);

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
    }

    @Override
    public void testInit() {

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
