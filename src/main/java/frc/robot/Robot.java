// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeXtakeSubsystem;
import frc.robot.subsystems.StateMachineCoordinator;
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
public class Robot extends TimedRobot {
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
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final IntakeXtakeSubsystem m_intakeShooter = new IntakeXtakeSubsystem();
    private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);

    /** Coordinates all autonomous and teleop driving modes. */
    private final DriveStateMachine m_driveStateMachine =
      new DriveStateMachine(m_robotDrive, m_poseEstimator, m_driver);

    /** Central coordinator that keeps drive and manipulator state machines in sync. */
    private final StateMachineCoordinator m_coordinator =
      new StateMachineCoordinator(m_driveStateMachine);

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

        URCL.start();

        // If logging only to DataLog.
        URCL.start(DataLogManager.getLog());

        SmartDashboard.putData(CommandScheduler.getInstance());
        
        //Display subystem satatus
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
