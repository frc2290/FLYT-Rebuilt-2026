// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeXtakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
    private IntakeXtakeSubsystem intake;
    private double speed;
    private Timer shootWait = new Timer();

    /** Creates a new Intake. */
    public Shoot(IntakeXtakeSubsystem m_intake, double m_speed) {
        intake = m_intake;
        speed = m_speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shootWait.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!shootWait.isRunning()) {
            shootWait.restart();
            intake.shooterMotor(speed);
        } else if (shootWait.hasElapsed(0.5) && shootWait.isRunning()) {
            intake.hopperMotor(sp);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.shoot(0);
        shootWait.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
