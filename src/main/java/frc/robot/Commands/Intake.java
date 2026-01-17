package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

public class Intake extends Command {
    private IntakeShooter intake;

    public Intake(IntakeShooter m_intake) {
        intake = m_intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake(0.5);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
