// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.ballsPerRotation;
import static frc.robot.subsystems.dyerotor.DyeRotorConstants.defaultTargetBps;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.dyerotor.DyeRotor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunDyeRotor extends Command {
    private static final double JAM_DETECT_SECONDS = 0.5;
    private static final double BACKDRIVE_SECONDS = 0.25;
    private static final double AT_SPEED_RATIO = 0.95;

    private final DyeRotor dyeRotor;
    private final Timer jamTimer = new Timer();
    private final Timer backdriveTimer = new Timer();

    private enum RotorState {
        FORWARD,
        BACKDRIVE
    }

    private RotorState rotorState = RotorState.FORWARD;

    /** Creates a new RunDyeRotor. */
    public RunDyeRotor(DyeRotor dyeRotor) {
        this.dyeRotor = dyeRotor;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.dyeRotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        jamTimer.reset();
        jamTimer.stop();
        backdriveTimer.reset();
        backdriveTimer.stop();
        rotorState = RotorState.FORWARD;
        dyeRotor.runDyeRotor(true);
        dyeRotor.setTargetBPS(defaultTargetBps);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (rotorState == RotorState.BACKDRIVE) {
            dyeRotor.setTargetBPS(-defaultTargetBps);
            if (backdriveTimer.hasElapsed(BACKDRIVE_SECONDS)) {
                backdriveTimer.stop();
                backdriveTimer.reset();
                jamTimer.stop();
                jamTimer.reset();
                rotorState = RotorState.FORWARD;
                dyeRotor.setTargetBPS(defaultTargetBps);
            }
            return;
        }

        double targetRotorRpm = (defaultTargetBps * 60.0) / ballsPerRotation;
        double measuredRotorRpm = Math.abs(dyeRotor.getRotorSpeed());
        boolean atTargetSpeed = measuredRotorRpm >= (targetRotorRpm * AT_SPEED_RATIO);

        if (!atTargetSpeed) {
            if (!jamTimer.isRunning()) {
                jamTimer.reset();
                jamTimer.start();
            }

            if (jamTimer.hasElapsed(JAM_DETECT_SECONDS)) {
                jamTimer.stop();
                jamTimer.reset();
                backdriveTimer.reset();
                backdriveTimer.start();
                rotorState = RotorState.BACKDRIVE;
                dyeRotor.setTargetBPS(-defaultTargetBps);
            }
        } else {
            jamTimer.stop();
            jamTimer.reset();
            dyeRotor.setTargetBPS(defaultTargetBps);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        dyeRotor.runDyeRotor(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
