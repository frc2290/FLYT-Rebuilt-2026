package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.turret.Turret;

/** Characterizes flywheel velocity tracking at the tuned trench shot distance. */
public class CharacterizeTrenchShot extends Command {
    private static final String logRoot = "Shooter/TrenchCharacterization";
    private static final double trenchShotDistance = 3.0;
    private static final double defaultDurationSeconds = 6.0;
    private static final double readyTimeoutSeconds = 6.0;

    private final Turret turret;
    private final DyeRotor dyeRotor;
    private final Timer timer = new Timer();
    private final double durationSeconds;

    private double commandFlywheelVelocityMps = 0.0;
    private double commandShotAngleDeg = 0.0;
    private double maxAbsErrorMps = 0.0;
    private double sumSquaredError = 0.0;
    private int sampleCount = 0;
    private boolean feedingStarted = false;
    private boolean timedOutWaitingForReady = false;
    private boolean previousSotfEnabled = true;
    private boolean previousStopShootEnabled = false;

    public CharacterizeTrenchShot(Turret turret, DyeRotor dyeRotor) {
        this(turret, dyeRotor, defaultDurationSeconds);
    }

    public CharacterizeTrenchShot(Turret turret, DyeRotor dyeRotor, double durationSeconds) {
        this.turret = turret;
        this.dyeRotor = dyeRotor;
        this.durationSeconds = durationSeconds;
        addRequirements(this.turret, this.dyeRotor);
    }

    @Override
    public void initialize() {
        commandFlywheelVelocityMps = turret.getShotVelocityForDistanceMeters(trenchShotDistance);
        commandShotAngleDeg = turret.getShotAngleForDistanceMeters(trenchShotDistance);
        maxAbsErrorMps = 0.0;
        sumSquaredError = 0.0;
        sampleCount = 0;
        feedingStarted = false;
        timedOutWaitingForReady = false;
        previousSotfEnabled = turret.isSotfEnabled();
        previousStopShootEnabled = turret.isStopShootEnabled();

        turret.setSotfEnabled(false);
        turret.setStopShoot(false);
        turret.runManualShot(commandFlywheelVelocityMps, commandShotAngleDeg);
        dyeRotor.runDyeRotor(false);

        timer.restart();
        Logger.recordOutput(logRoot + "/MaxAbsErrorMps", 0.0);
        Logger.recordOutput(logRoot + "/RMSE", 0.0);
    }

    @Override
    public void execute() {
        if (!feedingStarted) {
            if (turret.manualShotReady(commandShotAngleDeg)) {
                feedingStarted = true;
                timer.restart();
            } else {
                if (timer.hasElapsed(readyTimeoutSeconds)) {
                    timedOutWaitingForReady = true;
                }
                dyeRotor.runDyeRotor(false);
                return;
            }
        }

        dyeRotor.runDyeRotor(true);

        double measuredFlywheelVelocity = turret.getFlywheelVelocityMetersPerSecond();
        double velocityError = commandFlywheelVelocityMps - measuredFlywheelVelocity;
        double absVelocityError = Math.abs(velocityError);

        maxAbsErrorMps = Math.max(maxAbsErrorMps, absVelocityError);
        sumSquaredError += velocityError * velocityError;
        sampleCount++;

        double mse = sampleCount > 0 ? sumSquaredError / sampleCount : 0.0;
        double rmse = Math.sqrt(mse);
        Logger.recordOutput(logRoot + "/MaxAbsErrorMps", maxAbsErrorMps);
        Logger.recordOutput(logRoot + "/RMSE", rmse);
    }

    @Override
    public void end(boolean interrupted) {
        dyeRotor.runDyeRotor(false);
        turret.setSotfEnabled(previousSotfEnabled);
        turret.setStopShoot(previousStopShootEnabled);
        if (!previousSotfEnabled) {
            turret.setShooterSpeed(0.0);
            turret.setHoodAngle(0.0);
        }

        double mse = sampleCount > 0 ? sumSquaredError / sampleCount : 0.0;
        double rmse = Math.sqrt(mse);
        Logger.recordOutput(logRoot + "/MaxAbsErrorMps", maxAbsErrorMps);
        Logger.recordOutput(logRoot + "/RMSE", rmse);

        String result = String.format(
                "Trench shot characterization %s | time=%.2fs samples=%d cmd=%.3f mps maxErr=%.3f mps rmse=%.4f",
                interrupted ? "interrupted" : "complete",
                timer.get(),
                sampleCount,
                commandFlywheelVelocityMps,
                maxAbsErrorMps,
                rmse);
        DriverStation.reportWarning(result, false);
        if (timedOutWaitingForReady) {
            DriverStation.reportWarning(
                    "Trench shot characterization timed out waiting for shooter ready state; no feed samples were collected.",
                    false);
        } else if (!feedingStarted) {
            DriverStation.reportWarning(
                    "Trench shot characterization ended before shooter reached ready state; no feed samples were collected.",
                    false);
        }
    }

    @Override
    public boolean isFinished() {
        if (!feedingStarted) {
            return timedOutWaitingForReady;
        }
        return timer.hasElapsed(durationSeconds);
    }
}
