package frc.robot.subsystems.dyerotor;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase {
    private static final double JAM_DETECT_SECONDS = 0.25;
    private static final double BACKDRIVE_SECONDS = 0.25;
    private static final double AT_SPEED_RATIO = 0.95;

    private final DyeRotorIO io;
    private final DyeRotorIOInputsAutoLogged inputs = new DyeRotorIOInputsAutoLogged();

    private final Timer jamTimer = new Timer();
    private final Timer backdriveTimer = new Timer();

    private boolean runDyeRotor = false;
    private boolean backdriving = false;

    public DyeRotor(DyeRotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DyeRotor", inputs);

        if (!runDyeRotor) {
            io.setRotorSpeed(0);
            io.setFeederSpeed(0);
            return;
        }

        if (backdriving) {
            setTargetBPS(-defaultTargetBps);
            if (backdriveTimer.hasElapsed(BACKDRIVE_SECONDS)) {
                backdriving = false;
                backdriveTimer.stop();
                backdriveTimer.reset();
                jamTimer.stop();
                jamTimer.reset();
            }
            return;
        }

        double targetRotorRpm = (defaultTargetBps * 60.0) / ballsPerRotation;
        boolean atTargetSpeed = Math.abs(inputs.rotorEncoderRPM) >= (targetRotorRpm * AT_SPEED_RATIO);

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
                backdriving = true;
            } else {
                setTargetBPS(defaultTargetBps);
            }
        } else {
            jamTimer.stop();
            jamTimer.reset();
            setTargetBPS(defaultTargetBps);
        }
    }

    public void runDyeRotor(boolean run) {
        this.runDyeRotor = run;
        // backdriving = false;
        // jamTimer.stop();
        // jamTimer.reset();
        // backdriveTimer.stop();
        // backdriveTimer.reset();

        // if (!run) {
        //     io.setRotorSpeed(0);
        //     io.setFeederSpeed(0);
        // }
    }

    public Command runDyeRotorCommand(boolean run) {
        if (run) {
            return runDyeRotorAutoCommand();
        }
        return Commands.runOnce(() -> runDyeRotor(false), this);
    }

    public Command runDyeRotorAutoCommand() {
        return Commands.startEnd(() -> runDyeRotor(true), () -> runDyeRotor(false), this);
    }

    /**
     * Calculates and commands the motor speeds required to hit a target throughput.
     *
     * @param targetBPS target balls per second
     */
    public void setTargetBPS(double targetBPS) {
        // 1) DYE ROTOR KINEMATICS
        double rotorSpeed = (targetBPS * 60.0) / ballsPerRotation;

        // 2) FEED WHEEL KINEMATICS (PURE ROLLING + OVERFEED)
        double feedMultiplier =
                (ballsPerRotation * fuelDiameterInches * overfeedRatio) / (Math.PI * feedWheelRadiusInches);
        double feedSpeedAbs = rotorSpeed * feedMultiplier;
        double feedSpeed = feedSpeedAbs - rotorSpeed;

        // 3) COMMAND MECHANISM RPM DIRECTLY
        // Spark encoder conversion factors apply motor->mechanism gear ratio scaling.
        io.setRotorSpeed(rotorSpeed);
        io.setFeederSpeed(feedSpeed);
    }

    public void driveRotor(double speed) {
        io.setRotorSpeed(speed);
    }

    public void driveFeeder(double speed) {
        io.setFeederSpeed(speed);
    }

    public double getRotorSpeed() {
        return inputs.rotorEncoderRPM;
    }

    public double getFeederSpeed() {
        return inputs.feederEncoderRPM;
    }
}
