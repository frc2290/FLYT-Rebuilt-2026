package frc.robot.subsystems.dyerotor;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase {
    private final DyeRotorIO io;
    private final DyeRotorIOInputsAutoLogged inputs = new DyeRotorIOInputsAutoLogged();

    public DyeRotor(DyeRotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DyeRotor", inputs);
    }

    // ♿︎
    public void runDyeRotor(boolean run) {
        if (run) {
            setTargetBPS(defaultTargetBps);
        } else {
            io.setRotorSpeed(0);
            io.setFeederSpeed(0);
        }
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
}
