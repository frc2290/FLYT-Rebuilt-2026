package frc.robot.subsystems.dyerotor;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.feederRunSpeed;
import static frc.robot.subsystems.dyerotor.DyeRotorConstants.rotorRunSpeed;

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
    }

    // ♿︎
    public void runDyeRotor(boolean run) {
        if (run) {
            io.setRotorSpeed(rotorRunSpeed);
            io.setFeederSpeed(feederRunSpeed);
        } else {
            io.setRotorSpeed(0);
            io.setFeederSpeed(0);
        }
    }

    public void driveRotor(double speed) {
        io.setRotorSpeed(speed);
    }

    public void driveFeeder(double speed) {
        io.setFeederSpeed(speed);
    }
}
