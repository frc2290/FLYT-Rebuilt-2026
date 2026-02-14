package frc.robot.subsystems.dyerotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.dyerotor.DyeRotorIOInputsAutoLogged;

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

    public void driveRotor(double speed) {
        io.setRotorSpeed(speed);
    }

    public void driveRoller(double speed) {
        io.setRollerSpeed(speed);
    }
}
