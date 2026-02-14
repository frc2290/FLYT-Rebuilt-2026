package frc.robot.subsystems.dyerotor;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DyeRotorIOSim implements DyeRotorIO {
    private final DCMotorSim rotorSim;
    private final DCMotor rotorGearbox = DCMotor.getNeoVortex(1);
    private final DCMotorSim feederSim;
    private final DCMotor feederGearbox = DCMotor.getNEO(1);

    private double rotorSpeed = 0.0;
    private double feederSpeed = 0.0;

    public DyeRotorIOSim() {
        rotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rotorGearbox, 0.025, rotorMotorReduction),
            rotorGearbox);

        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feederGearbox, 0.025, rollerMotorReduction),
            feederGearbox);
    }

    @Override
    public void updateInputs(DyeRotorIOInputs inputs) {
        rotorSim.setInputVoltage(rotorSpeed * 12.0);
        feederSim.setInputVoltage(feederSpeed * 12.0);
        rotorSim.update(0.02);
        feederSim.update(0.02);

        inputs.rotorSpeed = rotorSpeed;
        inputs.rotorAppliedVolts = rotorSim.getInputVoltage();
        inputs.rotorCurrentAmps = Math.abs(rotorSim.getCurrentDrawAmps());

        inputs.feederSpeed = feederSpeed;
        inputs.feederAppliedVolts = feederSim.getInputVoltage();
        inputs.feederCurrentAmps = Math.abs(feederSim.getCurrentDrawAmps());
    }

    @Override
    public void setRotorSpeed(double speed) {
        rotorSpeed = speed;
    }

    @Override
    public void setFeederSpeed(double speed) {
        feederSpeed = speed;
    }
}