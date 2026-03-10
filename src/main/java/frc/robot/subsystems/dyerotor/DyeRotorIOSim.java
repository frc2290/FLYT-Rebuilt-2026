package frc.robot.subsystems.dyerotor;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import edu.wpi.first.math.MathUtil;
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
    private double rotorCommandedVolts = 0.0;
    private double feederCommandedVolts = 0.0;

    public DyeRotorIOSim() {
        rotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rotorGearbox, 0.025, rotorGearRatio),
            rotorGearbox);

        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feederGearbox, 0.025, feedGearRatio),
            feederGearbox);
    }

    @Override
    public void updateInputs(DyeRotorIOInputs inputs) {
        // Feedforward speed targets and direct-voltage commands both resolve to this voltage path.
        rotorSim.setInputVoltage(rotorCommandedVolts);
        feederSim.setInputVoltage(feederCommandedVolts);
        rotorSim.update(0.02);
        feederSim.update(0.02);

        inputs.rotorSpeed = rotorSpeed;
        inputs.rotorEncoderPosition = (rotorSim.getAngularPositionRad() / (2.0 * Math.PI)) * rotorEncoderPositionFactor;
        inputs.rotorAppliedVolts = rotorCommandedVolts;
        inputs.rotorEncoderRPM = rotorSim.getAngularVelocityRPM();
        inputs.rotorCurrentAmps = Math.abs(rotorSim.getCurrentDrawAmps());

        inputs.feederSpeed = feederSpeed;
        inputs.feederEncoderPosition =
                (feederSim.getAngularPositionRad() / (2.0 * Math.PI)) * feederEncoderPositionFactor;
        inputs.feederAppliedVolts = feederCommandedVolts;
        inputs.feederEncoderRPM = feederSim.getAngularVelocityRPM();
        inputs.feederCurrentAmps = Math.abs(feederSim.getCurrentDrawAmps());
    }

    @Override
    public void setRotorSpeed(double speed) {
        rotorSpeed = speed;
        rotorCommandedVolts = MathUtil.clamp(rotorSpeed * rotorKv, -12.0, 12.0);
    }

    @Override
    public void setFeederSpeed(double speed) {
        feederSpeed = speed;
        feederCommandedVolts = MathUtil.clamp(feederSpeed * feederKv, -12.0, 12.0);
    }

    @Override
    public void setRotorVoltage(double volts) {
        rotorSpeed = 0.0;
        rotorCommandedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setFeederVoltage(double volts) {
        feederSpeed = 0.0;
        feederCommandedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
