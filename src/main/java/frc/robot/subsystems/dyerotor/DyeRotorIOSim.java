package frc.robot.subsystems.dyerotor;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.dyerotor.DyeRotorIO.DyeRotorIOInputs;

public class DyeRotorIOSim implements DyeRotorIO {
    private final DCMotorSim rotorSim;
    private final DCMotor rotorGearbox = DCMotor.getNeoVortex(1);
    private final DCMotorSim rollerSim;
    private final DCMotor rollerGearbox = DCMotor.getNEO(1);

    private double rotorSpeed = 0.0;
    private double rollerSpeed = 0.0;

    public DyeRotorIOSim() {
        rotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rotorGearbox, 0.025, rotorMotorReduction),
            rotorGearbox);

        rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rollerGearbox, 0.025, rollerMotorReduction),
            rollerGearbox);
    }

    @Override
    public void updateInputs(DyeRotorIOInputs inputs) {
        rotorSim.setInputVoltage(rotorSpeed * 12.0);
        rollerSim.setInputVoltage(rollerSpeed * 12.0);
        rotorSim.update(0.02);
        rollerSim.update(0.02);

        inputs.rotorSpeed = rotorSpeed;
        inputs.rotorAppliedVolts = rotorSim.getInputVoltage();
        inputs.rotorCurrentAmps = Math.abs(rotorSim.getCurrentDrawAmps());

        inputs.rollerSpeed = rollerSpeed;
        inputs.rollerAppliedVolts = rollerSim.getInputVoltage();
        inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    }
}