package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim driveSim;
    private final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
    private final DCMotorSim deploySim;
    private final DCMotor deployGearbox = DCMotor.getNEO(1);

    private boolean driveClosedLoop = false;
    private boolean deployClosedLoop = false;
    private PIDController driveController = new PIDController(driveSimP, 0, 0);
    private PIDController deployController = new PIDController(deploySimP, 0, 0);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double deployAppliedVolts = 0.0;

    public IntakeIOSim() {
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
                driveGearbox);
        deploySim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(deployGearbox, 0.025, deployMotorReduction),
                deployGearbox);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (deployClosedLoop) {
            deployAppliedVolts = deployController.calculate(deploySim.getAngularPositionRad());
        } else {
            deployController.reset();
        }

        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        deploySim.setInputVoltage(MathUtil.clamp(deployAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        deploySim.update(0.02);

        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.deployConnected = true;
        inputs.deployPosition = new Rotation2d(deploySim.getAngularPositionRad());
        inputs.deployVelocityRadPerSec = deploySim.getAngularVelocityRadPerSec();
        inputs.deployAppliedVolts = deployAppliedVolts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
    }

    @Override
    public void setIntakeVelocity(double vel) {
        driveClosedLoop = true;
        driveFFVolts = driveSimKs * Math.signum(vel) + driveSimKv * vel;
        driveController.setSetpoint(vel);
    }

    @Override
    public void setDeployPosition(Rotation2d rotation) {
        deployClosedLoop = true;
        deployController.setSetpoint(rotation.getRadians());
    }
}
