package frc.robot.subsystems.intake;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.utils.FuelSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim driveSim;
    private final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
    private final DCMotorSim deploySim;
    private final DCMotor deployGearbox = DCMotor.getNEO(1);

    private PIDController deployController = new PIDController(deploySimKp, deploySimKi, deploySimKd);
    private double deployAppliedVolts = 0.0;
    private double driveSpeed = 0.0;
    private double driveCommandedVolts = 0.0;

    public IntakeIOSim(IntakeSide side, TurretIOSim turret) {
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
                driveGearbox);
        deploySim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(deployGearbox, 0.025, deployMotorReduction),
                deployGearbox);

        FuelSim fuelSim = FuelSim.getInstance();
        
        if (side == IntakeSide.LEFT) {
            fuelSim.registerIntake(
                    -inchesToMeters(15), // length, min x
                    inchesToMeters(15), // length, max x
                    inchesToMeters(15), // width, min y
                    inchesToMeters(15 + 12), // width, max y
                    () -> radiansToDegrees(deploySim.getAngularPositionRad()) > outPosition - positionBuffer,
                    turret::simIntake);
        } else {
            fuelSim.registerIntake(
                    -inchesToMeters(15), // length, min x
                    inchesToMeters(15), // length, max x
                    -inchesToMeters(15 + 12), // width, min y
                    -inchesToMeters(15), // width, max y
                    () -> radiansToDegrees(deploySim.getAngularPositionRad()) > outPosition - positionBuffer,
                    turret::simIntake);
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        deployAppliedVolts = deployController.calculate(deploySim.getAngularPositionRad());

        driveSim.setInputVoltage(driveCommandedVolts);
        deploySim.setInputVoltage(MathUtil.clamp(deployAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        deploySim.update(0.02);

        inputs.drivePositionMeters = (driveSim.getAngularPositionRad() / (2.0 * Math.PI)) * rollerEncoderPositionFactor;
        inputs.driveSpeed = driveSim.getAngularVelocityRPM() * rollerEncoderVelocityFactor;
        inputs.driveAppliedVolts = driveCommandedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.deployPosition = deploySim.getAngularPosition().in(Degrees);
        inputs.deployVelocityRadPerSec = deploySim.getAngularVelocityRadPerSec();
        inputs.deployAppliedVolts = deployAppliedVolts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
    }

    @Override
    public void setIntakeSpeed(double speed) {
        driveSpeed = speed;
        driveCommandedVolts = MathUtil.clamp(driveSpeed * rollerKv, -12.0, 12.0);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        driveSpeed = 0.0;
        driveCommandedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setDeployPosition(double angle, boolean useProfile) {
        deployController.setSetpoint(degreesToRadians(angle));
    }
}
