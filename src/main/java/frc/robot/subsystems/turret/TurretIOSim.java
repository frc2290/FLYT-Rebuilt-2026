package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.FuelSim;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim turretTurnSim;
    private final DCMotor turretTurnGearbox = DCMotor.getNeoVortex(1);
    private final DCMotorSim turretShootSim;
    private final DCMotor turretShootGearbox = DCMotor.getNeoVortex(1);
    private final DCMotorSim turretHoodSim;
    private final DCMotor turretHoodGearbox = DCMotor.getNeoVortex(1);
    private final PIDController turretTurnController = new PIDController(turretTurnSimP, 0, 0);
    private final PIDController turretShootController = new PIDController(turretShootSimP, 0, 0);
    private final PIDController turretHoodController = new PIDController(turretHoodSimP, 0, 0);

    private double turretTurnAppliedVolts = 0.0;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedSupplier;
    private int fuelCount = 8;
    private double turretAngle = 0;
    private double turretSpeed = 0;
    private double turretHoodAngle = 0;
    private double turretAngleSetpoint = 0;
    private double shooterVoltageCommand = 0.0;

    public TurretIOSim(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;

        turretTurnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(turretTurnGearbox, 0.025, turretTurnReduction),
                turretTurnGearbox);
        turretShootSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(turretShootGearbox, 0.025, turretShootReduction),
                turretShootGearbox);
        turretHoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(turretHoodGearbox, 0.025, turretHoodReduction),
                turretHoodGearbox);

        turretTurnController.enableContinuousInput(0, 360);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        Logger.recordOutput("Fuel", fuelCount);

        // double hub =
        // turnToTarget(VisionConstants.hubCenterPose.toPose2d().getTranslation());
        // turretTurnAppliedVolts =
        // turretTurnController.calculate(turretTurnSim.getAngularPositionRad(),
        // Radians.convertFrom(hub, Units.Degrees));
        turretTurnAppliedVolts = turretTurnController.calculate(turretTurnSim.getAngularPositionRad());
        turretTurnSim.setInputVoltage(MathUtil.clamp(turretTurnAppliedVolts, -12.0, 12.0));
        turretTurnSim.update(0.02);
        turretShootSim.setInputVoltage(MathUtil.clamp(shooterVoltageCommand, -12.0, 12.0));
        turretShootSim.update(0.02);

        turretAngle = turretTurnSim.getAngularPosition().in(Units.Degrees);
        inputs.turretAngle = turretAngle;
        inputs.turretSpeed = turretSpeed;
        inputs.turretHoodAngle = turretHoodAngle;
        inputs.turretAngleSetpoint = turretAngleSetpoint;
        inputs.flywheelPositionMeters =
                (turretShootSim.getAngularPositionRad() / (2.0 * Math.PI)) * flywheelEncoderPositionFactor;
        inputs.flywheelVelocity = turretShootSim.getAngularVelocityRPM() * flywheelEncoderVelocityFactor;
        inputs.flywheelAppliedVolts = shooterVoltageCommand;
        inputs.flywheelCurrentAmps = Math.abs(turretShootSim.getCurrentDrawAmps());
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turretTurnController.setSetpoint(rotation.getRadians());
        turretAngleSetpoint = rotation.getDegrees();
    }

    @Override
    public void setHoodAngle(double angle) {
        turretHoodAngle = angle;
    };

    @Override
    public void setShooterSpeed(double speed) {
        turretSpeed = speed;
        shooterVoltageCommand = MathUtil.clamp(turretSpeed * flywheelKv, -12.0, 12.0);
    };

    @Override
    public void setShooterVoltage(double volts) {
        shooterVoltageCommand = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public boolean flywheelAtSpeed() {
        return true;
    }

    public void simIntake() {
        fuelCount++;
    }

    @Override
    public void shootFuel() {
        if (fuelCount <= 0) return;
        fuelCount--;

        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds robotSpeed = speedSupplier.get();

        double yawRad = Math.toRadians(turretAngle);
        double pitchRad = Math.toRadians(turretHoodAngle);

        Translation3d velocity = new Translation3d(
                turretSpeed * Math.cos(pitchRad) * Math.cos(yawRad) + robotSpeed.vxMetersPerSecond, // X (forward)
                turretSpeed * Math.cos(pitchRad) * Math.sin(yawRad) + robotSpeed.vyMetersPerSecond, // Y (left)
                turretSpeed * Math.sin(pitchRad) // Z (up)
        );

        FuelSim.getInstance().spawnFuel(
                new Translation3d(robotPose.getX(), robotPose.getY(), TurretConstants.turretHeight),
                velocity);
    }
}
