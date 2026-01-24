package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.turret.TurretConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.VisionConstants;
import frc.utils.FuelSim;
import frc.utils.PoseEstimatorSubsystem;

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

    private FuelSim fuelSim;
    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedSupplier;
    private int fuelCount = 30;
    private double turretAngle = 0;

    public TurretIOSim(FuelSim fuelSim, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        this.fuelSim = fuelSim;
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
        double hub = turnToTarget(VisionConstants.hubCenterPose.toPose2d().getTranslation());
        turretTurnAppliedVolts = turretTurnController.calculate(turretTurnSim.getAngularPositionRad(),
                Radians.convertFrom(hub, Units.Degrees));
        turretTurnSim.setInputVoltage(MathUtil.clamp(turretTurnAppliedVolts, -12.0, 12.0));
        turretTurnSim.update(0.02);

        turretAngle = turretTurnSim.getAngularPosition().in(Units.Degrees);
        inputs.turretAngle = turretAngle;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turretTurnController.setSetpoint(rotation.getDegrees());
    }

    @Override
    public void shootFuel() {
        // if (fuelCount <= 0) return;
        fuelCount--;

        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds robotSpeed = speedSupplier.get();

        Rotation2d targetAngle = Rotation2d.fromDegrees(turretAngle);

        double magnitude = 4.0;

        Translation3d shootVelocity = new Translation3d(
                magnitude * targetAngle.getCos() + robotSpeed.vxMetersPerSecond,
                magnitude * targetAngle.getSin() + robotSpeed.vyMetersPerSecond,
                magnitude * 1.75);
        this.fuelSim.spawnFuel(
            new Translation3d(robotPose.getX(), robotPose.getY(), TurretConstants.turretHeight),
            shootVelocity);
    }

    private double turnToTarget(Translation2d target) {
        Translation2d currentTrans = poseSupplier.get().getTranslation();
        double offsetX = target.getX() - currentTrans.getX();
        double offsetY = target.getY() - currentTrans.getY();
        // return (360 - Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
        return Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360;
    }
}
