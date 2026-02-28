package frc.robot.subsystems.intake;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.utils.SparkUtil.ifOk;
import static frc.utils.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;

public class IntakeIOSpark implements IntakeIO {
    private final SparkBase driveSpark;
    private final SparkBase deploySpark;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder deployEncoder;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController deployController;

    public IntakeIOSpark(IntakeSide side) {
        driveSpark = new SparkMax(
                switch (side) {
                    case LEFT -> leftDriveCanId;
                    case RIGHT -> rightDriveCanId;
                }, MotorType.kBrushless);

        deploySpark = new SparkFlex(
                switch (side) {
                    case LEFT -> leftDeployCanId;
                    case RIGHT -> rightDeployCanId;
                }, MotorType.kBrushless);

        driveEncoder = driveSpark.getEncoder();
        deployEncoder = deploySpark.getAbsoluteEncoder();
        driveController = driveSpark.getClosedLoopController();
        deployController = deploySpark.getClosedLoopController();

        var driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig.encoder
                .positionConversionFactor(rollerEncoderPositionFactor)
                .velocityConversionFactor(rollerEncoderVelocityFactor)
                .uvwAverageDepth(2);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(rollerKp, rollerKi, rollerKd);
        driveConfig.closedLoop.feedForward.kV(rollerKv);
        // driveConfig.signals
        // .appliedOutputPeriodMs(20)
        // .busVoltagePeriodMs(20)
        // .outputCurrentPeriodMs(20);

        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        var deployConfig = new SparkFlexConfig();
        deployConfig
                .inverted(deployInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(deployMotorCurrentLimit)
                .voltageCompensation(12.0);
        deployConfig.absoluteEncoder
                .inverted(deployEncoderInverted)
                .positionConversionFactor(deployEncoderPositionFactor)
                .velocityConversionFactor(deployEncoderVelocityFactor)
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        deployConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(deployKp, deployKi, deployKd);
        // driveConfig.signals
        // .absoluteEncoderPositionAlwaysOn(true)
        // .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        // .absoluteEncoderVelocityAlwaysOn(true)
        // .absoluteEncoderVelocityPeriodMs(20)
        // .appliedOutputPeriodMs(20)
        // .busVoltagePeriodMs(20)
        // .outputCurrentPeriodMs(20);
        tryUntilOk(
                deploySpark,
                5,
                () -> deploySpark.configure(
                        deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update drive inputs
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveSpeed = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);

        // Update deploy inputs
        ifOk(
                deploySpark,
                deployEncoder::getPosition,
                (value) -> inputs.deployPosition = Rotation2d.fromDegrees(value));
        ifOk(
                deploySpark,
                deployEncoder::getVelocity,
                (value) -> inputs.deployVelocityRadPerSec = degreesToRadians(value));
        ifOk(
                deploySpark,
                new DoubleSupplier[] { deploySpark::getAppliedOutput, deploySpark::getBusVoltage },
                (values) -> inputs.deployAppliedVolts = values[0] * values[1]);
        ifOk(deploySpark, deploySpark::getOutputCurrent, (value) -> inputs.deployCurrentAmps = value);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        driveController.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void setDeployPosition(Rotation2d rotation) {
        deployController.setSetpoint(rotation.getDegrees(), ControlType.kPosition);
    }

    public double getPosition() {
        return deployEncoder.getPosition();
    }
}
