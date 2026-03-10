package frc.robot.subsystems.intake;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.utils.SparkUtil.ifOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
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

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;

public class IntakeIOSpark implements IntakeIO {
    private final SparkBase driveSpark;
    private final SparkBase deploySpark;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder deployEncoder;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController deployController;

    private IntakeSide side;

    private double deploySetpoint = 0.0;

    public IntakeIOSpark(IntakeSide side, boolean inverted, double zeroOffset) {
        this.side = side;
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
                .idleMode(IdleMode.kBrake)
                .inverted(inverted)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(rollerEncoderPositionFactor)
                .velocityConversionFactor(rollerEncoderVelocityFactor)
                .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(rollerKp, rollerKi, rollerKd);
        driveConfig.closedLoop.feedForward.kV(rollerKv);
        // driveConfig.signals
        // .appliedOutputPeriodMs(20)
        // .busVoltagePeriodMs(20)
        // .outputCurrentPeriodMs(20);

        REVLibError driveErr = driveSpark.configure(
                driveConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (driveErr != REVLibError.kOk) {
                System.err.println("INTAKE " + side + " DRIVE CONFIG FAILED: " + driveErr.name());
        }

        var deployConfig = new SparkFlexConfig();
        deployConfig
                .inverted(inverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(deployMotorCurrentLimit)
                .voltageCompensation(12.0);
        deployConfig
                .absoluteEncoder
                .inverted(inverted)
                .zeroOffset(zeroOffset)
                .positionConversionFactor(deployEncoderPositionFactor)
                .velocityConversionFactor(deployEncoderVelocityFactor)
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        deployConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(deployKp, deployKi, deployKd);
        deployConfig.closedLoop
                .maxMotion
                // Calculated for a 0.5s, 86-degree move
                .cruiseVelocity(350)
                .maxAcceleration(1400)
                // Keep this loose during tuning to prevent premature profile regeneration
                .allowedProfileError(20);

        deployConfig.closedLoop.feedForward.kV(deployKv);
        deployConfig.closedLoop.feedForward.kA(0);
        // driveConfig.signals
        // .absoluteEncoderPositionAlwaysOn(true)
        // .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        // .absoluteEncoderVelocityAlwaysOn(true)
        // .absoluteEncoderVelocityPeriodMs(20)
        // .appliedOutputPeriodMs(20)
        // .busVoltagePeriodMs(20)
        // .outputCurrentPeriodMs(20);
        REVLibError deployErr = deploySpark.configure(
                deployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (deployErr != REVLibError.kOk) {
                System.err.println("INTAKE " + side + " DEPLOY CONFIG FAILED: " + deployErr.name());
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        deployController.setSetpoint(deploySetpoint + getZeroOffsetAdj(), ControlType.kMAXMotionPositionControl);

        // Update drive inputs
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionMeters = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveSpeed = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);

        // Update deploy inputs
        ifOk(
                deploySpark,
                () -> getPosition(),
                (value) -> inputs.deployPosition = value);
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
    public void setIntakeVoltage(double volts) {
        driveSpark.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setDeployPosition(double angle) {
        this.deploySetpoint = angle;
    }

    public double getPosition() {
        switch (this.side) {
            case LEFT:
                return deployEncoder.getPosition() - leftZeroOffsetAdj;
            case RIGHT:
                return deployEncoder.getPosition() - rightZeroOffsetAdj;
            default:
                return 0;
        }
    }

    private double getZeroOffsetAdj() {
        switch (this.side) {
            case LEFT:
                return leftZeroOffsetAdj;
            case RIGHT:
                return rightZeroOffsetAdj;
            default:
                return 0;
        }
    }
}
