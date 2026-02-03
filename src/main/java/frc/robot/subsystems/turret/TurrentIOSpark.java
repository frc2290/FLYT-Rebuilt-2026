package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.utils.SparkUtil.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurrentIOSpark implements TurretIO {

    // Harware objects
    private final SparkBase turretSpark;
    private final SparkBase hoodSpark; 
    private final SparkBase flywheel1Spark; //later change to bottom/top
    private final SparkBase flywheel2Spark;

    // Fo turret
    private RelativeEncoder turnRelEncoder; // Experimental for using one encoder for two things at the same time
    //private AbsoluteEncoder turnEncoder;
    private DutyCycleEncoder turnEncoder1;
    private DutyCycleEncoder turnEncoder2;
    //private AbsoluteEncoder turnEncoder2;
    private AbsoluteEncoder hoodEncoder;
    private RelativeEncoder flywheel1Encoder;
    private RelativeEncoder flywheel2Encoder;

    // Closed loop controllers
    private final SparkClosedLoopController turretController;
    private final SparkClosedLoopController hoodController;
    private final SparkClosedLoopController flywheelController1; 
    private final SparkClosedLoopController flywheelController2; 

    public TurrentIOSpark() {

        // Create motor controllers
        turretSpark = new SparkMax(turretCanId, MotorType.kBrushless);
        hoodSpark = new SparkMax(hoodCanId, MotorType.kBrushless);
        flywheel1Spark = new SparkFlex(flywheel1CanId, MotorType.kBrushless);
        flywheel2Spark = new SparkFlex(flywheel2CanId, MotorType.kBrushless);
        
        // Setup encoders
        flywheel2Encoder = flywheel1Spark.getEncoder();
        flywheel2Encoder = flywheel2Spark.getEncoder();
        //turnRelEncoder = turretSpark.getEncoder(); EXPERIMENTAL HYBRID
        hoodEncoder = hoodSpark.getAbsoluteEncoder();

        // Setup RoboRio Absalute encoders
        // Gets if the encoder is connected, IMPLEMENT LATER
        //m_encoder.isConnected();
         // Initializes a duty cycle encoder on DIO pins 0 to return a value of 4 for
        // a full rotation, with the encoder reporting 0 half way through rotation (2
        // out of 4)
        turnEncoder1 = new DutyCycleEncoder(0, 360, 0);
        turnEncoder2 = new DutyCycleEncoder(0, 360, 0);

        /*In 2025 the API changed to remove rollover detection as rollover 
        detection did not work. The get() method returns the value within a
        rotation where the maximum value in a rotation is defined in the constructor\
        (default 1).*/

        // Setup Controllers
        turretController = turretSpark.getClosedLoopController();
        hoodController = hoodSpark.getClosedLoopController();
        flywheelController1 = flywheel1Spark.getClosedLoopController();
        flywheelController2 = flywheel2Spark.getClosedLoopController();

        // Turret parameters
        var turretConfig = new SparkMaxConfig();
        turretConfig
            .inverted(turretIsInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(turretMotorCurrent)
            .voltageCompensation(12.0);
        turretConfig
            .encoder
            .positionConversionFactor(turretEncoderPositionFactor)
            .velocityConversionFactor(turretEncoderVelocityFactor)
            .uvwAverageDepth(2);
        turretConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(turretKp, turretKi, turretKd);
        tryUntilOk(
            turretSpark,
        5,
            () ->
                turretSpark.configure(
                    turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Hood Config
        var hoodConfig = new SparkMaxConfig();
        hoodConfig
            .inverted(hoodIsInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(hoodMotorCurrent)
            .voltageCompensation(12.0);
        hoodConfig
            .absoluteEncoder
            .inverted(hoodEncoderInverted)
            .positionConversionFactor(hoodEncoderPositionFactor)
            .velocityConversionFactor(hoodEncoderVelocityFactor)
            .averageDepth(2);
        hoodConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(hoodKp, hoodKi, hoodKd);
        tryUntilOk(
            hoodSpark,
        5,
            () ->
                hoodSpark.configure(
                    hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));


        // Flywheel config
        var flywheelConfig = new SparkMaxConfig();
        flywheelConfig
            .inverted(flywheelIsInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(flywheelMotorCurrent)
            .voltageCompensation(12.0);
        flywheelConfig
            .encoder
            .positionConversionFactor(flywheelEncoderPositionFactor)
            .velocityConversionFactor(flywheelEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        flywheelConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(flywheelKp, flywheelKi, flywheelKd);
        tryUntilOk(
            flywheel1Spark,
        5,
            () ->
                flywheel1Spark.configure(
                    flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
            flywheel2Spark,
        5,
            () ->
                flywheel2Spark.configure(
                    flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }


    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // double hub =
        // turnToTarget(VisionConstants.hubCenterPose.toPose2d().getTranslation());
        // turretTurnAppliedVolts =
        // turretTurnController.calculate(turretTurnSim.getAngularPositionRad(),
        // Radians.convertFrom(hub, Units.Degrees));
        // turretTurnAppliedVolts = turretTurnController.calculate(turretTurnSim.getAngularPositionRad());
        // turretTurnSim.setInputVoltage(MathUtil.clamp(turretTurnAppliedVolts, -12.0, 12.0));
        // turretTurnSim.update(0.02);

        // turretAngle = turretTurnSim.getAngularPosition().in(Units.Degrees);
        // inputs.turretAngle = turretAngle;
        // inputs.turretSpeed = turretSpeed;
        // inputs.turretHoodAngle = turretHoodAngle;
        // inputs.turretAngleSetpoint = turretAngleSetpoint;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        // turretTurnController.setSetpoint(rotation.getRadians());
        // turretAngleSetpoint = rotation.getDegrees();
    }

    @Override
    public void setHoodAngle(double angle) {
        // turretHoodAngle = angle;
    };

    @Override
    public void setShooterSpeed(double speed) {
        // turretSpeed = speed;
    };

    @Override
    public void shootFuel() {
        // if (fuelCount <= 0) return;
        // fuelCount++;
        // Logger.recordOutput("Fuel Shot", fuelCount);


        // Pose2d robotPose = poseSupplier.get();
        // ChassisSpeeds robotSpeed = speedSupplier.get();

        // double yawRad = Math.toRadians(turretAngle);
        // double pitchRad = Math.toRadians(turretHoodAngle);

        // Translation3d velocity = new Translation3d(
        //         turretSpeed * Math.cos(pitchRad) * Math.cos(yawRad) + robotSpeed.vxMetersPerSecond, // X (forward)
        //         turretSpeed * Math.cos(pitchRad) * Math.sin(yawRad) + robotSpeed.vyMetersPerSecond, // Y (left)
        //         turretSpeed * Math.sin(pitchRad) // Z (up)
        // );

        // this.fuelSim.spawnFuel(
        //         new Translation3d(robotPose.getX(), robotPose.getY(), TurretConstants.turretHeight),
        //         velocity);
    }

       
}
