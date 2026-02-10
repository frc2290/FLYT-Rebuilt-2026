package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.utils.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOSpark implements TurretIO {

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

    //

    // Global variables
    private double turretAngle = 0;
    private double turretSpeed = 0;
    private double turretHoodAngle = 0;
    private double turretAngleSetpoint = 0;

    public TurretIOSpark() {

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


    // Right now it is stupid but here we going to put
    // Absolute dual encoder position of turret estimator master piece
    // Also I think this function should be run only once to set position for rerlative encoder
    private double getTurretPosAtStart(){
        return turnEncoder1.get()+turnEncoder1.get();
    }


    private double getTurretPos(){
        return turnRelEncoder.getPosition();
    }
    private double getTurretVel(){
        return turnRelEncoder.getVelocity();
    }



    // I think this is used for logger pro to keep track of things?
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turretAngle = getTurretPos();
        inputs.turretAngle = turretAngle;
        turretSpeed = getTurretVel();
        inputs.turretSpeed = turretSpeed;
        turretHoodAngle = hoodEncoder.getPosition();
        inputs.turretHoodAngle = turretHoodAngle;
        inputs.turretAngleSetpoint = turretAngleSetpoint;
    }


    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turretController.setSetpoint(rotation.getDegrees(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, turretff);
        turretAngleSetpoint = rotation.getDegrees();
    }


    @Override
    public void setHoodAngle(double angle) {
        hoodController.setSetpoint(angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, hoodff);
        turretHoodAngle = angle;
    };

    @Override
    public void setShooterSpeed(double speed) {
        flywheelController1.setSetpoint(speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, shooterff);
        turretSpeed = speed;
    };

    @Override
    public void shootFuel() {

        //Code for making turret shoot fuel to position?


    }

       
}
