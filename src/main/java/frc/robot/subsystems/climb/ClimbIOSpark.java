package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.utils.SparkUtil.*;

import static frc.robot.subsystems.climb.ClimbConstants.*;

public class ClimbIOSpark implements ClimbIO {

    // Harware objects
    private final SparkBase m_climb;

    // Fo turret
    private RelativeEncoder e_climb; // Experimental for using one encoder for two things at the same time
    
    // Closed loop controllers
    private final SparkClosedLoopController c_turret; 

    // Global variables
    private double ClimbExtention = 0;

    public ClimbIOSpark() {

        // Create motor controllers
        m_climb = new SparkMax(climbCanId, MotorType.kBrushless);
        
        e_climb = m_climb.getEncoder();


        /*In 2025 the API changed to remove rollover detection as rollover 
        detection did not work. The get() method returns the value within a
        rotation where the maximum value in a rotation is defined in the constructor\
        (default 1).*/

        // Setup Controllers
        c_turret = m_climb.getClosedLoopController();

        // Turret parameters
        var climbConfig = new SparkMaxConfig();
        climbConfig
            .inverted(climbIsInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(climbMotorCurrent)
            .voltageCompensation(12.0);
        climbConfig
            .encoder
            .positionConversionFactor(climbEncoderPositionFactor)
            .velocityConversionFactor(climbEncoderVelocityFactor)
            .uvwAverageDepth(2);
        climbConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(climbKp, climbKi, climbKd);
        tryUntilOk(
            m_climb,
                   5,
                    () ->
                        m_climb.configure(
                        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }




    // I think this is used for logger pro to keep track of things?
    @Override
    public void updateInputs(ClimbIOInputs inputs) {

    }


       
}
