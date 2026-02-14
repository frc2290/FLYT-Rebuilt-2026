package frc.robot.subsystems.StateMachines;

import static frc.robot.Constants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.Turret;
import frc.utils.FieldConstants;
import frc.utils.FieldConstants.*;

public class StateMachine extends SubsystemBase {
    public enum FieldZone {
        ALLIANCE,      // alliance zone
        NEUTRAL,       // neutral zone
        ANTI_ALLIANCE, // opposite alliance zone
    }

    public enum SpecialZone {
        NONE,   // no special zone
        TOWER,  // we don't really want to shoot here
        TRENCH, // we want to put the hood down under here
    }

    private boolean isHubActive = false;
    private boolean isOnLeftSide = false; // left = high x, right = low x
    private FieldZone fieldZone = FieldZone.ALLIANCE;
    private SpecialZone specialZone = SpecialZone.NONE;

    private boolean isIntaking = false;
    private IntakeSide intakeSide = IntakeSide.LEFT;

    private Supplier<Pose2d> poseSupplier;
    private Intake m_intake;
    private Turret m_turret;
    private DyeRotor m_dyeRotor;

    public StateMachine(Supplier<Pose2d> poseSupplier, Intake intake, Turret turret, DyeRotor dyeRotor) {
        this.poseSupplier = poseSupplier;
        m_intake = intake;
        m_turret = turret;
        m_dyeRotor = dyeRotor;
    }

    @Override
    public void periodic() {
        updateZones();
        updateSubsystems();

        Logger.recordOutput("StateMachine/Zone/Field", fieldZone);
        Logger.recordOutput("StateMachine/Zone/Special", specialZone);
    }

    private void updateZones() {
        Translation2d currentTranslation = poseSupplier.get().getTranslation();
        double x = currentTranslation.getX();
        double y = currentTranslation.getY();

        isOnLeftSide = y > FieldConstants.fieldWidth / 2;

        if (x < LinesVertical.allianceZone) {
            fieldZone = FieldZone.ALLIANCE;
        } else if (x <= LinesVertical.oppAllianceZone) {
            fieldZone = FieldZone.NEUTRAL;
        } else {
            fieldZone = FieldZone.ANTI_ALLIANCE;
        }

        specialZone = SpecialZone.NONE;
        if (translationInBound(currentTranslation, Tower.rightBackCorner, Tower.leftUpright)) {
            specialZone = SpecialZone.TOWER;
        } else if (x >= LinesVertical.allianceZone - boundBuffer &&
                   x <= LinesVertical.neutralZoneNear + boundBuffer) {
            specialZone = SpecialZone.TRENCH;
        }
    }

    private void updateSubsystems() {
        switch (specialZone) {
            case NONE:
                m_turret.setStopShoot(false);
                switch (fieldZone) {
                    case ALLIANCE:
                        // point at the hub, but only shoot if hub is active
                        m_turret.setTargetTranslation(Hub.topCenterPoint.toTranslation2d());
                        break;
                    case NEUTRAL:
                        // point at one side of the alliance zone, shoot if magic
                        double y = FieldConstants.fieldWidth;
                        if (isOnLeftSide) {
                            y *= 6.0 / 7.0;
                        } else {
                            y *= 1.0 / 7.0;
                        }
                        m_turret.setTargetTranslation(new Translation2d(LinesVertical.allianceZone * 3 / 4, y));
                        break;
                    case ANTI_ALLIANCE:
                        // point at one side of neutral zone, shoot if uhh more magic
                        // or just do nothing
                        break;
                }
                break;
            // this case is for both tower & trench
            case TOWER:
            case TRENCH:
                // stop shooting (dye rotor & turret) & hood down (turret)
                m_turret.setStopShoot(true);
                m_turret.setHoodAngle(0);
                break;
        }
    }

    public FieldZone getFieldZone() {
        return fieldZone;
    }

    public SpecialZone getSpecialZone() {
        return specialZone;
    }

    public void setfieldZone(FieldZone fieldZone) {
        this.fieldZone = fieldZone;
    }

    /**
     * Test if a Translation2d is within a specified boundary, with a buffer (from constants).
     * @param test  The Pose2d to test
     * @param lower The lower bound
     * @param upper The upper bound
     * @return      True if the pose is within the specified bounds
     */
    private boolean translationInBound(Translation2d test, Translation2d lower, Translation2d upper) {
        double testX = test.getX();
        double testY = test.getY();
        return (testX >= lower.getX() - boundBuffer &&
                testY >= lower.getY() - boundBuffer &&
                testX <= upper.getX() + boundBuffer &&
                testY <= upper.getY() + boundBuffer);
    }
}