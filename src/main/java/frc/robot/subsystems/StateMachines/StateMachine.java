package frc.robot.subsystems.StateMachines;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.boundBuffer;
import static frc.robot.Constants.bumpBuffer;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.Turret;
import frc.utils.FieldConstants;
import frc.utils.FieldConstants.Hub;
import frc.utils.FieldConstants.LinesHorizontal;
import frc.utils.FieldConstants.LinesVertical;
import frc.utils.FieldConstants.Tower;

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
        BUMP,   // we want to be at a 45 deg angle here
    }

    private boolean hubActive = true;
    private Timer hubTimer = new Timer();

    private boolean isOnLeftSide = false; // left = high x, right = low x
    private FieldZone fieldZone = FieldZone.ALLIANCE;
    private SpecialZone specialZone = SpecialZone.NONE;

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
        updateTime();
        updateZones();
        updateSubsystems();

        Logger.recordOutput("StateMachine/HubActive", hubActive);
        Logger.recordOutput("StateMachine/Zone/Field", fieldZone);
        Logger.recordOutput("StateMachine/Zone/Special", specialZone);
    }

    public void startHubTimer() {
        hubTimer.restart();
    }

    public void stopHubTimer() {
        hubTimer.stop();
    }

    private void updateTime() {
        String gameData = DriverStation.getGameSpecificMessage();
        double time = hubTimer.get();
        Logger.recordOutput("StateMachine/HubTimer", time);
        if (gameData.length() <= 0) {return;}

        boolean evenShift;
        // transition shift or endgame
        if (time < 10.0 || time >= 110.0) {
            hubActive = true;
            return;
        } else if ((time >= 10.0 && time < 35.0) || (time >= 60.0 && time < 85.0)) {
            evenShift = false;
        } else if ((time >= 35.0 && time < 60.0) || (time >= 85.0 && time < 110.0)) {
            evenShift = true;
        } else {
            // in theory this will never happen but java complains about evenShift
            // potentially not being initialized if i don't have this
            return;
        }

        boolean blueActive;
        switch (gameData.charAt(0)) {
            case 'B':
                blueActive = evenShift;
                break;
            case 'R':
                blueActive = !evenShift;
                break;
            default:
                return; // bad things have happened uhh panic i guess
        }

        // if we're red, hubActive = !blueActive. otherwise (if we're blue,
        // or if there's no driverstation), hubActive = blueActive.
        hubActive = blueActive ^ (!DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
    }

    private void updateZones() {
        Translation2d currentTranslation = poseSupplier.get().getTranslation();
        double x = currentTranslation.getX();
        double y = currentTranslation.getY();

        isOnLeftSide = y > LinesHorizontal.center;

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
        } else if (x >= LinesVertical.allianceZone - bumpBuffer &&
                   x <= LinesVertical.neutralZoneNear + bumpBuffer) {
            if (y < LinesHorizontal.rightTrenchOpenStart + inchesToMeters(6.0) || y > LinesHorizontal.leftTrenchOpenEnd - inchesToMeters(6.0)) {
                specialZone = SpecialZone.TRENCH;
            } else {
                specialZone = SpecialZone.BUMP;
            }
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
                        m_turret.setTargetTranslation(new Translation2d(LinesVertical.allianceZone * 3.0 / 4.0, y));
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
                m_turret.setHoodAngle(0.0);
                break;
            case BUMP:
                break; // do nothing ig?
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