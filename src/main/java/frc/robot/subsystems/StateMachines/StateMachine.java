package frc.robot.subsystems.StateMachines;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.baseBumpBuffer;
import static frc.robot.Constants.boundBuffer;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.intake.Intake;
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
    private static final double kHubActiveWarningSeconds = 5.0;

    private boolean isOnLeftSide = false; // left = high x, right = low x
    private FieldZone fieldZone = FieldZone.ALLIANCE;
    private SpecialZone specialZone = SpecialZone.NONE;
    private boolean isAuto = false;
    private boolean shootOverride = false;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedSupplier;
    private Intake m_intake;
    private Turret m_turret;
    private DyeRotor m_dyeRotor;

    public StateMachine(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, Intake intake, Turret turret, DyeRotor dyeRotor) {
        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;
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
        Logger.recordOutput("StateMachine/ShootOverride", shootOverride);
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
        Logger.recordOutput("StateMachine/GameData", gameData);
        Logger.recordOutput("StateMachine/HubTimer", time);
        Logger.recordOutput("StateMachine/MatchTime", DriverStation.getMatchTime());
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
        hubActive = blueActive ^ DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    private boolean hubActiveAtTime(double timeSeconds, String gameData, Optional<Alliance> alliance) {
        if (gameData == null || gameData.isEmpty()) {
            return true;
        }

        if (timeSeconds < 10.0 || timeSeconds >= 110.0) {
            return true;
        }

        boolean evenShift;
        if ((timeSeconds >= 10.0 && timeSeconds < 35.0) || (timeSeconds >= 60.0 && timeSeconds < 85.0)) {
            evenShift = false;
        } else if ((timeSeconds >= 35.0 && timeSeconds < 60.0) || (timeSeconds >= 85.0 && timeSeconds < 110.0)) {
            evenShift = true;
        } else {
            return true;
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
                return true;
        }

        boolean isRedAlliance = alliance.equals(Optional.of(Alliance.Red));
        return blueActive ^ isRedAlliance;
    }

    /**
     * Returns how many seconds until the hub becomes active for our alliance. Returns 0 if the hub is
     * already active. Returns +infinity if the game-specific message is missing.
     */
    public double getHubSecondsUntilActive() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) {
            return Double.POSITIVE_INFINITY;
        }

        double now = hubTimer.get();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (hubActiveAtTime(now, gameData, alliance)) {
            return 0.0;
        }

        double[] boundaries = {10.0, 35.0, 60.0, 85.0, 110.0};
        double eps = 1e-3;
        for (double boundary : boundaries) {
            if (boundary <= now) {
                continue;
            }
            if (hubActiveAtTime(boundary + eps, gameData, alliance)) {
                return boundary - now;
            }
        }

        if (now < 110.0) {
            return 110.0 - now;
        }
        return 0.0;
    }

    /** True when the hub is currently inactive, but will become active within 5 seconds. */
    public boolean isHubAboutToBecomeActive() {
        return isHubAboutToBecomeActive(kHubActiveWarningSeconds);
    }

    /** True when the hub is currently inactive, but will become active within {@code warningSeconds}. */
    public boolean isHubAboutToBecomeActive(double warningSeconds) {
        if (warningSeconds <= 0.0) {
            return false;
        }
        if (hubActive) {
            return false;
        }
        double secondsUntilActive = getHubSecondsUntilActive();
        return secondsUntilActive > 0.0 && secondsUntilActive <= warningSeconds;
    }

    private void updateZones() {
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedSupplier.get();
        Translation2d fieldVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond).rotateBy(currentPose.getRotation());
        double x = currentPose.getX();
        double y = currentPose.getY();
        
        double coeff = 0.2 * fieldVelocity.getX();
        double bumpBoundNear = LinesVertical.allianceZone - baseBumpBuffer - Math.max(0, coeff);
        double bumpBoundFar = LinesVertical.neutralZoneNear + baseBumpBuffer - Math.min(0, coeff);
        Logger.recordOutput("StateMachine/BoundNear", new Pose2d(bumpBoundNear, 0, new Rotation2d()));
        Logger.recordOutput("StateMachine/BoundFar", new Pose2d(bumpBoundFar, 0, new Rotation2d()));

        isOnLeftSide = y > LinesHorizontal.center;

        if (x < LinesVertical.allianceZone) {
            fieldZone = FieldZone.ALLIANCE;
        } else if (x <= LinesVertical.oppAllianceZone) {
            fieldZone = FieldZone.NEUTRAL;
        } else {
            fieldZone = FieldZone.ANTI_ALLIANCE;
        }

        specialZone = SpecialZone.NONE;
        if (translationInBound(currentPose.getTranslation(), Tower.rightBackCorner, Tower.leftUpright)) {
            specialZone = SpecialZone.TOWER;
        } else if (x >= bumpBoundNear && x <= bumpBoundFar) {
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
                    case ANTI_ALLIANCE:
                    case ALLIANCE:
                        // point at the hub, but only shoot if hub is active
                        m_turret.setTargetTranslation(Hub.topCenterPoint.toTranslation2d());
                        if (shootOverride) {// && m_turret.flywheelAtSpeed()) {// && m_turret.isTurretPointedAtTarget()) {
                            m_dyeRotor.runDyeRotor(true);
                        } else {
                            m_dyeRotor.runDyeRotor(false);
                        }
                        break;
                    case NEUTRAL:
                        if (shootOverride) {// && m_turret.flywheelAtSpeed()) {// && m_turret.isTurretPointedAtTarget()) {
                            m_dyeRotor.runDyeRotor(true);
                        } else {
                            m_dyeRotor.runDyeRotor(false);
                        }
                        // point at one side of the alliance zone, shoot if magic
                        double y = FieldConstants.fieldWidth;
                        if (isOnLeftSide) {
                            y *= 6.0 / 7.0;
                        } else {
                            y *= 1.0 / 7.0;
                        }
                        m_turret.setTargetTranslation(new Translation2d(LinesVertical.allianceZone * 3.0 / 4.0, y));
                        break;
                }
                break;
            // this case is for both tower & trench
            case TOWER:
            case TRENCH:
            case BUMP:
                // stop shooting (dye rotor & turret) & hood down (turret)
                m_turret.setStopShoot(true);
                m_dyeRotor.runDyeRotor(false);
                break;
        }
    }

    public FieldZone getFieldZone() {
        return fieldZone;
    }

    public SpecialZone getSpecialZone() {
        return specialZone;
    }

    public boolean getLeftSide() {
        return isOnLeftSide;
    }

    public void setfieldZone(FieldZone fieldZone) {
        this.fieldZone = fieldZone;
    }

    public boolean isAuto() {
        return isAuto;
    }

    public void setIsAuto(boolean isAuto) {
        this.isAuto = isAuto;
    }

    public boolean isShootOverride() {
        return shootOverride;
    }

    public void setShootOverride(boolean shootOverride) {
        this.shootOverride = shootOverride;
    }

    public Command setShooterOverrideCommand(boolean shootOverride) {
        return runOnce(() -> setShootOverride(shootOverride));
    }

    public boolean isHubActive() {
        return hubActive;
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
