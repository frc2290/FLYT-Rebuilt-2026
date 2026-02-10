package frc.robot.subsystems.StateMachines;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.DyeRotor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.Turret;

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

    boolean isHubActive = false;
    boolean intake = false;
    IntakeSide intakeSide = IntakeSide.LEFT;
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
        updateZones();
        updateSubsystems();
    }

    private void updateZones() {
        Pose2d currentPose = poseSupplier.get();
    }

    private void updateSubsystems() {
        switch (specialZone) {
            case NONE:
                switch (fieldZone) {
                    case ALLIANCE:
                        // point at the hub, but only shoot if hub is active
                        break;
                    case NEUTRAL:
                        // point at one side of the alliance zone, shoot if magic
                        break;
                    case ANTI_ALLIANCE:
                        // point at one side of neutral zone, shoot if uhh more magic
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
}