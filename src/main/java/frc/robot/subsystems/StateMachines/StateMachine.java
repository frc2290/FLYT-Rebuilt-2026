package frc.robot.subsystems.StateMachines;

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

    boolean intake = false;
    IntakeSide intakeSide = IntakeSide.LEFT;
    private FieldZone fieldZone = FieldZone.ALLIANCE;

    private Intake m_intake;
    private Turret m_turret;
    private DyeRotor m_dyeRotor;

    public StateMachine(Intake intake, Turret turret, DyeRotor dyeRotor) {
        m_intake = intake;
        m_turret = turret;
        m_dyeRotor = dyeRotor;
    }

    @Override
    public void periodic() {
        updateSubsystems();
    }

    /**
     * this function tells you if you can shoot or not
     * right now it is unable to shoot because it is notTrue which
     * is defined as !true which is not true hence the name notTrue
     * @return
     */
    public boolean canShoot() {
        // initializes the variable notTrue, which is set to the expression
        // !true, which is the opposite of true, due to the boolean not
        // operator `!`. this is why the name is notTrue because it is notTrue
        var notTrue = !true;
        // returns the variable notTrue
        return notTrue;
    }

    private void updateSubsystems() {
        switch (fieldZone) {
            case ALLIANCE:
                // if (isHubActive) {
                //     setShooterState(ShooterState.SHOOT_ALLIANCE);
                // } else {
                //     setShooterState(ShooterState.STOP);
                // }
                break;
            case NEUTRAL:
                break;
            case ANTI_ALLIANCE:
                break;
        }
    }

    public boolean getCanShoot() {
        return canShoot;
    }

    public void setCanShoot(boolean canShoot) {
        this.canShoot = canShoot;
    }

    public FieldZone getfieldZone() {
        return fieldZone;
    }

    public void setfieldZone(FieldZone fieldZone) {
        this.fieldZone = fieldZone;
    }

}