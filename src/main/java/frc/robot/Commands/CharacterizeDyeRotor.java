package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.dyerotor.DyeRotor;

/** Runs the full 8-step DyeRotor SysId sequence with settle delays between steps. */
public class CharacterizeDyeRotor extends SequentialCommandGroup {
    private static final double SETTLE_TIME_SECONDS = 1;

    public CharacterizeDyeRotor(DyeRotor dyeRotor) {
        addRequirements(dyeRotor);

        addCommands(
                // --- COMMON MODE EXCITATION ---
                dyeRotor.sysIdQuasistaticDyeRotorCommon(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),
                dyeRotor.sysIdQuasistaticDyeRotorCommon(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),
                dyeRotor.sysIdDynamicDyeRotorCommon(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),
                dyeRotor.sysIdDynamicDyeRotorCommon(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),

                // --- COUNTER MODE EXCITATION ---
                dyeRotor.sysIdQuasistaticDyeRotorCounter(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),
                dyeRotor.sysIdQuasistaticDyeRotorCounter(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),
                dyeRotor.sysIdDynamicDyeRotorCounter(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(SETTLE_TIME_SECONDS),
                dyeRotor.sysIdDynamicDyeRotorCounter(SysIdRoutine.Direction.kReverse));
    }
}
