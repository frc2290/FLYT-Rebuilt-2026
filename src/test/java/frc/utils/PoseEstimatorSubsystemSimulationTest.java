package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PoseEstimatorSubsystemSimulationTest {
    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    @Test
    void simulationUsesAbsoluteFieldFrameAfterAllianceChange() throws Exception {
        Drive drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
        PoseEstimatorSubsystem subsystem = new PoseEstimatorSubsystem(drive);

        Pose2d blueOdometryPose = new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(0.0));
        drive.setPose(blueOdometryPose);
        subsystem.setAlliance(Alliance.Red);

        Pose2d odometryAfterAlliance = drive.getTrueSimulatedPose();
        Pose2d expectedFlipped = blueOdometryPose.relativeTo(VisionConstants.FLIPPING_POSE);
        assertPoseEquals(expectedFlipped, odometryAfterAlliance, 1e-9);

        subsystem.simulationPeriodic();
        Pose2d lastSimPose = getLastSimPose(subsystem);
        assertPoseEquals(blueOdometryPose, lastSimPose, 1e-9);
    }

    private static Pose2d getLastSimPose(PoseEstimatorSubsystem subsystem) throws Exception {
        Field field = PoseEstimatorSubsystem.class.getDeclaredField("lastSimPose");
        field.setAccessible(true);
        return (Pose2d) field.get(subsystem);
    }

    private static void assertPoseEquals(Pose2d expected, Pose2d actual, double tolerance) {
        assertEquals(expected.getX(), actual.getX(), tolerance);
        assertEquals(expected.getY(), actual.getY(), tolerance);
        assertEquals(
                0.0,
                Math.abs(expected.getRotation().minus(actual.getRotation()).getRadians()),
                tolerance);
    }
}
