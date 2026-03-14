package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AngleSlewRateLimiterTest {
    private static final double kDelta = 1e-6;

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
    void testSlewRateLimit() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);

        SimHooks.stepTiming(1.0);
        assertEquals(0.0, limiter.calculate(Rotation2d.fromRadians(0.0)).getRadians(), kDelta);

        SimHooks.stepTiming(1.0);
        assertEquals(1.0, limiter.calculate(Rotation2d.fromRadians(2.0)).getRadians(), kDelta);

        SimHooks.stepTiming(1.0);
        assertEquals(2.0, limiter.calculate(Rotation2d.fromRadians(2.0)).getRadians(), kDelta);
    }

    @Test
    void testReset() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);

        SimHooks.stepTiming(1.0);
        assertEquals(1.0, limiter.calculate(Rotation2d.fromRadians(2.0)).getRadians(), kDelta);

        limiter.reset(Rotation2d.fromRadians(0.5));

        SimHooks.stepTiming(1.0);
        assertEquals(1.5, limiter.calculate(Rotation2d.fromRadians(2.0)).getRadians(), kDelta);
    }

    @Test
    void testTargetReachedInstantlyIfUnderLimit() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);
        limiter.reset(Rotation2d.fromRadians(0.0));

        SimHooks.stepTiming(1.0);
        Rotation2d result = limiter.calculate(Rotation2d.fromRadians(0.5));
        assertEquals(0.5, result.getRadians(), kDelta);
    }

    @Test
    void testShortestPathPositiveWraparound() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);
        limiter.reset(Rotation2d.fromDegrees(175.0));

        SimHooks.stepTiming(0.1);
        Rotation2d result = limiter.calculate(Rotation2d.fromDegrees(-175.0));
        Rotation2d expected = Rotation2d.fromDegrees(175.0).plus(Rotation2d.fromRadians(0.1));

        assertEquals(expected.getRadians(), result.getRadians(), kDelta);
    }

    @Test
    void testShortestPathNegativeWraparound() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);
        limiter.reset(Rotation2d.fromDegrees(-175.0));

        SimHooks.stepTiming(0.1);
        Rotation2d result = limiter.calculate(Rotation2d.fromDegrees(175.0));
        Rotation2d expected = Rotation2d.fromDegrees(-175.0).minus(Rotation2d.fromRadians(0.1));

        assertEquals(expected.getRadians(), result.getRadians(), kDelta);
    }

    @Test
    void testExact180DegreeSingularity() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);
        limiter.reset(Rotation2d.fromDegrees(0.0));

        SimHooks.stepTiming(0.1);
        Rotation2d result1 = limiter.calculate(Rotation2d.fromDegrees(180.0));

        SimHooks.stepTiming(0.1);
        Rotation2d result2 = limiter.calculate(Rotation2d.fromDegrees(180.0));

        assertEquals(0.1, result1.getRadians(), kDelta);
        assertEquals(0.2, result2.getRadians(), kDelta);
    }

    @Test
    void testZeroRateLimit() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(0.0);
        limiter.reset(Rotation2d.fromDegrees(0.0));

        SimHooks.stepTiming(1.0);
        Rotation2d result = limiter.calculate(Rotation2d.fromDegrees(90.0));

        assertEquals(0.0, result.getDegrees(), kDelta);
    }

    @Test
    void testStaleTimerJump() {
        AngleSlewRateLimiter limiter = new AngleSlewRateLimiter(1.0);

        SimHooks.stepTiming(100.0);
        Rotation2d result = limiter.calculate(Rotation2d.fromRadians(2.0));

        assertEquals(2.0, result.getRadians(), kDelta);
    }
}
