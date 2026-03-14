package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import frc.robot.subsystems.turret.TurretConstants;

class DualEncoderUnwrapperTest {
    private static final double RATIO1 =
            TurretConstants.numTeethTurret / TurretConstants.numTeethPulley1;
    private static final double RATIO2 =
            TurretConstants.numTeethTurret / TurretConstants.numTeethPulley2;
    private static final double MIN_MECH_ROT = -TurretConstants.rangeTurret / 2.0;
    private static final double MAX_MECH_ROT = TurretConstants.rangeTurret / 2.0;
    private static final double DEFAULT_MATCH_TOLERANCE = 0.05;
    private static final double STRICT_MATCH_TOLERANCE = 1e-6;
    private static final double EXACT_POSITION_TOLERANCE = 1e-9;
    private static final double SWEEP_POSITION_TOLERANCE = 1e-8;
    private static final double ERROR_TOLERANCE = 1e-9;

    private static DualEncoderUnwrapper newTurretUnwrapper() {
        return new DualEncoderUnwrapper(RATIO1, RATIO2, MIN_MECH_ROT, MAX_MECH_ROT);
    }

    private static DualEncoderUnwrapper newTurretUnwrapper(double matchTolerance) {
        return new DualEncoderUnwrapper(RATIO1, RATIO2, MIN_MECH_ROT, MAX_MECH_ROT, matchTolerance);
    }

    private static double wrap01(double value) {
        return value - Math.floor(value);
    }

    private static double absoluteEncoderReading(double mechanismRotations, double ratio) {
        return wrap01(mechanismRotations * ratio);
    }

    private static Stream<Double> selectedRangeOfMotionPositions() {
        return Stream.of(
                MIN_MECH_ROT,
                MIN_MECH_ROT + 1e-6,
                -0.875,
                -0.75,
                -0.625,
                -0.5,
                -0.375,
                -0.25,
                -0.125,
                0.0,
                0.125,
                0.25,
                0.375,
                0.5,
                0.625,
                0.75,
                0.875,
                MAX_MECH_ROT - 1e-6,
                MAX_MECH_ROT);
    }

    private static Stream<Double> outsideRangeOfMotionPositions() {
        return Stream.of(
                MIN_MECH_ROT - 0.10,
                MIN_MECH_ROT - 0.33,
                MAX_MECH_ROT + 0.10,
                MAX_MECH_ROT + 0.31);
    }

    private static Stream<Arguments> rolloverCases() {
        return Stream.of(
                Arguments.of("encoder1 positive boundary", RATIO1, 9.0),
                Arguments.of("encoder1 negative boundary", RATIO1, -9.0),
                Arguments.of("encoder2 positive boundary", RATIO2, 9.0),
                Arguments.of("encoder2 negative boundary", RATIO2, -9.0));
    }

    @ParameterizedTest
    @MethodSource("selectedRangeOfMotionPositions")
    void unwrapReconstructsSelectedBoundaryAndInteriorPositions(double mechanismRotations) {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper();

        double abs1 = absoluteEncoderReading(mechanismRotations, RATIO1);
        double abs2 = absoluteEncoderReading(mechanismRotations, RATIO2);

        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(abs1, abs2);

        assertEquals(DualEncoderUnwrapper.Status.OK, result.status);
        assertEquals(mechanismRotations, result.position, EXACT_POSITION_TOLERANCE);
        assertTrue(result.error <= ERROR_TOLERANCE);
        assertTrue(result.position >= MIN_MECH_ROT - EXACT_POSITION_TOLERANCE);
        assertTrue(result.position <= MAX_MECH_ROT + EXACT_POSITION_TOLERANCE);
    }

    @Test
    void unwrapReconstructsDenseSweepAcrossExpectedRangeOfMotion() {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper();
        int samples = 2001;

        for (int i = 0; i < samples; i++) {
            double mechanismRotations = MIN_MECH_ROT + (MAX_MECH_ROT - MIN_MECH_ROT) * i / (samples - 1.0);

            DualEncoderUnwrapper.UnwrapResult result =
                    unwrapper.unwrap(
                            absoluteEncoderReading(mechanismRotations, RATIO1),
                            absoluteEncoderReading(mechanismRotations, RATIO2));

            assertEquals(DualEncoderUnwrapper.Status.OK, result.status);
            assertEquals(mechanismRotations, result.position, SWEEP_POSITION_TOLERANCE);
            assertTrue(result.position >= MIN_MECH_ROT - SWEEP_POSITION_TOLERANCE);
            assertTrue(result.position <= MAX_MECH_ROT + SWEEP_POSITION_TOLERANCE);
            assertTrue(result.error <= DEFAULT_MATCH_TOLERANCE);
        }
    }

    @ParameterizedTest
    @MethodSource("outsideRangeOfMotionPositions")
    void unwrapRejectsPositionsOutsideExpectedRangeOfMotion(double outsideMechanismRotations) {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper(STRICT_MATCH_TOLERANCE);

        DualEncoderUnwrapper.UnwrapResult result =
                unwrapper.unwrap(
                        absoluteEncoderReading(outsideMechanismRotations, RATIO1),
                        absoluteEncoderReading(outsideMechanismRotations, RATIO2));

        assertEquals(DualEncoderUnwrapper.Status.NO_SOLUTION, result.status);
        assertTrue(Double.isNaN(result.position));
        assertTrue(result.error > STRICT_MATCH_TOLERANCE);
    }

    @Test
    void unwrapMaintainsMonotonicContinuousPositionAcrossSweep() {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper();
        int samples = 2001;
        double step = (MAX_MECH_ROT - MIN_MECH_ROT) / (samples - 1.0);
        double previous = Double.NaN;

        for (int i = 0; i < samples; i++) {
            double mechanismRotations = MIN_MECH_ROT + step * i;
            DualEncoderUnwrapper.UnwrapResult result =
                    unwrapper.unwrap(
                            absoluteEncoderReading(mechanismRotations, RATIO1),
                            absoluteEncoderReading(mechanismRotations, RATIO2));

            assertEquals(DualEncoderUnwrapper.Status.OK, result.status);
            if (!Double.isNaN(previous)) {
                assertTrue(result.position >= previous - SWEEP_POSITION_TOLERANCE);
                assertEquals(step, result.position - previous, 1e-6);
            }
            previous = result.position;
        }
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("rolloverCases")
    void unwrapMaintainsContinuityAcrossEncoderRollovers(
            String description, double ratioUnderTest, double turnBoundary) {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper();
        double mechanismBeforeRollover = (turnBoundary - 0.01) / ratioUnderTest;
        double mechanismAfterRollover = (turnBoundary + 0.01) / ratioUnderTest;

        DualEncoderUnwrapper.UnwrapResult before =
                unwrapper.unwrap(
                        absoluteEncoderReading(mechanismBeforeRollover, RATIO1),
                        absoluteEncoderReading(mechanismBeforeRollover, RATIO2));
        DualEncoderUnwrapper.UnwrapResult after =
                unwrapper.unwrap(
                        absoluteEncoderReading(mechanismAfterRollover, RATIO1),
                        absoluteEncoderReading(mechanismAfterRollover, RATIO2));

        assertEquals(DualEncoderUnwrapper.Status.OK, before.status);
        assertEquals(DualEncoderUnwrapper.Status.OK, after.status);
        assertEquals(mechanismBeforeRollover, before.position, SWEEP_POSITION_TOLERANCE);
        assertEquals(mechanismAfterRollover, after.position, SWEEP_POSITION_TOLERANCE);
        assertEquals(0.02 / ratioUnderTest, after.position - before.position, 1e-6);
        assertTrue(Math.abs(after.position - before.position) < 0.01);
    }

    @ParameterizedTest
    @CsvSource({
        "3, -2",
        "-10, 8",
        "1234, -987"
    })
    void unwrapNormalizesRawAbsoluteValuesOutsideZeroToOne(double abs1Offset, double abs2Offset) {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper();
        double mechanismRotations = 0.42;
        double abs1Raw = absoluteEncoderReading(mechanismRotations, RATIO1) + abs1Offset;
        double abs2Raw = absoluteEncoderReading(mechanismRotations, RATIO2) + abs2Offset;

        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(abs1Raw, abs2Raw);

        assertEquals(DualEncoderUnwrapper.Status.OK, result.status);
        assertEquals(mechanismRotations, result.position, EXACT_POSITION_TOLERANCE);
    }

    @Test
    void unwrapAcceptsSmallEncoderDisagreementWithinConfiguredTolerance() {
        double matchTolerance = 0.02;
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper(matchTolerance);
        double mechanismRotations = 0.42;
        double abs1 = absoluteEncoderReading(mechanismRotations, RATIO1);
        double abs2 = absoluteEncoderReading(mechanismRotations, RATIO2) + 0.01;

        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(abs1, abs2);

        assertEquals(DualEncoderUnwrapper.Status.OK, result.status);
        assertTrue(result.error <= matchTolerance);
        assertEquals(mechanismRotations, result.position, 5e-3);
    }

    @Test
    void unwrapRejectsInconsistentTurretEncoderPair() {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper(0.02);
        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(0.0, 0.5);
        assertEquals(DualEncoderUnwrapper.Status.NO_SOLUTION, result.status);
        assertTrue(Double.isNaN(result.position));
        assertTrue(result.error > 0.02);
    }

    @Test
    void unwrapAlwaysReturnsInRangePositionWhenStatusIsOk() {
        DualEncoderUnwrapper unwrapper = newTurretUnwrapper();
        Random random = new Random(2026);

        for (int i = 0; i < 5000; i++) {
            double abs1Raw = random.nextDouble() * 40.0 - 20.0;
            double abs2Raw = random.nextDouble() * 40.0 - 20.0;
            DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(abs1Raw, abs2Raw);

            if (result.status == DualEncoderUnwrapper.Status.OK) {
                assertTrue(result.position >= MIN_MECH_ROT - SWEEP_POSITION_TOLERANCE);
                assertTrue(result.position <= MAX_MECH_ROT + SWEEP_POSITION_TOLERANCE);
            }
        }
    }

    @Test
    void unwrapReturnsNoSolutionWhenEncodersDisagreeBeyondTolerance() {
        DualEncoderUnwrapper unwrapper = new DualEncoderUnwrapper(1.0, 1.0, -2.0, 2.0, 0.05);
        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(0.0, 0.5);

        assertEquals(DualEncoderUnwrapper.Status.NO_SOLUTION, result.status);
        assertTrue(Double.isNaN(result.position));
        assertTrue(result.error > 0.05);
    }

    @Test
    void unwrapReturnsAmbiguousWhenMultipleSolutionsFitEquallyWell() {
        DualEncoderUnwrapper unwrapper = new DualEncoderUnwrapper(1.0, 1.0, -2.0, 2.0, 0.05);
        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(0.25, 0.25);

        assertEquals(DualEncoderUnwrapper.Status.AMBIGUOUS, result.status);
        assertTrue(Double.isNaN(result.position));
        assertEquals(0.0, result.error, ERROR_TOLERANCE);
    }
}
