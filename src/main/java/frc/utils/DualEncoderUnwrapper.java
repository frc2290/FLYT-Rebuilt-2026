// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.MathUtil;

/**
 * Resolves the continuous mechanism position from two absolute encoders.
 * Uses an analytic targeting approach (easycrt_solve) merged with the
 * safety structures of EasyCRT.
 */
public class DualEncoderUnwrapper {

    public enum Status {
        OK,
        NO_SOLUTION,
        AMBIGUOUS
    }

    public static class UnwrapResult {
        public final Status status;
        public final double position;
        public final double error;

        public UnwrapResult(Status status, double position, double error) {
            this.status = status;
            this.position = position;
            this.error = error;
        }
    }

    // Temporary data structure to hold the best metrics during the search
    private static class SearchState {
        double bestErr = Double.MAX_VALUE;
        double secondErr = Double.MAX_VALUE;
        double bestMech = Double.NaN;
    }

    private final double ratio1;
    private final double ratio2;
    private final double minMechRot;
    private final double maxMechRot;
    private final double matchTolerance;

    // Pre-calculate the denominator for the Least Squares math to save CPU cycles
    private final double denom;

    public DualEncoderUnwrapper(
            double ratio1, double ratio2, double minMechRot, double maxMechRot, double matchTolerance) {
        this.ratio1 = ratio1;
        this.ratio2 = ratio2;
        this.minMechRot = minMechRot;
        this.maxMechRot = maxMechRot;
        this.matchTolerance = matchTolerance;
        this.denom = (ratio1 * ratio1) + (ratio2 * ratio2);
    }

    public DualEncoderUnwrapper(double ratio1, double ratio2, double minMechRot, double maxMechRot) {
        this(ratio1, ratio2, minMechRot, maxMechRot, 0.05);
    }

    public UnwrapResult unwrap(double abs1Raw, double abs2Raw) {
        // Wrap inputs to [0, 1) phase
        double abs1 = MathUtil.inputModulus(abs1Raw, 0.0, 1.0);
        double abs2 = MathUtil.inputModulus(abs2Raw, 0.0, 1.0);

        // Find feasible wrap bounds for Encoder 1 (turns1)
        double encoder1MinContinuousRot = ratio1 * minMechRot;
        double encoder1MaxContinuousRot = ratio1 * maxMechRot;

        int turns1Start = (int) Math.floor(Math.min(encoder1MinContinuousRot, encoder1MaxContinuousRot) - abs1) - 1;
        int turns1End = (int) Math.ceil(Math.max(encoder1MinContinuousRot, encoder1MaxContinuousRot) - abs1) + 1;

        SearchState state = new SearchState();

        /*
         * THE UNWRAP LOGIC
         * Instead of a slow 2D loop guessing both combinations of full turns, we only
         * iterate over Encoder 1's possible full turns (turns1). Because the gears
         * are physically meshed, a guess for Encoder 1 mathematically dictates
         * where Encoder 2 should be for that same mechanism angle.
         */
        for (int turns1 = turns1Start; turns1 <= turns1End; turns1++) {
            // Encoder 1's continuous position (fractional turn + guessed full turns).
            double encoder1ContinuousRot = abs1 + turns1;

            // Analytic optimum for Encoder 2's turns (before rounding)
            // If Encoder 1 is at this continuous position, use the gear ratios to predict how many
            // turns Encoder 2 should have completed. This usually lands on a decimal.
            double turns2Exact = (ratio2 * encoder1ContinuousRot) / ratio1 - abs2;
            long turns2Floor = (long) Math.floor(turns2Exact);

            // Encoder 2 turns must be an integer count of full turns, so test only the two
            // nearest integers around the analytic prediction.
            evaluateCandidate(encoder1ContinuousRot, abs2 + turns2Floor, state);
            evaluateCandidate(encoder1ContinuousRot, abs2 + turns2Floor + 1, state);
        }

        if (Double.isNaN(state.bestMech) || state.bestErr > matchTolerance) {
            return new UnwrapResult(Status.NO_SOLUTION, Double.NaN, state.bestErr);
        }

        // EasyCRT style ambiguity check: two almost-equal best fits means we
        // cannot confidently choose a unique mechanism angle.
        // This can happen with noise, loose tolerance, or mechanical issues.
        if (state.secondErr <= matchTolerance && Math.abs(state.secondErr - state.bestErr) < 1e-3) {
            return new UnwrapResult(Status.AMBIGUOUS, Double.NaN, state.bestErr);
        }

        return new UnwrapResult(Status.OK, state.bestMech, state.bestErr);
    }

    /**
     * Evaluates one unwrapped encoder pair and updates SearchState if it is a new best.
     */
    private void evaluateCandidate(
            double encoder1ContinuousRot, double encoder2ContinuousRot, SearchState state) {
        /*
         * Least-squares mechanism position:
         * We have two sensors that may disagree slightly due to backlash or noise.
         * This computes the mechanism angle that minimizes the combined squared
         * error across both encoder equations.
         */
        double mechanismRot = (ratio1 * encoder1ContinuousRot + ratio2 * encoder2ContinuousRot) / denom;

        // Discard if physically impossible
        if (mechanismRot < minMechRot - 1e-6 || mechanismRot > maxMechRot + 1e-6) {
            return;
        }

        /*
         * Residual (error score):
         * Compare where each encoder should be based on this mechanism estimate
         * versus the candidate unwrapped values.
         * Lower error means a better physical fit.
         */
        double residual1 = ratio1 * mechanismRot - encoder1ContinuousRot;
        double residual2 = ratio2 * mechanismRot - encoder2ContinuousRot;
        double rmsError = Math.sqrt((residual1 * residual1 + residual2 * residual2) / 2.0);

        // Keep track of the best and second-best candidates for ambiguity checks.
        if (rmsError < state.bestErr) {
            state.secondErr = state.bestErr;
            state.bestErr = rmsError;
            state.bestMech = mechanismRot;
        } else if (rmsError < state.secondErr && Math.abs(mechanismRot - state.bestMech) > 1e-4) {
            state.secondErr = rmsError;
        }
    }
}
