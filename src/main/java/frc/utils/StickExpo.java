package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Applies an exponential curve to joystick inputs to smooth controls.
 * Designed to process inputs that have already had a deadband applied.
 *
 * <p>Boost ratio is "edge sensitivity divided by center sensitivity". For example, a boost ratio
 * of 4.0 means small motions near center are softened, but full-stick motions ramp up so the
 * response near the edge is 4x as sensitive as center.
 *
 * <p>Curve form (for magnitude x in [0, 1]): y = c*x + (1-c)*x^e
 *
 * <p>This keeps y(0)=0 and y(1)=1, while allowing sensitivity shaping through dy/dx:
 * center-slope = c, edge-slope = c + (1-c)*e. The constructor chooses c/e from boost ratio so:
 * edge-slope / center-slope = boostRatio.
 */
public class StickExpo {
  private static final double EPS = 1e-9;

  // Blending factor for linear term. Also equals sensitivity at stick center.
  private final double c;
  // Exponent for nonlinear term. Higher values push more shaping toward outer stick travel.
  private final double e;
  // Fast-path when boost ratio is effectively linear.
  private final boolean isLinear;

  /**
   * Creates a StickExpo curve using a Boost Ratio.
   *
   * <p>Interpretation:
   *
   * <p>- 1.0: linear response (same sensitivity everywhere)
   *
   * <p>- greater than 1.0: softer center for precision, more aggressive response near full stick
   *
   * <p>Parameterization:
   *
   * <p>Let R = boostRatio, k = sqrt(R). Then c = 1/k and e = k + 1.
   *
   * <p>With y = c*x + (1-c)*x^e, this gives:
   *
   * <p>- center slope y'(0) = c = 1/k
   *
   * <p>- edge slope y'(1) = k
   *
   * <p>- slope ratio y'(1)/y'(0) = k^2 = R
   *
   * @param boostRatio Ratio of sensitivity at full stick versus center stick. Values less than or
   *     equal to 1.0 are treated as linear.
   */
  public StickExpo(double boostRatio) {
    if (boostRatio <= 1.0 + 1e-6) {
      this.c = 1.0;
      this.e = 1.0;
      this.isLinear = true;
    } else {
      // Convert desired slope ratio into curve parameters.
      double k = Math.sqrt(boostRatio);
      this.c = 1.0 / k;
      this.e = k + 1.0;
      this.isLinear = false;
    }
  }

  /**
   * Shapes a 2D joystick input (for example left stick X and Y for translation).
   *
   * @param x The pre-deadbanded X input.
   * @param y The pre-deadbanded Y input.
   * @return A Translation2d containing the shaped and scaled X and Y values.
   */
  public Translation2d shape2D(double x, double y) {
    double mag = Math.hypot(x, y);

    // Return zero vector if magnitude is tiny to avoid divide-by-zero.
    if (mag < EPS) {
      return new Translation2d(0.0, 0.0);
    }

    // Clamp square corners to a perfect circle.
    if (mag > 1.0) {
      x /= mag;
      y /= mag;
      mag = 1.0;
    }

    // Shape only the magnitude, then re-apply original direction.
    // This preserves stick direction while changing response feel.
    double magShaped = isLinear ? mag : (c * mag + (1.0 - c) * Math.pow(mag, e));

    // Recompose vector.
    return new Translation2d((x / mag) * magShaped, (y / mag) * magShaped);
  }

  /**
   * Shapes a 1D joystick input (for example right stick X for rotation).
   *
   * @param val The pre-deadbanded 1D input.
   * @return The shaped input, retaining the original sign.
   */
  public double shape1D(double val) {
    double absVal = Math.abs(val);
    if (absVal < EPS) {
      return 0.0;
    }
    // Same curve as 2D magnitude shaping, applied to a scalar axis.
    double shaped = isLinear ? absVal : (c * absVal + (1.0 - c) * Math.pow(absVal, e));
    return Math.copySign(shaped, val);
  }
}
