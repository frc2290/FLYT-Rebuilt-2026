package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Slew-rate limiter for continuous angular values using {@link Rotation2d}.
 *
 * <p>The limiter automatically takes the shortest path around wraparound boundaries.
 */
public class AngleSlewRateLimiter {
  private final double maxRateRadiansPerSec;
  private Rotation2d prevVal;
  private double prevTime;

  /**
   * @param maxRateRadiansPerSec Maximum allowed change in radians per second.
   */
  public AngleSlewRateLimiter(double maxRateRadiansPerSec) {
    this.maxRateRadiansPerSec = Math.abs(maxRateRadiansPerSec);
    prevVal = new Rotation2d();
    prevTime = Timer.getFPGATimestamp();
  }

  /**
   * Filters the input angle.
   *
   * @param input Target angle.
   * @return Rate-limited angle.
   */
  public Rotation2d calculate(Rotation2d input) {
    double currentTime = Timer.getFPGATimestamp();
    double elapsedTime = Math.max(0.0, currentTime - prevTime);
    prevTime = currentTime;

    double angleErrorRad = input.minus(prevVal).getRadians();
    double maxChangeRad = maxRateRadiansPerSec * elapsedTime;
    double clampedChangeRad = MathUtil.clamp(angleErrorRad, -maxChangeRad, maxChangeRad);

    prevVal = prevVal.plus(Rotation2d.fromRadians(clampedChangeRad));
    return prevVal;
  }

  /** Resets the limiter state to the provided angle. */
  public void reset(Rotation2d value) {
    prevVal = value;
    prevTime = Timer.getFPGATimestamp();
  }
}
