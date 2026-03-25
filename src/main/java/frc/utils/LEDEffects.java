// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.utils;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

/** Central location for predefined LED animations used throughout the robot. */
public class LEDEffects {

  public static enum LEDEffect {
    SOLID,
    RAINBOW,
    RSL,
    FLASH,
    PULSE,
    CHASING,
    ALLIANCE,
    NAVLIGHTS
  }

  // Team Blue
  public static Color flytBlue = new Color("#0081B3");

  /** Returns the pattern that should be applied for a strip + legacy effect enum. */
  public static LEDPattern patternFor(LEDEffect effect, LEDStrip strip) {
    switch (effect) {
      case SOLID:
        return solid(strip.getColor());
      case RAINBOW:
        return rainbow();
      case RSL:
        return rslSynchronizedBlink(flytBlue);
      case FLASH:
        return flashing(strip.getColor(), 0.05);
      case PULSE:
        return pulsing(strip.getColor(), 2.0);
      case CHASING:
        return chasing(strip.getColor(), 100.0);
      case ALLIANCE:
        return allianceBreathe(2.0);
      case NAVLIGHTS:
        return navLights(strip.getColor(), 1.0, strip.getHelperBool());
      default:
        return solid(strip.getColor());
    }
  }

  public static LEDPattern solid(Color color) {
    return LEDPattern.solid(color);
  }

  public static LEDPattern solidHSV(int h, int s, int v) {
    return LEDPattern.solid(Color.fromHSV(h, s, v));
  }

  public static LEDPattern rainbow() {
    return LEDPattern.rainbow(255, 128);
  }

  public static LEDPattern rslSynchronizedBlink(Color color) {
    return LEDPattern.solid(color).synchronizedBlink(RobotController::getRSLState);
  }

  public static LEDPattern flashing(Color color, double intervalSeconds) {
    return LEDPattern.solid(color).blink(Seconds.of(intervalSeconds));
  }

  public static LEDPattern pulsing(Color color, double intervalSeconds) {
    return LEDPattern.solid(color).breathe(Seconds.of(intervalSeconds));
  }

  public static LEDPattern chasing(Color color, double intervalSeconds) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.2, Color.kBlack);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(Percent.per(Second).of(intervalSeconds));
    return LEDPattern.solid(color).mask(mask);
  }

  public static LEDPattern navLights(Color color, double intervalSeconds, boolean on) {
    Map<Double, Color> maskSteps =
        Map.of(0.0, (on ? Color.kBlack : Color.kWhite), 0.5, (on ? Color.kWhite : Color.kBlack));
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(Percent.per(Second).of(intervalSeconds * 2.0));
    return LEDPattern.solid(color).mask(mask);
  }

  public static LEDPattern allianceBreathe(double intervalSeconds) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return LEDPattern.solid((alliance.get() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed))
          .breathe(Seconds.of(intervalSeconds));
    }
    return LEDPattern.solid(Color.kWhite);
  }

  // Backwards-compatible helpers (apply immediately).
  public static void setSolidColor(LEDStrip strip) {
    solid(strip.getColor()).applyTo(strip.getBufferView());
  }

  public static void setSolidColor(LEDStrip strip, Color color) {
    solid(color).applyTo(strip.getBufferView());
  }

  public static void setHSVColor(LEDStrip strip, int h, int s, int v) {
    solidHSV(h, s, v).applyTo(strip.getBufferView());
  }

  public static void setRainbow(LEDStrip strip) {
    rainbow().applyTo(strip.getBufferView());
  }

  public static void setRSLFlashing(LEDStrip strip) {
    rslSynchronizedBlink(flytBlue).applyTo(strip.getBufferView());
  }

  public static void setFlashing(LEDStrip strip, double intervalSeconds) {
    flashing(strip.getColor(), intervalSeconds).applyTo(strip.getBufferView());
  }

  public static void setNavLights(LEDStrip strip, double intervalSeconds, boolean on) {
    navLights(strip.getColor(), intervalSeconds, on).applyTo(strip.getBufferView());
  }

  public static void setPulsing(LEDStrip strip, int intervalSeconds) {
    pulsing(strip.getColor(), intervalSeconds).applyTo(strip.getBufferView());
  }

  public static void setChasing(LEDStrip strip, double intervalSeconds) {
    chasing(strip.getColor(), intervalSeconds).applyTo(strip.getBufferView());
  }

  public static void setAllianceColor(LEDStrip strip) {
    allianceBreathe(2.0).applyTo(strip.getBufferView());
  }
}
