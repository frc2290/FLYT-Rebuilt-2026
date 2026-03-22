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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LEDEffects.LEDEffect;
import java.util.ArrayList;

/** Manages LED strips and applies predefined animation effects across the robot. */
public class LEDUtility extends SubsystemBase {
  /** Physical PWM LED controller driving all strips. */
  AddressableLED addressableLED;

  /** Total number of LEDs required for the highest indexed strip. */
  int overallLength = 0;

  /** Shared buffer that backs the addressable LED output. */
  AddressableLEDBuffer filler = new AddressableLEDBuffer(0);

  /** List of all named strips currently managed by the utility. */
  ArrayList<LEDStrip> newLedStrips = new ArrayList<>();

  private boolean started = false;

  /** Creates a new LEDUtility. */
  public LEDUtility(int _port) {
    addressableLED = new AddressableLED(_port);
  }

  public void addStrip(String _name, int start, int end) {
    overallLength = Math.max(overallLength, end + 1);
    setLength(overallLength);
    newLedStrips.add(new LEDStrip(_name, filler, start, end));
  }

  public LEDStrip getStrip(int index) {
    return newLedStrips.get(index);
  }

  public LEDStrip getStrip(String name) {
    for (LEDStrip strip : newLedStrips) {
      if (strip.getName().equals(name)) {
        return strip;
      }
    }
    return null;
  }

  public Command setStrip(String _name, LEDEffect _effect) {
    return Commands.runOnce(() -> getStrip(_name).setEffect(_effect), this);
  }

  public Command setStrip(String _name, LEDEffect _effect, Color _color) {
    return Commands.runOnce(() -> getStrip(_name).setEffect(_effect, _color), this);
  }

  public void setAll(LEDEffect _effect, Color _color) {
    newLedStrips.forEach(
        strip -> {
          strip.setColor(_color);
          strip.setEffect(_effect);
        });
  }

  public Command setAllCommand(LEDEffect _effect, Color _color) {
    return Commands.runOnce(() -> setAll(_effect, _color), this);
  }

  public void setAll(LEDEffect _effect) {
    newLedStrips.forEach(
        strip -> {
          strip.setEffect(_effect);
        });
  }

  public Command setAllCommand(LEDEffect _effect) {
    return Commands.runOnce(() -> setAll(_effect), this);
  }

  public void setAll(LEDPattern pattern) {
    newLedStrips.forEach(strip -> strip.setPattern(pattern));
  }

  public Command setAllCommand(LEDPattern pattern) {
    return Commands.runOnce(() -> setAll(pattern), this);
  }

  // DEFAULT LED PATTERN, CHANGE PER SEASON
  public void setDefault() {
    getStrip("Left").setEffect(LEDEffect.PULSE);
    getStrip("Left").setColor(LEDEffects.flytBlue);
    getStrip("Right").setEffect(LEDEffect.PULSE);
    getStrip("Right").setColor(LEDEffects.flytBlue);
    getStrip("Front").setEffect(LEDEffect.PULSE);
    getStrip("Front").setColor(LEDEffects.flytBlue);
    getStrip("Back").setEffect(LEDEffect.PULSE);
    getStrip("Back").setColor(LEDEffects.flytBlue);
  }

  private void setLength(int length) {
    filler = new AddressableLEDBuffer(length);
    addressableLED.setLength(length);
    for (LEDStrip strip : newLedStrips) {
      strip.setBufferView(filler);
    }
    if (!started && length > 0) {
      addressableLED.start();
      started = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try {
      newLedStrips.forEach(strip -> strip.getPattern().applyTo(strip.getBufferView()));
      addressableLED.setData(filler);
      addressableLED.start();
    } catch (Exception e) {
      System.out.println("LED EXception: " + e);
    }
  }
}
