// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.energy;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** Class for logging current, power, and energy usage. */
public class BatteryLogger {
    @AutoLog
    public static class BatteryIOInputs {
        public double batteryVoltage = 12.0;
        public double rioCurrent = 0.0;
        public double radioCurrent = 0.0;
        public double cameraCurrent = 0.0;
    }

    public BatteryIOInputsAutoLogged inputs;

    private double totalCurrent = 0.0;
    private double totalPower = 0.0;
    private double totalEnergy = 0.0;

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public BatteryLogger(BatteryIOInputsAutoLogged inputs) {
        this.inputs = inputs;
    }

    /**
     * report current usage of a device with battervoltage
     * @param key name of device, can use slashes like networktables
     * @param amps amps the device is consuming (at battery voltage)
     */
    public void reportCurrentUsage(String key, double amps) {
        reportCurrentUsage(key, amps, inputs.batteryVoltage);
    }

    /**
     * report current usage of a device with specific voltage
     * @param key name of device, can use slashes like networktables
     * @param amps amps the device is consuming (at its applied voltage)
     * @param volts applied voltage
     */
    public void reportCurrentUsage(String key, double amps, double volts) {
        double power = amps * Math.abs(volts);
        double energy = power * 0.02; // loopPeriodSeconds

        totalCurrent += amps;
        totalPower += power;
        totalEnergy += energy;

        subsytemCurrents.put(key, amps);
        subsytemPowers.put(key, power);
        subsytemEnergies.merge(key, energy, Double::sum);

        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsytemCurrents.merge(subkey, amps, Double::sum);
            subsytemPowers.merge(subkey, power, Double::sum);
            subsytemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    /**
     * to be run in the main periodic loop after scheduler, records things to logger
     */
    public void periodicAfterScheduler() {
        Logger.processInputs("BatteryLogger", inputs);

        reportCurrentUsage("Controls/roboRIO", inputs.rioCurrent);
        reportCurrentUsage("Controls/Radio", inputs.radioCurrent);
        reportCurrentUsage("Controls/Cameras", inputs.cameraCurrent);

        // Log total and subsystem energy usage
        Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
        Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
        Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy), "watt hours");

        for (var entry : subsytemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsytemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsytemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemEnergies.entrySet()) {
            Logger.recordOutput(
                    "EnergyLogger/Energy/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()),
                    "watt hours");
        }

        // Reset power and curren totals, before next loop
        totalPower = 0.0;
        totalCurrent = 0.0;
    }

    public double getTotalCurrent() {
        return totalCurrent;
    }

    public double getTotalPower() {
        return totalPower;
    }

    public double getTotalEnergy() {
        return totalEnergy;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}