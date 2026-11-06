package frc.utils;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.lumynlabs.devices.ConnectorXAnimate;

import static edu.wpi.first.units.Units.Milliseconds;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;
import com.lumynlabs.domain.config.NetworkType;
import com.lumynlabs.domain.led.Animation;

public class LEDControllerCx extends SubsystemBase {
    public static final Color flytBlue = new Color("#0081B3");
    private ConnectorXAnimate cXAnimate = new ConnectorXAnimate();

    // TODO: add jolly mode
    public LEDControllerCx() {
        ConfigBuilder builder = new ConfigBuilder();
        LumynDeviceConfig cfg = builder
            .forTeam("2290")
            .setNetworkType(NetworkType.USB)
            .addChannel(1, "main-channel", 66 + 71 + 30 + 71)
                // .addStripZone("front", 66)
                // .addStripZone("right", 71)
                // .addStripZone("back", 30)
                // .addStripZone("left", 71)
                .addStripZone("all", 66 + 71 + 30 + 71) // no need for seperate strips w/ 2026 robot
                .endChannel()
            .build();
        // applying the configuration before connection should be fine,
        // theoretically it will be sent first thing once connected
        cXAnimate.ApplyConfiguration(cfg);
        boolean connected = cXAnimate.Connect(USBPort.kUSB1);
        System.out.println("ConnectorX connected: " + connected);
        setDefault();
    }

    public void setDefault() {
        // TODO: make chase effect better somehow
        setAll(Animation.Chase, flytBlue);
    }

    public void setAll(Animation animation, Color color) {
        cXAnimate.leds.SetAnimation(animation)
            .ForZone("all")
            .WithColor(color)
            .WithDelay(Milliseconds.of(0))
            .RunOnce(false);
    }

    public Command setAllCommand(Animation animation, Color color) {
        return runOnce(() -> {
            setAll(animation, color);
        });
    }
}
