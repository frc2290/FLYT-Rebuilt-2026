package frc.utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
import com.lumynlabs.domain.led.DirectLED;

public class LEDControllerCx extends SubsystemBase {
    public static final Color flytBlue = new Color("#0081B3");
    private static final int numLeds = 66 + 71 + 30 + 71;
    
    private ConnectorXAnimate cXAnimate = new ConnectorXAnimate();
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(numLeds);
    private DirectLED direct;
    private int animTime;

    public LEDControllerCx() {
        ConfigBuilder builder = new ConfigBuilder();
        LumynDeviceConfig cfg = builder
            .forTeam("2290")
            .setNetworkType(NetworkType.USB)
            .addChannel(1, "main-channel", numLeds)
                // .addStripZone("front", 66)
                // .addStripZone("right", 71)
                // .addStripZone("back", 30)
                // .addStripZone("left", 71)
                .addStripZone("all", numLeds) // no need for seperate strips w/ 2026 robot
                .endChannel()
            .build();
        // applying the configuration before connection should be fine,
        // theoretically it will be sent first thing once connected
        cXAnimate.ApplyConfiguration(cfg);
        boolean connected = cXAnimate.Connect(USBPort.kUSB1);
        System.out.println("ConnectorX connected: " + connected);
        direct = cXAnimate.leds.createDirectLED("all", numLeds);
        setDefault();
    }

    public void setDefault() {
        cXAnimate.leds.SetAnimation(Animation.Chase)
            .ForZone("all")
            .WithColor(flytBlue)
            .WithDelay(Milliseconds.of(15))
            .execute();
    }

    public Command setDefaultCommand() {
        return runOnce(this::setDefault);
    }

    public Command jollyModeCommand() {
        return run(() -> {
            for (int i = 0; i < numLeds; i++) {
                buffer.setLED(i, ((i - animTime / 10) / 3 % 2) == 0 ? Color.kLime : Color.kRed);
            }
            direct.update(buffer);
            animTime++;
        }).finallyDo(interrupted -> setDefault());
    }

    public Command setSolidCommand(Color color) {
        return runOnce(() -> {
            cXAnimate.leds.SetAnimation(Animation.Fill)
                .ForZone("all")
                .WithColor(color)
                .WithDelay(Milliseconds.of(0))
                .execute();
        });
    }

    public Command setStrobeCommand(Color color) {
        return runOnce(() -> {
            cXAnimate.leds.SetAnimation(Animation.Blink)
                .ForZone("all")
                .WithColor(color)
                .WithDelay(Milliseconds.of(200))
                .execute();
        });
    }
}
