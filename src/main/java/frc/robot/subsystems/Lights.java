package frc.robot.subsystems;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.function.Supplier;

import javax.imageio.ImageIO;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;

public class Lights extends SubsystemBase {
    AddressableLED driver;
    AddressableLEDBuffer buffer;

    public enum LightState {
        CONE(staticImageSupplier("cone.png")),
        CUBE(staticImageSupplier("cube.png")),
        LOGO(staticImageSupplier("logo.png")),
        BLANK(new Supplier<BufferedImage>() {
            public BufferedImage get() {
                return new BufferedImage(MiscConstants.STRIP_W, MiscConstants.STRIP_H,
                        BufferedImage.TYPE_INT_ARGB);
            };
        }),
        RAINBOW(new Supplier<BufferedImage>() {
            public BufferedImage get() {
                return rainbow();
            };
        });

        private Supplier<BufferedImage> supplier;

        private LightState(Supplier<BufferedImage> supplier) {
            this.supplier = supplier;
        }
    }

    public static LightState state = LightState.LOGO;

    public Lights() {
        driver = new AddressableLED(MiscConstants.LED_PORT);
        driver.setLength(MiscConstants.NUM_LEDS);
        buffer = new AddressableLEDBuffer(MiscConstants.NUM_LEDS);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        clear();
        setImage(state.supplier.get());
        driver.setData(buffer);
        driver.start();
    }

    public static BufferedImage rainbow() {
        BufferedImage img = new BufferedImage(MiscConstants.STRIP_W, MiscConstants.STRIP_H,
                BufferedImage.TYPE_INT_ARGB);
        for (int i = 0; i < MiscConstants.NUM_LEDS; ++i) {
            Color c = Color.getHSBColor(
                    (((float) i / MiscConstants.NUM_LEDS) + ((float) Timer.getFPGATimestamp() * 0.25f)) % 1.0f,
                    1.0f,
                    0.5f);
            img.setRGB(i / MiscConstants.STRIP_H, i % MiscConstants.STRIP_H, c.getRGB());
        }

        return img;
    }

    private int getPixelIndex(int x, int y) {
        x = MiscConstants.STRIP_W - 1 - x;
        y = MiscConstants.STRIP_H - 1 - y;

        return x * MiscConstants.STRIP_H + (((x & 1) == 1) ? MiscConstants.STRIP_H - 1 - y : y);
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); ++i) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }

    private void setPixel(int x, int y, Color color) {
        if (x >= 0 && y >= 0 && x < MiscConstants.STRIP_W && y < MiscConstants.STRIP_H) {
            buffer.setRGB(getPixelIndex(x, y), color.getRed(), color.getGreen(), color.getBlue());
        }
    }

    private void setImage(BufferedImage img) {

        for (int x = 0; x < img.getWidth(); ++x) {
            for (int y = 0; y < img.getHeight(); ++y) {
                int c = img.getRGB(x, y);
                // Color v = new Color(((c & 0xFF0000) >> 16), ((c & 0xFF00) >> 8), ((c &
                // 0xFF)));
                Color v = new Color(c);
                setPixel(x, MiscConstants.STRIP_H - 1 - y, v);
            }
        }
    }

    private static BufferedImage loadImage(String name) {
        File file = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "images", name).toFile();
        BufferedImage img;

        try {
            img = ImageIO.read(file);
            return img;
        } catch (IOException e) {
            System.err.println(e);
            return null;
        }
    }

    private static Supplier<BufferedImage> staticImageSupplier(String path) {
        return new Supplier<BufferedImage>() {
            BufferedImage img = loadImage(path);

            @Override
            public BufferedImage get() {
                return img;
            }
        };
    }

    public void setState(LightState state) {
        Lights.state = state;
    }
}
