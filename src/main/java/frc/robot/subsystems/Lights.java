package frc.robot.subsystems;

import javax.sound.midi.MidiChannel;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.MiscConstants;

public class Lights extends SubsystemBase {
    AddressableLED driver;
    AddressableLEDBuffer buffer;

    public enum States {
        CONE("cone.png"),
        CUBE("cube.png"),
        LOGO("logo.png");

        private String path;

        private States(String path) {
            this.path = path;
        }
    }

    public static States state = States.LOGO;

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
        readImage(state.path);
        driver.setData(buffer);
        driver.start();
    }

    public void rainbow() {
        for (int i = 0; i < buffer.getLength(); ++i) {
            buffer.setHSV(i, (int)((float)i/buffer.getLength() * 180) % 180 + (int)(Timer.getFPGATimestamp() * 45), 255,128);
        }
    }

    public void test() {
        for (int i = 0; i < buffer.getLength(); ++i) {
            buffer.setRGB(i, 0, 0, 0);
        }

        buffer.setRGB(getPixelIndex(0, 0), 255, 255, 255);
        buffer.setRGB(getPixelIndex(0,1), 255, 255, 255);
        buffer.setRGB(getPixelIndex(1,0), 255, 255, 255);
    }

    private int getPixelIndex(int x, int y) {
        x = MiscConstants.STRIP_W-1 - x;
        y = MiscConstants.STRIP_H-1 - y;

        return x * MiscConstants.STRIP_H + (((x&1)==1) ? MiscConstants.STRIP_H-1-y : y);
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); ++i) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }


    private void setPixel(int x, int y, Color color) {
        if (x >= 0 && y >=0 && x < MiscConstants.STRIP_W && y < MiscConstants.STRIP_H) {
            buffer.setRGB(getPixelIndex(x, y), (int)(color.red * 255),(int)(color.green* 255),(int)(color.blue * 255));
        }
    }

    private void readImage(String path) {
        File file = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "images", path).toFile();
        BufferedImage img;

        try {
            img = ImageIO.read(file);
            for(int x = 0; x < img.getWidth(); ++x ) {
                for(int y = 0; y < img.getHeight(); ++y) {
                    int c = img.getRGB(x, y);
                    Color v = new Color(((c & 0xFF0000) >> 16), ((c & 0xFF00) >> 8), ((c & 0xFF)));
                    setPixel(x,MiscConstants.STRIP_H-1-y, v);
                }
            }
        } 
        catch (IOException e) {
            System.out.println(e);
        }

        
    }

    public void setState(States state) {
        Lights.state = state;
    }
}
