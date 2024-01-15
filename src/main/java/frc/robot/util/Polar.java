package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Polar {
    private final double r;
    private double theta;

    /**
     * 
     * @param r radius
     * @param theta angle theta
     */
    public Polar(double r, double theta) {
        this.r = r;
        this.theta = theta;
    }

    public double getR() {
        return r;
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public Translation2d getTranslation2d() {
        return new Translation2d(r, new Rotation2d(theta));
    }
}
