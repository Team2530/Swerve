package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class JointedArm {
    private Polar p1;
    private Polar p2;

    public JointedArm(Polar p1, Polar p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    public Translation2d getFinalPosition() {
        return p1.getTranslation2d().plus(p2.getTranslation2d());
    }
}
