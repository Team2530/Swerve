package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.JointedArm;
import frc.robot.util.Polar;

public class Shootake extends SubsystemBase {

    private Polar mastArm = new Polar(ArmConstants.INNER_MAST_LENGTH, 0);
    private Polar shooter = new Polar(ArmConstants.SHOOTER_LENGTH, 0);
    private Polar intake = new Polar(ArmConstants.INTAKE_LENGTH, 0);

    JointedArm shooterArm = new JointedArm(mastArm, shooter);    
    JointedArm intakeArm = new JointedArm(mastArm, intake);

    CANcoder mastEncoder = new CANcoder(ArmConstants.MAST_ARM_ENCODER_PORT);
    CANcoder shootakeEncoder = new CANcoder(ArmConstants.SHOOTAKE_ENCODER_PORT);

    private static Mechanism2d shootake2d = new Mechanism2d(50, 50);

    MechanismRoot2d root = shootake2d.getRoot("shootake", 25, 0);
    MechanismLigament2d mast2d = null;
    MechanismLigament2d shooter2d = null;
    MechanismLigament2d intake2d = null;

    public enum ShootakeState {
        STOWED(0, 0),
        SHOOT(0, 0),
        INTAKE(0, 0),
        AMP(0 , 0),
        AUTO(0, 0);

        private double theta1;
        private double theta2;

        private ShootakeState(double theta1, double theta2) {
            this.theta1 = theta1;
            this.theta2 = theta2;
        }
    }

    ShootakeState currentState;

    public Shootake() {
        currentState = ShootakeState.AUTO;
        MechanismLigament2d mastbar = root.append(new MechanismLigament2d("mastbar", ArmConstants.MAST_CENTER_OF_ROTATION.getY(), 90));
        mast2d = mastbar.append(new MechanismLigament2d("mast", ArmConstants.INNER_MAST_LENGTH, 90, 10, new Color8Bit(255, 255, 255)));
        shooter2d = mast2d.append(new MechanismLigament2d("shooter", ArmConstants.SHOOTER_LENGTH, 90, 10, new Color8Bit(0, 0, 255)));
        intake2d = mast2d.append(new MechanismLigament2d("intake", ArmConstants.INTAKE_LENGTH, -90, 10, new Color8Bit(255, 0, 0)));

        SmartDashboard.putData("Shootake", shootake2d);
    }
    

    @Override
    public void periodic() {
        updateArmState();


        
    }

    private void updateArmState() {
        // update rotation dependencies
        double shooterAngle = shootakeEncoder.getPosition().getValue();
        double mastAngle = mastEncoder.getPosition().getValue();

        intake.setTheta(shooterAngle);
        shooter.setTheta(shooterAngle);
        mastArm.setTheta(mastAngle);

        mast2d.setAngle(new Rotation2d(Units.degreesToRadians(mastAngle - 90)));
        intake2d.setAngle(new Rotation2d(Units.degreesToRadians(shooterAngle - mast2d.getAngle())));
        shooter2d.setAngle(new Rotation2d(Units.degreesToRadians(shooterAngle - mast2d.getAngle() - 180)));
    }

    @Override
    public void simulationPeriodic() {
        updateArmState();

        shootakeEncoder.setPosition(shootakeEncoder.getPosition().getValue() + .1);
        mastEncoder.setPosition(mastEncoder.getPosition().getValue() + .3);

    }
}
