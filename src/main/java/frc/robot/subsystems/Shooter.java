package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.PrepShooterCommand;

public class Shooter extends SubsystemBase {
    public enum ShooterMode {
        STOPPED(0.0),
        FULL(1.0),
        REVERSE(-0.2),
        IDLE(0.1),
        CUSTOM(1.5);

        private double modeSpeed;

        private ShooterMode(double modeSpeed) {
            this.modeSpeed = modeSpeed;
        }
    }

    // current wanted setpoint "mode" of the shooter
    private ShooterMode shooterMode = ShooterMode.STOPPED;

    // Falcon 500 shooter motor
    private final TalonFX shooterMotor = new TalonFX(ArmConstants.SHOTER_MOTOR_PORT);

    private double outputPercent = 0.0;

    private final PIDController shooterPID = new PIDController(0.001, 0.0, 0.0);

    public Shooter() {
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        double percent = shooterPID.calculate(shooterMotor.get(), outputPercent);
        shooterMotor.set(percent);
    }

    public void setMode(ShooterMode mode) {
        shooterMode = mode;
        outputPercent = shooterMode.modeSpeed;
    }

    public void setCustomPercent(double percent) {
        shooterMode = ShooterMode.CUSTOM;
        outputPercent = percent;
    }

    public double getOutputPercent() {
        return outputPercent;
    }

    /**
     * Sets the shooter motor into coast mode
     */
    public void coast() {
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets the shooter motor into brake mode
     */
    public void brake() {
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean isUpToSpeed() {
        return Math.abs(shooterMotor.get() - outputPercent) < 0.05;
    }
}
