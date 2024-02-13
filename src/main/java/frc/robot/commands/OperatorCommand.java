package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class OperatorCommand extends Command {
    private final XboxController xbox;
    private final Arm arm;

    public OperatorCommand(XboxController xbox, Arm arm) {
        this.xbox = xbox;
        this.arm = arm;

        addRequirements(arm);
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        if (xbox.getAButton()) {
            arm.setShoulder(-0.06);
        } else {
            arm.setShoulder(-0.06);
        }
        if (xbox.getBButton()) {
            arm.setWrist(20);
        } else {
            arm.setWrist(10);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}