package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

public class PrepShooterCommand extends Command {

    private Intake intake;
    private Shooter shooter;

    public PrepShooterCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        intake.coast();
        shooter.coast();
        shooter.setMode(ShooterMode.FULL);
        SmartDashboard.putString("Shootake", "Spooling Shooter");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        // piece is behind front limit switch and is clear
        return shooter.isUpToSpeed();
    }
}
