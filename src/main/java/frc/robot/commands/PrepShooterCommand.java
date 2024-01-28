package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterMode;

public class PrepShooterCommand extends Command {
    
    private final Shooter shooter;

    public PrepShooterCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.coast();
        shooter.setMode(ShooterMode.FULL);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

}
