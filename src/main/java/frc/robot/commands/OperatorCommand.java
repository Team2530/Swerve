package frc.robot.commands;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Lights.States;

public class OperatorCommand extends CommandBase {
    private final XboxController xbox;
    private final Intake intake;

    public OperatorCommand(Intake intake, XboxController xbox) {
        this.xbox = xbox;
        this.intake = intake;

        addRequirements(intake);
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
        if (DriverStation.isTeleop()) {
            boolean intakeIn = ((intake.getIntakeState() == IntakeState.TIPPEDCONE_CUBE) || (intake.getIntakeState() == IntakeState.STOWED) || (intake
                    .getIntakeState() == IntakeState.PICKUP)) && !xbox.getBButton();

            if (xbox.getRightBumper()) {
                    if (intakeIn) 
                        intake.setIntakeSpeed(-0.7);
                    else {
                        intake.setIntakeSpeed(1.0);
                        intake.enableCurrentControl(false);
                    }
            } else {
                intake.enableCurrentControl(true);
                intake.setIntakeSpeed(xbox.getRightTriggerAxis() * (intakeIn ? -1 : 1) - 0.2); // Max 80% output!
            }

            intake.enableStateControl(true);
            
            // Set intake state based on the xbox POV
            switch (xbox.getPOV()) {
                case -1:
                    break;
                case 0:
                    intake.setIntakeState(IntakeState.HIGH);
                    break;
                case 90:
                    intake.setIntakeState(IntakeState.STOWED);
                    break;
                case 180:
                    intake.setIntakeState(IntakeState.PICKUP);
                    RobotContainer.lights.setState(States.CONE);
                    break;
                case 270:
                    intake.setIntakeState(IntakeState.LOW);
                    break;
            }

            if (xbox.getLeftBumper()) {
                RobotContainer.lights.setState(States.CUBE);
                intake.setIntakeState(IntakeState.TIPPEDCONE_CUBE);
            }

            intake.addIntakeState((0.02 * DeadBand(xbox.getLeftY(), 0.1)) * 60.0);
        } else {
            // Auton
            intake.setIntakeSpeed(0.0);
            intake.setIntakeState(IntakeState.STOWED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}