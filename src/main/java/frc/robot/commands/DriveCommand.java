package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.util.Units;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;
    
    private double ORIENTATION = 0;

    private enum DriveState {
        Free,
        Locked
    };

    private DriveState state = DriveState.Free;

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        dsratelimiter.reset(SLOWMODE_MULT);

        addRequirements(swerveSubsystem);
    }

    double clamp(double v, double mi, double ma) {
        return (v < mi) ? mi : (v > ma ? ma : v);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {

        double rightX = DeadBand(xbox.getRightX(), 0.05);
        double rightY = DeadBand(xbox.getRightY(), 0.05);

        if (Math.abs(rightX) > 0 || Math.abs(rightY) > 0){
            this.ORIENTATION = Units.radiansToDegrees(Math.atan2(rightX, -rightY));
        }
        
        double aSpeedRaw = (Units.radiansToDegrees(swerveSubsystem.getHeading()) + this.ORIENTATION )/20;
        double aSpeedRawClock = (Units.radiansToDegrees(swerveSubsystem.getHeading()) + this.ORIENTATION + 360 )/20;
        double aSpeed = Math.max(Math.min(aSpeedRaw, 1), -1);
        double aSpeedClock = Math.max(Math.min(aSpeedRawClock, 1), -1);

        Translation2d xyRaw = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
        Translation2d xySpeed = DeadBand(xyRaw, 0.15);
        double zSpeed = DeadBand(xbox.getRightX(), 0.1);
        double xSpeed = xySpeed.getX(); // xbox.getLeftX();
        double ySpeed = xySpeed.getY(); // xbox.getLeftY();

        zSpeed = aSpeed;

        SmartDashboard.putNumber("Orientation", this.ORIENTATION);
        SmartDashboard.putNumber("aSpeed", aSpeed);
        SmartDashboard.putNumber("heading", Units.radiansToDegrees(swerveSubsystem.getHeading()));

        // TODO: Full speed!
        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        // double dmult = dsratelimiter.calculate(xbox.getRightBumper() ? 1.0 :
        // SLOWMODE_MULT);
        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        if (xbox.getXButton()) {
            swerveSubsystem.zeroHeading();
            swerveSubsystem.resetOdometry(new Pose2d());
            swerveSubsystem.zeroHeading();
        }

        ChassisSpeeds speeds;

        // Drive Non Field Oriented
        if (!xbox.getLeftBumper()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, -zSpeed,
                    new Rotation2d(
                            -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET).getRadians()));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, -zSpeed);
        }

        // State transition logic
        switch (state) {
            case Free:
                state = xbox.getBButton() ? DriveState.Locked : DriveState.Free;
                break;
            case Locked:
                state = ((xyRaw.getNorm() > 0.15) && !xbox.getBButton()) ? DriveState.Free : DriveState.Locked;
                break;
        }

        // Drive execution logic
        switch (state) {
            case Free:
                SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
                swerveSubsystem.setModules(calculatedModuleStates);
                break;
            case Locked:
                swerveSubsystem.setXstance();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}
