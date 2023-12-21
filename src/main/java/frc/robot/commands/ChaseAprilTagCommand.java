package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;
import static edu.wpi.first.math.MathUtil.clamp;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class ChaseAprilTagCommand extends CommandBase {
    
    private final SwerveSubsystem swerveSubsystem;
    private final  SimpleWidget targetX;
    private final SimpleWidget targetY;
    private final SimpleWidget targetZ;
    private final SimpleWidget xSpeedWidget;
    private final SimpleWidget ySpeedWidget;
    private final SimpleWidget omegaSpeedWidget;
    private final SimpleWidget tagErrorWidget;

    private final PIDController pidControllerX = new PIDController(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD);
    private final PIDController pidControllerY = new PIDController(AutoConstants.Y_kP, AutoConstants.Y_kI, AutoConstants.Y_kD);
    private final PIDController pidControllerOmega = new PIDController(AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD);
    
    StringLogEntry log;
    //double x = 50;
    //double y = 50;
    //double z = 1.00;

    public ChaseAprilTagCommand(
        SwerveSubsystem swerveSubsystem,
        SimpleWidget targetX,
        SimpleWidget targetY,
        SimpleWidget targetZ,
        SimpleWidget xSpeedWidget,
        SimpleWidget ySpeedWidget,
        SimpleWidget omegaSpeedWidget,
        SimpleWidget tagErrorWidget) 
    {
        this.swerveSubsystem = swerveSubsystem;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetZ= targetZ;
        this.xSpeedWidget = xSpeedWidget;
        this.ySpeedWidget = ySpeedWidget;
        this.omegaSpeedWidget = omegaSpeedWidget;
        this.tagErrorWidget = tagErrorWidget;
        addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset();

    pidControllerX.setSetpoint(Units.inchesToMeters(36)); // Move forward/backwork to keep 36 inches from the target
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    pidControllerY.setTolerance(Units.inchesToMeters(2.5));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(0)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));

    if(CommonConstants.LOG_INTO_FILE_ENABLED){
      /// Starts recording to data log
      try {
        String directory = Paths.get("").toAbsolutePath().toString();
        Files.createDirectories(Path.of(directory));
        DataLogManager.start(directory);
      } catch (IOException e) {
        tagErrorWidget.getEntry().setValue(e.getMessage());
      }
      // Record both DS control and joystick data
      DriverStation.startDataLog(DataLogManager.getLog());
      log = new StringLogEntry(DataLogManager.getLog(), "ChaseApriltagCommand");
    }

  }

  //Need help here does not know how to move robot using limelight co-ordinates
  @Override
  public void execute() {
    if(CommonConstants.LOG_INTO_FILE_ENABLED){
      String logMessage = "Chase April Tag Command execute";
      logMessage += " test log again ";
      log.append(logMessage);
    }
    
    try{
      ChassisSpeeds speeds;
      // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
      LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults((LimelightConstants.limeLightName));
     //System.out.println("I am inside april tag command execute");
      if(results.targetingResults.targets_Fiducials.length > 0){
        Pose3d pose = results.targetingResults.targets_Fiducials[0].getTargetPose_RobotSpace();
          // if(x > 0.8676){
          //   x = x - 1;
          // }
          // else{
          //   x = 0.8676;
          // }

          // if( y > 0){
          //   y = y - 1;
          // }
          // else {
          //   y = 0;
          // }

          // if(z < 3.14159){
          //   z = z + 0.2;
          // }
          // else {
          //   z = 3.14159;
          // }
          //Pose3d pose = new Pose3d(new Translation3d(x, y, 1), new Rotation3d(0, 0, z));
          //var temp = pose.getX() + ": Set Point: " + pidControllerX.getSetpoint() + "; Is At Set Point: " + pidControllerX.atSetpoint();

          targetX.getEntry().setValue(pose.getZ());
          SmartDashboard.putNumber("x SetPoint", pidControllerX.getSetpoint());
          SmartDashboard.putBoolean("Is x At Set Point", pidControllerX.atSetpoint());

          targetY.getEntry().setValue(pose.getX());
          targetZ.getEntry().setValue(pose.getRotation().getY());
          
          var xSpeed = pidControllerX.calculate(pose.getZ());          
          if (pidControllerX.atSetpoint()) {
            xSpeed = 0;
          }
          xSpeedWidget.getEntry().setValue(xSpeed);
    
            // Handle alignment side-to-side
          var ySpeed = 0;//pidControllerY.calculate(pose.getX());
          if (pidControllerY.atSetpoint()) {
            ySpeed = 0;
          }
          ySpeedWidget.getEntry().setValue(ySpeed);

          // Handle rotation using target Yaw/Z rotation
          var omegaSpeed = 0;//pidControllerOmega.calculate(pose.getRotation().getY());
          if (pidControllerOmega.atSetpoint()) {
            omegaSpeed = 0;
          }
          omegaSpeedWidget.getEntry().setValue(omegaSpeed);
    
          if(CommonConstants.LOG_INTO_FILE_ENABLED){
            String logMessage = "target X: " + pose.getX() + ": ";
            logMessage += "target X(inches): " + Units.metersToInches(pose.getX()) + ": ";
            logMessage += "X speed : " + xSpeed + ": ";
            logMessage += "X SetPoint : " + pidControllerX.getSetpoint() + ": ";
            logMessage += "X is at SetPoint : " + pidControllerX.atSetpoint() + ": ";
            logMessage += "target Y: " + pose.getY() + ": ";
            logMessage += "Y speed : " + ySpeed + ": ";
            logMessage += "Y SetPoint : " + pidControllerY.getSetpoint() + ": ";
            logMessage += "Y is at SetPoint : " + pidControllerY.atSetpoint() + ": ";
            logMessage += "target rotation: " + pose.getRotation().getZ() + ": ";
            logMessage += "rotation speed : " + omegaSpeed + ": ";
            logMessage += "rotation SetPoint : " + pidControllerOmega.getSetpoint() + ": ";
            logMessage += "rotation is at SetPoint : " + pidControllerOmega.atSetpoint() + ": ";
            log.append(logMessage);
          }          
          //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, -zSpeed, swerveSubsystem.getRotation2d());
            speeds = new ChassisSpeeds(-xSpeed, -ySpeed, -omegaSpeed);

            SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
            swerveSubsystem.setModules(calculatedModuleStates);
      }
      else{
        swerveSubsystem.stopDrive();
      }
    }
    catch(Exception e){
      tagErrorWidget.getEntry().setValue(e.getMessage());
    }
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

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }
}