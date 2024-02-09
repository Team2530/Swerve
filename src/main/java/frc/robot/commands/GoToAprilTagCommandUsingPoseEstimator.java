package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionContsants;
import frc.robot.AprilTag;
import frc.robot.Constants;
import frc.robot.KnownAprilTagDetail;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;
import frc.robot.Constants.CommonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Enumeration;
import java.util.Optional;
import java.util.function.Supplier;

public class GoToAprilTagCommandUsingPoseEstimator extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    public final Supplier<Pose2d> robotPoseSupplier;
    public final AprilTagPosition tagPosition;
    public final AprilTagType tagType;
    public final AprilTag tag;

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(VisionContsants.X_kP, VisionContsants.X_kI, VisionContsants.X_kD, LimelightConstants.pidXConstriants);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(VisionContsants.Y_kP, VisionContsants.Y_kI, VisionContsants.Y_kD, LimelightConstants.pidYConstraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(VisionContsants.THETA_kP, VisionContsants.THETA_kI, VisionContsants.THETA_kD, LimelightConstants.pidOmegaConstraints);
    
    StringLogEntry log;

    public GoToAprilTagCommandUsingPoseEstimator(
        SwerveSubsystem swerveSubsystem,
        Supplier<Pose2d> robotPoseSupplier,
        AprilTagPosition tagPosition,
        AprilTagType tagType) 
    {
        this.swerveSubsystem = swerveSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.tagPosition = tagPosition;
        this.tagType = tagType;
        this.tag = getApirlTag(tagPosition, tagType);
        addRequirements(swerveSubsystem);
    }

  @Override
  public void initialize() {
    super.initialize();
    if(tag != null)
    {
      Pose2d goalPose = tag.getPose2d();
      if(goalPose != null){
        resetPIDControllers();
        pidControllerX.setGoal(Units.inchesToMeters(goalPose.getX())); // Move forward/backwork to keep 36 inches from the target
        pidControllerX.setTolerance(Units.inchesToMeters(2.5));

        pidControllerY.setGoal(goalPose.getY()); // Move side to side to keep target centered
        pidControllerY.setTolerance(Units.inchesToMeters(2.5));

        pidControllerOmega.setGoal(Units.degreesToRadians(goalPose.getRotation().getRadians())); // Rotate the keep perpendicular with the target
        pidControllerOmega.setTolerance(Units.degreesToRadians(1));

        if(CommonConstants.LOG_INTO_FILE_ENABLED){
          /// Starts recording to data log
          try {
            String directory = Paths.get("").toAbsolutePath().toString();
            Files.createDirectories(Path.of(directory));
            DataLogManager.start(directory);
          } catch (IOException e) {
            SmartDashboard.putString("AprilTag Log error", e.getMessage());
          }
          // Record both DS control and joystick data
          DriverStation.startDataLog(DataLogManager.getLog());
          log = new StringLogEntry(DataLogManager.getLog(), "GoToAprilTagCommand");
        }
      }
    }
  }

  @Override
  public void execute() {
    try{
        ChassisSpeeds speeds;
        Pose2d robotPose = robotPoseSupplier.get();
        if(robotPose != null){
         if(CommonConstants.LOG_INTO_FILE_ENABLED){
            SmartDashboard.putString("Current tagID", tag.GetTagId());
            SmartDashboard.putNumber("Current pose X", robotPose.getX());
            SmartDashboard.putNumber("Current pose Y", robotPose.getY());
            SmartDashboard.putNumber("Current pose Rotation", robotPose.getRotation().getRadians());
          }
          var xSpeed = pidControllerX.calculate(robotPose.getX());          
          if (pidControllerX.atSetpoint()) {
            xSpeed = 0;
          }

          // Handle alignment side-to-side
          var ySpeed = 0;//pidControllerY.calculate(robotPose.getY());
          if (pidControllerY.atSetpoint()) {
            ySpeed = 0;
          }

          // Handle rotation using target Yaw/Z rotation
          var omegaSpeed = 0;//pidControllerOmega.calculate(robotPose.getRotation().getDegrees());
          if (pidControllerOmega.atSetpoint()) {
            omegaSpeed = 0;
          }

          if(CommonConstants.LOG_INTO_FILE_ENABLED){
            String logMessage = "target X: " + robotPose.getX() + ": ";
            logMessage += "target X(inches): " + Units.metersToInches(robotPose.getX()) + ": ";
            logMessage += "X speed : " + xSpeed + ": ";
            logMessage += "X SetPoint : " + pidControllerX.getSetpoint() + ": ";
            logMessage += "X is at SetPoint : " + pidControllerX.atSetpoint() + ": ";
            logMessage += "target Y: " + robotPose.getY() + ": ";
            logMessage += "Y speed : " + ySpeed + ": ";
            logMessage += "Y SetPoint : " + pidControllerY.getSetpoint() + ": ";
            logMessage += "Y is at SetPoint : " + pidControllerY.atSetpoint() + ": ";
            logMessage += "target rotation: " + robotPose.getRotation().getRadians() + ": ";
            logMessage += "rotation speed : " + omegaSpeed + ": ";
            logMessage += "rotation SetPoint : " + pidControllerOmega.getSetpoint() + ": ";
            logMessage += "rotation is at SetPoint : " + pidControllerOmega.atSetpoint() + ": ";
            log.append(logMessage);
          }          
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, omegaSpeed, robotPose.getRotation());

          SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
          swerveSubsystem.setModules(calculatedModuleStates); 
        }
    }
    catch(Exception e){
      SmartDashboard.putString("AprilTag pose command error", e.getMessage());
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }

  public boolean atGoal() {
    return pidControllerX.atGoal() && pidControllerY.atGoal() && pidControllerOmega.atGoal();
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  private void resetPIDControllers() {
    var robotPose =  robotPoseSupplier.get();
    pidControllerOmega.reset(robotPose.getRotation().getRadians());
    pidControllerX.reset(robotPose.getX());
    pidControllerY.reset(robotPose.getY());
  }

  public AprilTag getApirlTag(AprilTagPosition tagPosition, AprilTagType tagType)
  {
      // if there is an alliance it gets the alliance (blue or red)
      Optional<Alliance> alliance = DriverStation.getAlliance();
      AprilTag returnValue = null;
      // loops through the hashtable and finds the correct apriltag and returns the details
      if(alliance.isPresent()){
          Enumeration<String> e = Constants.AllAprilTags.keys();
          AprilTag tag = null;
          while(e.hasMoreElements()) {
              String key = e.nextElement();
              tag = Constants.AllAprilTags.get(key);
              if(tag != null && tag.GetAlliance() == alliance.get() && tag.GetTagPosition() == tagPosition && tag.GetTagType() == tagType){
                  returnValue = tag;
                  break;
              }
          }
      }
      return returnValue;
  }

}
