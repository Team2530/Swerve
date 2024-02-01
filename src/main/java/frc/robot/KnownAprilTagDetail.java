package frc.robot;

import java.time.LocalDateTime;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;

public class KnownAprilTagDetail {
    private AprilTag _tag;
    private LocalDateTime _tagCaptureTime;
    private Pose3d _tagPose3d;
    public KnownAprilTagDetail(AprilTag tag, LocalDateTime tagCaptureTime, Pose3d tagPose3d){
        _tag = tag;
        _tagCaptureTime = tagCaptureTime;
        _tagPose3d = tagPose3d;
    }

    public void SetTagCaptureTime(LocalDateTime tagCaptureTime){
        _tagCaptureTime = tagCaptureTime;
    }
    
    public LocalDateTime GetTagCaptureTime(){
        return _tagCaptureTime;
    }

    public void SetTagPose3d(Pose3d tagPose3d){
        _tagPose3d = tagPose3d;
    }

    public Pose3d GetTagPose3d(){
        return _tagPose3d;
    }

    public AprilTag GetAprilTag() {
        return _tag;
    }
}
