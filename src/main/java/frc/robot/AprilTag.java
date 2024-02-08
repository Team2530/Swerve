package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;

public class AprilTag {
    private String _tagId;
    private AprilTagType _tagType;
    private AprilTagPosition _tagPosition;
    private Alliance _alliance;
    private Double _x;
    private Double _y;
    private Double _z;
    private double _rotation;

    public AprilTag(String tagId, AprilTagType tagType, AprilTagPosition tagPosition, Alliance alliance, Double x, Double y, Double z, Double rotation){
        _tagId = tagId;
        _tagType = tagType;
        _tagPosition = tagPosition;
        _alliance = alliance;
        _x = x;
        _y = y;
        _z = z;
        _rotation = rotation;
    }

    public AprilTagType GetTagType(){
        return _tagType;
    }

    public String GetTagId(){
        return _tagId;
    }

    public AprilTagPosition GetTagPosition(){
        return _tagPosition;
    }

    public Alliance GetAlliance(){
        return _alliance;
    }

    public Pose2d getPose2d(){
        return new Pose2d(new Translation2d(_x, _y), new Rotation2d(Units.degreesToRadians(_rotation)));
    }
}