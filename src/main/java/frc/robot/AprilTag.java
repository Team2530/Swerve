package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AprilTagPosition;
import frc.robot.Constants.AprilTagType;

public class AprilTag {
    private String _tagId;
    private AprilTagType _tagType;
    private AprilTagPosition _tagPosition;
    private Alliance _alliance;

    public AprilTag(String tagId, AprilTagType tagType, AprilTagPosition tagPosition, Alliance alliance){
        _tagId = tagId;
        _tagType = tagType;
        _tagPosition = tagPosition;
        _alliance = alliance;
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
}