package frc.robot.PathPlanningThing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathPoseThing{
    
    public Pose2d m_pose;
    public double m_deadzone;
    public double m_maxVelocity;
    public double m_maxAngularVelocity;
    public boolean m_isImportant = true;
    public Translation2d m_translation;

    public double xPosition;
    public double yPosition;
    public double rotation = -1000;
    /*
     * Make it have a positionX and positionY variable that gets set by the json then in the constructor 
     * they will make m_pose = new Pose2d(PositionX, positionY, Rotation)
     */

    public PathPoseThing(){
        // if (rotation == -1000){ // random value it should never be so this only happens if it isn't set
        //     m_translation = new Translation2d(xPosition, yPosition);
        // }else{
        //     m_pose = new Pose2d(xPosition, yPosition, new Rotation2d(rotation));
        // }
    }

    public PathPoseThing(Pose2d pose, double deadzone, double maxVelocity, double maxAngularVelocity){
        m_pose = pose;
        m_deadzone = deadzone;
        m_maxVelocity = maxVelocity;
        m_maxAngularVelocity = maxAngularVelocity;
    }

    public PathPoseThing(Translation2d translation, double deadzone, double maxVelocity, double maxAngularVelocity, boolean isImportant){
        m_translation = translation;
        m_deadzone = deadzone;
        m_maxVelocity = maxVelocity;
        m_maxAngularVelocity = maxAngularVelocity;
        m_isImportant = isImportant;
    }

    public double getDistance(PathPoseThing nextPose) {

        if (rotation == -1000){
            try{
                return m_translation.getDistance(nextPose.m_pose.getTranslation());
            }catch(Exception e){
                return m_translation.getDistance(nextPose.m_translation);
            }
        }else{
            try{
                return m_pose.getTranslation().getDistance(nextPose.m_pose.getTranslation());
            }catch(Exception e){
                return m_pose.getTranslation().getDistance(nextPose.m_translation);
            }
        }
        
    }
}
