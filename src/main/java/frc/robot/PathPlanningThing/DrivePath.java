package frc.robot.PathPlanningThing;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.fields.Fields;
import org.wpilib.command2.Command;

import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;
  
public class DrivePath extends Command{
    
    private PathPoseThing[] m_pose;
    private final SwerveSubsystem m_swerveSubsystem = Subsystems.m_swerveSubsystem;
    private int m_point = 0;
    private final double alignDeadzone = 1.1;
    private int m_rotationIndex = -1;

    private ObjectMapper m_jsonMapper = new ObjectMapper();
    
    private Optional<DoubleSupplier> m_commandedRotation = Optional.empty();

    private final boolean m_resetPose;

    public DrivePath(PathPoseThing[] pose, boolean resetPose) {
        m_pose = pose;

        m_resetPose = resetPose;
        
    }
    public DrivePath(boolean resetPose){
        m_resetPose = resetPose;
        m_jsonMapper.configure(DeserializationFeature.USE_JAVA_ARRAY_FOR_JSON_ARRAY, true);
        Path jsonPath = Paths.get("./src/main/java/frc/robot/PathPlanningThing/Autos/" + SelectAuto.m_autoSelection.getSelected()).toAbsolutePath();
        String json = "smth";
        try {
            json = Files.readString(jsonPath);
        } catch (IOException e) {
           System.err.println("please no error :/");
        }
        try {
            m_pose = m_jsonMapper.readValue(json, PathPoseThing[].class);

            for (int i = 0; i < m_pose.length; i++){
            if (m_pose[i].rotation == -3000){ // random value it should never be so this only happens if it isn't set
                m_pose[i].m_translation = new Translation2d(m_pose[i].xPosition, m_pose[i].yPosition);
            }else{
                m_pose[i].m_pose = new Pose2d(m_pose[i].xPosition, m_pose[i].yPosition, new Rotation2d(m_pose[i].rotation));
            }
            }
            
        } catch (JsonMappingException e) {System.err.println("m_pose can't be set, mapping error" + e);}
        catch (JsonProcessingException e) {System.err.println("m_pose can't be set, processing error");}
        
        
        
        addRequirements(m_swerveSubsystem);
    }


    /**
     * Used for constructing a new Auto
     * @param AutoName
     * This is the name of the auto as <b>WITH THE JSON FILE EXTENSION </b> in the PathPlanningThing/Autos folder
     */
    public DrivePath(String AutoName, boolean mirror, boolean resetPose){
        m_resetPose = resetPose;
        m_jsonMapper.configure(DeserializationFeature.USE_JAVA_ARRAY_FOR_JSON_ARRAY, true);
        Path jsonPath = Paths.get("./src/main/java/frc/robot/PathPlanningThing/Autos/" + AutoName).toAbsolutePath();
        String json = "smth";
        try {
            json = Files.readString(jsonPath);
        } catch (IOException e) {
           System.err.println("please no error :/");
        }
        try {
            m_pose = m_jsonMapper.readValue(json, PathPoseThing[].class);

            for (int i = 0; i < m_pose.length; i++){
            if (m_pose[i].rotation == -1000){ // random value it should never be so this only happens if it isn't set
                m_pose[i].m_translation = new Translation2d(m_pose[i].xPosition, m_pose[i].yPosition);
            }else{
                m_pose[i].m_pose = new Pose2d(m_pose[i].xPosition, m_pose[i].yPosition, new Rotation2d(m_pose[i].rotation));
            }
            }

            if (mirror){
                for (PathPoseThing i : m_pose) {
                    i.yPosition = Math.abs(i.yPosition - Fields.DEFAULT_FIELD.width);
                }
            }
        } catch (JsonMappingException e) {System.err.println("m_pose can't be set, mapping error" + e);}
        catch (JsonProcessingException e) {System.err.println("m_pose can't be set, processing error");}
        
        
        addRequirements(m_swerveSubsystem);
    }

    /**
     * Used for constructing a new Auto with a rotation from somewhere else
     * @param AutoName
     * This is the name of the auto as <b>WITH THE JSON FILE EXTENSION </b> in the PathPlanningThing/Autos folder
     * 
     * @param rotation
     * This is the rotational velocity. This parameter is intended to be used with paths made of translations where the robot must look somewhere
     * It should be calculated from a PID rotation as it is a <b>rate of change</b> not a <b>target rotation</b>
     * <b>This will override Pose Waypoint's Rotation</b>
     */
    public DrivePath(String AutoName, DoubleSupplier rotation, boolean resetPose){
        m_resetPose = resetPose;
        m_commandedRotation = Optional.of(rotation);

        m_jsonMapper.configure(DeserializationFeature.USE_JAVA_ARRAY_FOR_JSON_ARRAY, true);
        Path jsonPath = Paths.get("./src/main/java/frc/robot/PathPlanningThing/Autos/" + AutoName).toAbsolutePath();
        String json = "smth";
        try {
            json = Files.readString(jsonPath);
        } catch (IOException e) {
           System.err.println("please no error :/");
        }
        try {
            m_pose = m_jsonMapper.readValue(json, PathPoseThing[].class);

            for (int i = 0; i < m_pose.length; i++){
            if (m_pose[i].rotation == -1000){ // random value it should never be so this only happens if it isn't set
                m_pose[i].m_translation = new Translation2d(m_pose[i].xPosition, m_pose[i].yPosition);
            }else{
                m_pose[i].m_pose = new Pose2d(m_pose[i].xPosition, m_pose[i].yPosition, new Rotation2d(m_pose[i].rotation));
            }
            }
        } catch (JsonMappingException e) {System.err.println("m_pose can't be set, mapping error" + e);}
        catch (JsonProcessingException e) {System.err.println("m_pose can't be set, processing error");}
        
        
        addRequirements(m_swerveSubsystem);
    }




    @Override
    public void initialize() {
        m_point = 0;
        m_rotationIndex = -1;
        if (m_resetPose)
            Subsystems.m_swerveSubsystem.resetPose(m_pose[m_point].m_pose);
        // for (int i = 0; i < m_pose.length; i++){
        //     m_totalPathDistance = m_pose[i].getDistance(m_pose[i + 1]);
        // }
    }

    @Override
    public void execute() {
        checkClosest();
        DriveToPose();
        Telemetry.log("Auto Index", m_point);
    }
    private void checkClosest(){
        var importantPose = nextImportantPose();
        if (m_pose[m_point].m_translation == null){
            return;
        }
        if (m_pose[m_point].m_isImportant){
            return;
        }
        if (importantPose == -1){
            return;
        }
        if (m_pose[importantPose].m_translation == null){
            if (m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[m_point].m_translation) > m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[importantPose].m_pose.getTranslation())){
                m_point = importantPose;
                return;
            }
        }else{
            if (m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[m_point].m_translation) > m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[importantPose].m_translation)){
                m_point = importantPose;
                return;
            }
        }
    }
    private void DriveToPose() {
        
        
        Pose2d targetPose = m_pose[m_point].m_pose;
        Translation2d targetTranslation2d = m_pose[m_point].m_translation;

        Pose2d robotPose = m_swerveSubsystem.getPose();
        Fields field = Fields.DEFAULT_FIELD;
        double x;
        double y;
        
        if (m_pose[m_point].rotation != -1000){
            x = Math.clamp(m_swerveSubsystem.m_xPID.calculate(robotPose.getX(), Math.abs(targetPose.getX() - (isRedAlliance() ? field.length : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            y = Math.clamp(m_swerveSubsystem.m_yPID.calculate(robotPose.getY(), Math.abs(targetPose.getY() - (isRedAlliance() ? field.width : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            Telemetry.log("x pose", Math.abs(targetPose.getX() - (isRedAlliance() ? field.length : 0)));
            Telemetry.log("y pose", Math.abs(targetPose.getY() - (isRedAlliance() ? field.width : 0)));
        }else{
            x = Math.clamp(m_swerveSubsystem.m_xPID.calculate(robotPose.getX(), Math.abs(targetTranslation2d.getX() - (isRedAlliance() ? field.length : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            y = Math.clamp(m_swerveSubsystem.m_yPID.calculate(robotPose.getY(), Math.abs(targetTranslation2d.getY() - (isRedAlliance() ? field.width : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            Telemetry.log("x pose", Math.abs(targetTranslation2d.getX() - (isRedAlliance() ? field.length : 0)));
            Telemetry.log("y pose", Math.abs(targetTranslation2d.getY() - (isRedAlliance() ? field.width : 0)));
        }
        
        if (m_commandedRotation.isPresent()){
            m_swerveSubsystem.drive(new ChassisVelocities(x, y, m_commandedRotation.get().getAsDouble() * (isRedAlliance() ? -1 : 1)));
            return;
        }

        if (m_pose[m_point].m_translation == null){
            m_swerveSubsystem.drive(new ChassisVelocities(x, y, orientPID(() -> Units.degreesToRadians(m_pose[m_point].m_pose.getRotation().getDegrees()) * (isRedAlliance() ? -1 : 1)).get().getDegrees()));
        }else{
        if (m_rotationIndex == -1){
            for (int i = m_point; i > m_pose.length; i++){
                if (m_pose[i].m_translation == null){
                    m_rotationIndex = i;
                    break;
                }else{
                    m_rotationIndex = -1;
                }
            }
            if (m_rotationIndex == -1) {
                for (int i = m_point; i < 0; i--){
                    if (m_pose[i].m_translation == null){
                        m_rotationIndex = i;
                        break;
                    }else{
                        m_rotationIndex = -1;
                    }
                }
            }
        }    
          
            if (m_rotationIndex == -1){
                m_swerveSubsystem.drive(new ChassisVelocities(x, y, 0));
                
            }else{
                m_swerveSubsystem.drive(new ChassisVelocities(x, y, orientPID(() -> m_pose[m_rotationIndex].m_pose.getRotation().getDegrees() * (isRedAlliance() ? -1 : 1)).get().getDegrees()));
                
            }
        }
        
        
        
    }


    public Supplier<Rotation2d> orientPID(DoubleSupplier targetRotation){
        double setPointDegrees = targetRotation.getAsDouble();
        double heading = m_swerveSubsystem.getPose().getRotation().getDegrees();

        double rotation = Math.clamp(m_swerveSubsystem.m_orientPID.calculate(heading, setPointDegrees), -m_pose[m_point].m_maxAngularVelocity, m_pose[m_point].m_maxAngularVelocity);
        return () -> Rotation2d.fromDegrees(rotation);
    }

    private boolean isRedAlliance() {
        return SwerveSubsystem.isRedAlliance();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(new ChassisVelocities(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        Telemetry.log("maxPoints", m_pose.length - 1);
        
        Pose2d targetPose = m_pose[m_point].m_pose;
        Translation2d targetTranslation2d = m_pose[m_point].m_translation;
        Fields field = Fields.DEFAULT_FIELD;

        if (targetPose != null){
            
            targetPose = new Pose2d(Math.abs(targetPose.getX() - (isRedAlliance() ? field.length : 0)), Math.abs(targetPose.getY() - (isRedAlliance() ? field.width : 0)), new Rotation2d(targetPose.getRotation().getRotations() * (isRedAlliance() ? -1 : 1)));
            if (m_swerveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < m_pose[m_point].m_deadzone){
                
                try {
                    if (Math.abs(targetPose.getRotation().getRadians() - m_swerveSubsystem.getPose().getRotation().getDegrees()) < alignDeadzone || Math.abs((targetPose.getRotation().getRadians() - 360) - m_swerveSubsystem.getPose().getRotation().getDegrees()) < alignDeadzone || Math.abs((targetPose.getRotation().getRadians() + 360) - m_swerveSubsystem.getPose().getRotation().getDegrees()) < alignDeadzone){
                        if (m_point >= m_pose.length - 1){
                            return true;
                        }else{
                            m_point ++;
                            m_rotationIndex = -1;
                        }   
                    }else {
                        return false;
                    }
                } catch(Exception e){
                    return false;
                }
            }else{
                return false;
            }
        }else{
            targetTranslation2d = new Translation2d(Math.abs(targetTranslation2d.getX() - (isRedAlliance() ? field.length : 0)), Math.abs(targetTranslation2d.getY() - (isRedAlliance() ? field.width : 0)));

            if (m_swerveSubsystem.getPose().getTranslation().getDistance(targetTranslation2d) < m_pose[m_point].m_deadzone){
                if (m_point >= m_pose.length - 1){
                    return true;
                }else{
                    m_point ++;
                    m_rotationIndex = -1;
                }
            }else{
                return false;
            }
        }
        return false;
        
    }

    private int nextImportantPose() {
        for (int i = m_point; i < m_pose.length; i++) {
            if (m_pose[i].m_translation == null){
                return i;
            }else if(m_pose[i].m_isImportant){
                return i;
            }
        }
        return -1;
        
    }
        
}
