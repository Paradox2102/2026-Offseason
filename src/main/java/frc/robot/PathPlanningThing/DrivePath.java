package frc.robot.PathPlanningThing;

import java.io.File;
import java.io.IOException;
import java.math.MathContext;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Subsystems;
import frc.robot.subsystems.drive.SwerveSubsystem;

/* I think this is done! I'll need to put it through it's paces a little bit more when I get the jason working
 * so... handling rotation.
 * lets plan out how to do that maybe :\
 * So it needs to look ahead to align to the next pose's rotation.
 * what if, it only switches points/ends if the next point is a rotation and the rotation is within a fixed(?)
 * deadzone. Then it also needs to, for orient pid, have a next index with rotation value. That could be a for loop
 * where it tries to say m_pose.m_pose.getRotation. If null it's a translation and therefore doesn't have a rotation
 * otherwise we commence normally with that being the correct pose. If the path ends in translations we want to just
 * use the rotation of the last pose so if we get bumped it'll rotate back.
 * 
 * FIXED YIPPEE
 * This also can only be run once ':/ I gotta fix that
 */

 /*
  * FIXED YIPPEE
  * We also don't take into account the is important to skip a translation if we're closer to an important position
  */
  /*
   * next on todo because I don't already have enough jobs, I want it to only calculate and do the end detection every few loops
   * especially when we're far away we don't need to calculate as often to save on loop overruns.
   * Maybe make it only go through one for loop per overrun?
   * that way we can optimize so it doesn't go through that much each loop
   */
  /*
   * All translations might be an issue.
   * although this would reasonably never happen (unless this is used for PID align. It will probably be used for PID align, I need to fix this)
   * If the auto is just translations, nothing happens. We don't move at all.
   * :'D
   */
  /*
   * I think I fixed that! It's just been a while since I worked on this. Let me test some autos.
   * Now that the software is updated to be nicer to use it should be alright and not break anything. Then I can get to work on our autos maybe
   */
  
public class DrivePath extends Command{
    
    private PathPoseThing[] m_pose;
    private final SwerveSubsystem m_swerveSubsystem = Subsystems.m_swerveSubsystem;
    private int m_point = 0;
    private final double alignDeadzone = 1.1;

    private double m_totalPathDistance;
    private int m_rotationIndex = -1;

    private final double k_checkEndThreshold = 1.0; 

    private ObjectMapper m_jsonMapper = new ObjectMapper();
    
    private Optional<DoubleSupplier> m_commandedRotation = Optional.empty();

    public DrivePath(PathPoseThing[] pose) {
        m_pose = pose;


        
        
    }
    public DrivePath(){
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
            System.out.println("AAAAAAAAA" + m_pose.length);

            for (int i = 0; i < m_pose.length; i++){
            System.out.println("pls do i at least see this?");
            if (m_pose[i].rotation == -1000){ // random value it should never be so this only happens if it isn't set
                System.out.println("whu duh hecky y don't i see this");
                m_pose[i].m_translation = new Translation2d(m_pose[i].xPosition, m_pose[i].yPosition);
            }else{
                System.out.println("whu duh hecky y don't i see this, butt pose");
                m_pose[i].m_pose = new Pose2d(m_pose[i].xPosition, m_pose[i].yPosition, new Rotation2d(m_pose[i].rotation));
            }
            }
            // System.out.println("this is a string so i can find this in the console " + m_pose[0].m_pose.getX());
        } catch (JsonMappingException e) {System.err.println("m_pose can't be set, mapping error" + e);}
        catch (JsonProcessingException e) {System.err.println("m_pose can't be set, processing error");}
        
        
        
        addRequirements(m_swerveSubsystem);
    }


    /**
     * Used for constructing a new Auto
     * @param AutoName
     * This is the name of the auto as <b>WITH THE JSON FILE EXTENSION </b> in the PathPlanningThing/Autos folder
     */
    public DrivePath(String AutoName){
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
            System.out.println("AAAAAAAAA" + m_pose.length);

            for (int i = 0; i < m_pose.length; i++){
            System.out.println("pls do i at least see this?");
            if (m_pose[i].rotation == -1000){ // random value it should never be so this only happens if it isn't set
                System.out.println("whu duh hecky y don't i see this");
                m_pose[i].m_translation = new Translation2d(m_pose[i].xPosition, m_pose[i].yPosition);
            }else{
                System.out.println("whu duh hecky y don't i see this, butt pose");
                m_pose[i].m_pose = new Pose2d(m_pose[i].xPosition, m_pose[i].yPosition, new Rotation2d(m_pose[i].rotation));
            }
            }
            // System.out.println("this is a string so i can find this in the console " + m_pose[0].m_pose.getX());
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
    public DrivePath(String AutoName, DoubleSupplier rotation){

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
            System.out.println("AAAAAAAAA" + m_pose.length);

            for (int i = 0; i < m_pose.length; i++){
            System.out.println("pls do i at least see this?");
            if (m_pose[i].rotation == -1000){ // random value it should never be so this only happens if it isn't set
                System.out.println("whu duh hecky y don't i see this");
                m_pose[i].m_translation = new Translation2d(m_pose[i].xPosition, m_pose[i].yPosition);
            }else{
                System.out.println("whu duh hecky y don't i see this, butt pose");
                m_pose[i].m_pose = new Pose2d(m_pose[i].xPosition, m_pose[i].yPosition, new Rotation2d(m_pose[i].rotation));
            }
            }
            // System.out.println("this is a string so i can find this in the console " + m_pose[0].m_pose.getX());
        } catch (JsonMappingException e) {System.err.println("m_pose can't be set, mapping error" + e);}
        catch (JsonProcessingException e) {System.err.println("m_pose can't be set, processing error");}
        
        
        addRequirements(m_swerveSubsystem);
    }




    @Override
    public void initialize() {
        m_point = 0;
        m_rotationIndex = -1;
        // for (int i = 0; i < m_pose.length; i++){
        //     m_totalPathDistance = m_pose[i].getDistance(m_pose[i + 1]);
        // }
    }

    @Override
    public void execute() {
        checkClosest();
        DriveToPose();
        SmartDashboard.putNumber("Auto Index", m_point);
    }
    private void checkClosest(){
        var importantPose = nextImportantPose();
        if (m_pose[m_point].m_translation == null){
            System.out.println("It's a pose!");
            return;
        }
        if (m_pose[m_point].m_isImportant){
            System.out.println("It's Important!");
            return;
        }
        if (importantPose == -1){
            System.out.println("Nothing else is important!");
            return;
        }
        if (m_pose[importantPose].m_translation == null){
            if (m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[m_point].m_translation) > m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[importantPose].m_pose.getTranslation())){
                m_point = importantPose;
                System.out.println("We're closer to the next important pose");
                return;
            }else{
                System.out.println("We're not closer to the next important pose");
            }
        }else{
            if (m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[m_point].m_translation) > m_swerveSubsystem.getPose().getTranslation().getDistance(m_pose[importantPose].m_translation)){
                m_point = importantPose;
                System.out.println("We're closer to the next important translation");
                return;
            }else{
                System.out.println("We're not closer to the next important translation");

            }
        }
    }
    private void DriveToPose() {
        
        
        Pose2d targetPose = m_pose[m_point].m_pose;
        Translation2d targetTranslation2d = m_pose[m_point].m_translation;

        Pose2d robotPose = m_swerveSubsystem.getPose();
        AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        double x;
        double y;
        
        if (m_pose[m_point].rotation != -1000){
            x = MathUtil.clamp(m_swerveSubsystem.m_xPID.calculate(robotPose.getX(), Math.abs(targetPose.getX() - (isRedAlliance() ? field.getFieldLength() : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            y = MathUtil.clamp(m_swerveSubsystem.m_yPID.calculate(robotPose.getY(), Math.abs(targetPose.getY() - (isRedAlliance() ? field.getFieldWidth() : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            SmartDashboard.putNumber("x pose", Math.abs(targetPose.getX() - (isRedAlliance() ? field.getFieldLength() : 0)));
            SmartDashboard.putNumber("y pose", Math.abs(targetPose.getY() - (isRedAlliance() ? field.getFieldWidth() : 0)));
        }else{
            x = MathUtil.clamp(m_swerveSubsystem.m_xPID.calculate(robotPose.getX(), Math.abs(targetTranslation2d.getX() - (isRedAlliance() ? field.getFieldLength() : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            y = MathUtil.clamp(m_swerveSubsystem.m_yPID.calculate(robotPose.getY(), Math.abs(targetTranslation2d.getY() - (isRedAlliance() ? field.getFieldWidth() : 0))), -m_pose[m_point].m_maxVelocity, m_pose[m_point].m_maxVelocity);
            SmartDashboard.putNumber("x pose", Math.abs(targetTranslation2d.getX() - (isRedAlliance() ? field.getFieldLength() : 0)));
            SmartDashboard.putNumber("y pose", Math.abs(targetTranslation2d.getY() - (isRedAlliance() ? field.getFieldWidth() : 0)));
        }
        
        if (m_commandedRotation.isPresent()){
            m_swerveSubsystem.drive(new Translation2d(x, y), m_commandedRotation.get().getAsDouble() * (isRedAlliance() ? -1 : 1), true);
            return;
        }

        if (m_pose[m_point].m_translation == null){
            m_swerveSubsystem.drive(new Translation2d(x, y), orientPID(() -> Units.degreesToRadians(m_pose[m_point].m_pose.getRotation().getDegrees()) * (isRedAlliance() ? -1 : 1)).get().getDegrees(), true);
            System.out.println("We gettin velocity because of no rotation");
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
                m_swerveSubsystem.drive(new Translation2d(x, y), 0, true);
                
            }else{
                m_swerveSubsystem.drive(new Translation2d(x, y), orientPID(() -> m_pose[m_rotationIndex].m_pose.getRotation().getDegrees() * (isRedAlliance() ? -1 : 1)).get().getDegrees(), true);
                System.out.println("The rotation index is NOT -1");
            }
        }
        
        
        
    }


    public Supplier<Rotation2d> orientPID(DoubleSupplier targetRotation){
        double setPointDegrees = targetRotation.getAsDouble();
        double heading = m_swerveSubsystem.getPose().getRotation().getDegrees();

        double rotation = MathUtil.clamp(m_swerveSubsystem.m_orientPID.calculate(heading, setPointDegrees), -m_pose[m_point].m_maxAngularVelocity, m_pose[m_point].m_maxAngularVelocity);
        return () -> Rotation2d.fromDegrees(rotation);
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : true;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("maxPoints", m_pose.length - 1);
        
        Pose2d targetPose = m_pose[m_point].m_pose;
        Translation2d targetTranslation2d = m_pose[m_point].m_translation;
        AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        if (targetPose != null){
            
            targetPose = new Pose2d(Math.abs(targetPose.getX() - (isRedAlliance() ? field.getFieldLength() : 0)), Math.abs(targetPose.getY() - (isRedAlliance() ? field.getFieldWidth() : 0)), new Rotation2d(targetPose.getRotation().getRotations() * (isRedAlliance() ? -1 : 1)));
            if (m_swerveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < m_pose[m_point].m_deadzone){
                
                try {
                    if (Math.abs(targetPose.getRotation().getRadians() - m_swerveSubsystem.getPose().getRotation().getDegrees()) < alignDeadzone || Math.abs((targetPose.getRotation().getRadians() - 360) - m_swerveSubsystem.getPose().getRotation().getDegrees()) < alignDeadzone || Math.abs((targetPose.getRotation().getRadians() + 360) - m_swerveSubsystem.getPose().getRotation().getDegrees()) < alignDeadzone){
                        if (m_point >= m_pose.length - 1){
                            return true;
                        }else{
                            m_point ++;
                            m_rotationIndex = -1;
                            System.out.println("AAAAA Why are you increasing the point!!!!!");
                        }   
                    }else {
                        // System.out.println(m_pose[m_point].m_pose.getRotation().getRadians());
                        return false;
                    }
                } catch(Exception e){
                    return false;
                }
            }else{
                return false;
            }
        }else{
            targetTranslation2d = new Translation2d(Math.abs(targetTranslation2d.getX() - (isRedAlliance() ? field.getFieldLength() : 0)), Math.abs(targetTranslation2d.getY() - (isRedAlliance() ? field.getFieldWidth() : 0)));

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
                System.out.println(String.format("Returning %d, we're at %d", i, m_point));
                return i;
            }else if(m_pose[i].m_isImportant){
                System.out.println(String.format("Returning %d, we're at %d", i, m_point));
                return i;
            }
        }
        return -1;
        
    }
        
}
