package frc.robot.PathPlanningThing;

import java.io.File;
import java.nio.file.Paths;

import org.wpilib.tunable.Selectable;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.telemetry.Telemetry;

public class SelectAuto {
    
    public static Selectable<Command> m_autoSelection = new Selectable<Command>();
    // private static String m_previousAutoSelection = m_autoSelection.getSelected();

    /**
     * This is the auto that has been selected through Smart Dashboard.
     * Make sure that you call checkSelectedAuto() in Disabled Periodic. 
     */
    public static Command selectedAuto = new DrivePath();
    @SuppressWarnings("unused")
    private static final File autoDirectory = Paths.get("./src/main/java/frc/robot/PathPlanningThing/Autos/").toAbsolutePath().toFile();


    // function in disabled periodic that checks if the auto has been changed in order to set selected auto to a new drive path


    /**
     * This function needs to be called in disabled periodic in the Robot Java Class
     * 
     * @param swerveSubsystem make sure that your swerve subsystem in robot container is static so you can pass it into this function
     */
    // public static void checkSelectedAuto(SwerveSubsystem swerveSubsystem){
    //     if (m_previousAutoSelection != m_autoSelection.getSelected()){
    //         selectedAuto = new DrivePath(m_autoSelection.getSelected(), swerveSubsystem);
    //     }

    //     m_previousAutoSelection = m_autoSelection.getSelected();
    // }

        /**
     * If you want to test autos in simulation then you should call this in autonomousInit() in Robot.
     * If you don't then you'll have to change the selected auto in disabled so that it parses it before running the auto.
     * 
     * @param swerveSubsystem make sure that your swerve subsystem in robot container is static so you can pass it into this function
     */
    // public static void forceSelectedAuto(SwerveSubsystem swerveSubsystem){
    //     selectedAuto = new DrivePath(m_autoSelection.getSelected(), swerveSubsystem);
        
    //     m_previousAutoSelection = m_autoSelection.getSelected();
    // }


    //  SmartDashboard.putData(Commands.runOnce(() -> {
    //     m_isOn = !m_isOn;
    //   })
    //   .withName("Toggle Lights")
    //   .ignoringDisable(true));

    //   SmartDashboard.putData(Commands.runOnce(() -> {
    //     m_isScrolling = !m_isScrolling;
    //     LedPatterns.m_disabledPattern = LEDPattern.steps(Map.of(0, Color.RED, 0.16, Color.BLUE, 0.33, new Color(1.0f, 0.35f, 0.0f),0.49, Color.RED, 0.66, Color.BLUE, 0.83, new Color(1.0f, 0.35f, 0.0f))).scrollAtAbsoluteVelocity(m_isScrolling ? LightConstants.k_disabledVelocity : MetersPerSecond.of(0), LightConstants.k_ledSpacing);
    //   })
    //   .withName("Toggle Disable Scrolling")
    //   .ignoringDisable(true));
    /**
     * This is a Javadoc Comment that I will write later :P
     * @param auto_name
     * @param command
     */
    public static void addCommand(String auto_name, Command command){
        m_autoSelection.add(auto_name, command);
    }

    public static Command getSelectedAuto(){
        try {
            return m_autoSelection.getSelected();
        } catch (Exception ignored) {
            return Commands.runOnce(() -> {System.out.println("No selected Auto");});
        }
        
        // return Commands.sequence(Subsystems.m_intakeExtension.extendIntake(), Commands.parallel(
        //     Subsystems.m_shooterSubsystem.staticShoot(),
        //                     Subsystems.m_indexerConveyor.testRun(),
        //         Subsystems.m_kickerSubsystem.testRun(),
        //         Subsystems.m_secondaryKicker.testRun(),
        //     Commands.repeatingSequence(Commands.either(Commands.parallel(

        //         Subsystems.m_indexerRollerSubsystem.testRun(true)
        //     ), Commands.none(), Subsystems.m_shooterSubsystem::m_isAtSpeed))
        // ));
    }

    /**
     * Call this function in the Robot Container constructer!
     * Make sure that it is before configure Bindings
     */

    // public static void addAutos() {
        
    //     for (final File fileEntry : autoDirectory.listFiles()) {
    //         if (fileEntry.isDirectory()){
    //             addAutos(fileEntry);
    //         } else {
    //             m_autoSelection.addOption(fileEntry.getName(), fileEntry.getName());
    //         }
    //     }

    //     SmartDashboard.putData("Auto Selection", m_autoSelection);
    // }
//     private static void addAutos(final File folder) {
        
//         for (final File fileEntry : folder.listFiles()) {
//             if (fileEntry.isDirectory()){
//                 addAutos(fileEntry);
//             } else {
//                 m_autoSelection.addOption(fileEntry.getName(), fileEntry.getName());
//             }
//         }
//     }
}
