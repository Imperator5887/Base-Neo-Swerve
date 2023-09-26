package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.goalConstants.floorGoal;
import frc.robot.Constants.goalConstants.homePosition;
import frc.robot.Constants.goalConstants.upperNodeGoal;
import frc.robot.commands.swerveDriveComando;
import frc.robot.commands.SuperStructure.intakeCommand;
import frc.robot.commands.SuperStructure.pivotCommand;
import frc.robot.commands.SuperStructure.telescopicCommand;
import frc.robot.commands.autos.autos;
import frc.robot.commands.limelight.aprilTagAlign;
import frc.robot.commands.limelight.reflectiveTapeAlign;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.Superstructure.telescopicSubsystem;
import frc.robot.subsystems.Swerve.subsistemaSwerve;

public class RobotContainer {

    //private final subsistemaSwerve swerveSubsystem = new subsistemaSwerve();
    private subsistemaSwerve swerve;
    private LimeLightObject limelight;
    private telescopicSubsystem telescopic;

    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    
  

    public RobotContainer() {

        swerve = subsistemaSwerve.getInstance();
        limelight  = LimeLightObject.getInstance();
        telescopic  = telescopicSubsystem.getInstance();

        swerve.setDefaultCommand(new swerveDriveComando(
                swerve,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                true, limelight
                ));

        

              //limelight.setDefaultCommand(new limelighCommand(swerveSubsystem, limelight, false));

        configureButtonBindings();
    }

    
    private void configureButtonBindings() {

        //APRIL TAG:
       new JoystickButton(driverJoytick, 5).whileTrue(new aprilTagAlign(limelight, true));

       //REFLECTIVE TAPE:
        //new JoystickButton(driverJoytick, 4).whileTrue(new reflectiveTapeAlign(swerveSubsystem, limelight, false));

    /*new JoystickButton(placerJoystick, 1).whileTrue(new pivotCommand(homePosition.constants));
        new JoystickButton(placerJoystick, 2).whileTrue(new pivotCommand(upperNodeGoal.constants));
        new JoystickButton(placerJoystick, 3).whileTrue(new pivotCommand(floorGoal.constants));
*/

       //UPPER NODE
       /*new JoystickButton(placerJoystick, 1).whileTrue(
        Commands.parallel(
            new pivotCommand(upperNodeGoal.constants), 
            new telescopicCommand(upperNodeGoal.constants))
       );*/


       /*new JoystickButton(placerJoystick, 5).whileTrue(new intakeCommand(.15));
       new JoystickButton(placerJoystick, 5).whileFalse(new intakeCommand(0.4));

       new JoystickButton(placerJoystick, 6).whileTrue(new intakeCommand(-.15));
       new JoystickButton(placerJoystick, 6).whileFalse(new intakeCommand(-0.4));
    
    */}

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
       return autos.autoForward();

        // 5. Add some init and wrap-up, and return everything
    }
}
