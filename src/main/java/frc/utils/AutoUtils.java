package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.Swerve.subsistemaSwerve;
import frc.utils.Shuffleboard.Autoselector;

public class AutoUtils {
    protected static Autoselector autoselector = new Autoselector();
    protected static subsistemaSwerve swerve = subsistemaSwerve.getInstance();
    protected static LimeLightObject limelight = LimeLightObject.getInstance();

    public static Command getAuto(){
        return Autoselector.getAutoSelected();
    }
}
