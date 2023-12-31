/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */


package frc.utils;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerveSusbsystem;

public class TrajectoryReader {
    private static swerveSusbsystem swerve = swerveSusbsystem.getInstance();

    public static Command readTrajectory(PathPlannerTrajectory trajectory, Boolean isFirstPath ){
        
        return new SequentialCommandGroup(
            new InstantCommand(() ->{
                if(isFirstPath){
                    swerve.resetOdometry(trajectory.getInitialPose());
                }
            }),

            new PPSwerveControllerCommand(
                trajectory, 
                swerve::getPose, 
                swerve.getKinematics(), 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), 
                swerve::setModuleStates, 
                swerve)
        );
    }
}
