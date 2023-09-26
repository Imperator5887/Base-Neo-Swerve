package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.Swerve.subsistemaSwerve;

public class swerveDriveComando extends CommandBase {

    private final subsistemaSwerve swerveSubsystem;
    private final Supplier<Double> avanceFuncion, strafeFuncion, giroFuncion;
    private final Supplier<Boolean> isOrientadoACancha;
    private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
    private final  Boolean joystickDrive;
    private final PIDController xPID, yPID, giroPID;
    private final LimeLightObject LimeLightObject;

    public swerveDriveComando(subsistemaSwerve subsistemaSwerve,
            Supplier<Double> velAvanceFuncion, Supplier<Double> velStrafeFuncion, Supplier<Double> velGiroFuncion,
            Supplier<Boolean> fieldOrientedFunction, Boolean joystickDrive, LimeLightObject limelight
            ) {
        this.swerveSubsystem = subsistemaSwerve;
        this.avanceFuncion = velAvanceFuncion; 
        this.strafeFuncion = velStrafeFuncion;
        this.giroFuncion = velGiroFuncion;
        this.isOrientadoACancha = fieldOrientedFunction;
        this.joystickDrive = joystickDrive;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kLimitadorAceleracionDrive);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kLimitadorAceleracionDrive);
        this.giroLimiter = new SlewRateLimiter(DriveConstants.kLimitadorAceleracionAngular);
        this.LimeLightObject = limelight;
        this.xPID = new PIDController(0.08, 0, 0);
        this.yPID = new PIDController(0.04, 0, 0);
        this.giroPID = new PIDController(0.01, 0, 0);
        addRequirements(subsistemaSwerve);
    }

    @Override
    public void initialize() {
    
        
    }

    @Override
    public void execute() {

        LimeLightObject.periodic();
        // 1. Get real-time joystick inputs
 if(joystickDrive == true){
        double velAvance;
        double velStrafe;
        double velGiro; 


    velAvance = avanceFuncion.get();
    velStrafe = strafeFuncion.get();
    velGiro = giroFuncion.get();  

    

        // 2. Apply deadband
        velAvance = Math.abs(velAvance) > OIConstants.kDeadband ? velAvance : 0.0;
        velStrafe = Math.abs(velStrafe) > OIConstants.kDeadband ? velStrafe : 0.0;
        velGiro = Math.abs(velGiro) > OIConstants.kDeadband ? velGiro : 0.0;

         // 3. Make the driving smoother
        velAvance = xLimiter.calculate(velAvance) * 4;
        velStrafe = yLimiter.calculate(velStrafe) * 4;
        velGiro = giroLimiter.calculate(velGiro)
                * 6;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (isOrientadoACancha.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    velAvance, velStrafe, velGiro, swerveSubsystem.getRotation2d());
        } else {
             //Relative to robot
            chassisSpeeds = new ChassisSpeeds(velAvance, velStrafe, velGiro);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    } else {
        return;
    }
    } 

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.pararModulos();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
