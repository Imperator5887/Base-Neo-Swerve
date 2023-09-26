package frc.robot.commands.limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.Swerve.subsistemaSwerve;

public class aprilTagAlign extends CommandBase {

   private final subsistemaSwerve swerve;
    private final LimeLightObject limelight;
    private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
    private final PIDController drivePID, strafePID, giroPID;
    private final boolean alingToAprilTag;

    public aprilTagAlign(LimeLightObject limelight, boolean alingToAprilTag){

        this.swerve = subsistemaSwerve.getInstance();
        this.limelight = limelight;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kLimitadorAceleracionDrive);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kLimitadorAceleracionDrive);
        this.giroLimiter = new SlewRateLimiter(DriveConstants.kLimitadorAceleracionAngular);

        this.drivePID = new PIDController(0.08, 0, 0);
        this.strafePID = new PIDController(0.04, 0, 0);
        this.giroPID = new PIDController(0.01, 0, 0);

        this.alingToAprilTag = alingToAprilTag;

        addRequirements(swerve);
        
     }

     @Override
     public void initialize() {
     CameraServer.startAutomaticCapture();
         
     }
 
     @Override
     public void execute() {
        limelight.periodic();
        
        limelight.getVLimelight();
        limelight.getALimelight();
        limelight.getXLimelight();
        limelight.getVLimelight();

        limelight.alingToAprilTag(alingToAprilTag);
        
        double velAvance = 0;
        double velStrafe = 0;
        double velGiro = 0;
 
        if(limelight.getObjetivoVisto()){

            velAvance = drivePID.calculate(limelight.getALimelight(), 5.4);
            velStrafe = strafePID.calculate(limelight.getXLimelight(), -1);
            velGiro = giroPID.calculate(limelight.getYaw(), 17); 
        } else if(limelight.getObjetivoVisto() == false){
            velAvance = 0;
            velStrafe = 0;
            velGiro = 0.4;  
        } else {
            velAvance = 0;
            velStrafe = 0;
            velGiro = 0; 
        }
 
         // 2. Apply deadband
          velAvance = Math.abs(velAvance) > OIConstants.kDeadband ? velAvance : 0.0;
         velStrafe = Math.abs(velStrafe) > OIConstants.kDeadband ? velStrafe : 0.0;
         velGiro = Math.abs(velGiro) > OIConstants.kDeadband ? velGiro : 0.0;
 
          // 3. Make the driving smoother
         velAvance = xLimiter.calculate(velAvance) * 3;
         velStrafe = yLimiter.calculate(velStrafe) * 3;
         velGiro = giroLimiter.calculate(velGiro) * 5;
 
         // 4. Construct desired chassis speeds
         ChassisSpeeds chassisSpeeds;
         
              //Relative to robot
             chassisSpeeds = new ChassisSpeeds(velAvance, velStrafe, velGiro);

 
         // 5. Convert chassis speeds to individual module states
         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
         swerve.setModuleStates(moduleStates);
         // 6. Output each module states to wheels
         //swerve.setModuleStates(moduleStates);
        } 
     
 
     @Override
     public void end(boolean interrupted) {
         swerve.pararModulos();
     }
 
     @Override
     public boolean isFinished() {
         return false;
     }
    
    }  
