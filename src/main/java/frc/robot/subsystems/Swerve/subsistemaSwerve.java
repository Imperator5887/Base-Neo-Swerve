package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MODS.frontLeftModule;
import frc.robot.Constants.MODS.frontRightModule;
import frc.robot.Constants.MODS.rearLeftModule;
import frc.robot.Constants.MODS.rearRightModule;

public class subsistemaSwerve extends SubsystemBase {

    

    private final Field2d field = new Field2d();

    private final SwerveModule frontLeft = new SwerveModule(frontLeftModule.constantes, 1);

    private final SwerveModule adelanteDer = new SwerveModule(frontRightModule.constantes, 2);

    private final SwerveModule atrasIzq = new SwerveModule(rearLeftModule.constantes, 3);

    private final SwerveModule atrasDer = new SwerveModule(rearRightModule.constantes, 4);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private static subsistemaSwerve instance;

    final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
         DriveConstants.kDriveKinematics, 
         getRotation2d(), 
         new SwerveModulePosition[]{
            frontLeft.getPosition(),
            adelanteDer.getPosition(),
            atrasIzq.getPosition(),
            atrasIzq.getPosition()
         });
    
         

    public subsistemaSwerve() {
        odometry.resetPosition(
            getRotation2d(), 
            new SwerveModulePosition[]{
               frontLeft.getPosition(),
               adelanteDer.getPosition(),
               atrasIzq.getPosition(),
               atrasIzq.getPosition()
            }, new Pose2d());
    
        
       
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

    }

    

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetOdometria(Pose2d pose){
        resetEncoders();
        odometry.resetPosition(getRotation2d(), 
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            adelanteDer.getPosition(),
            atrasIzq.getPosition(),
            atrasIzq.getPosition()
         }, pose);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        this.setModuleStates(moduleStates);
    }
   

    public SwerveDriveKinematics getKinematics(){
        return DriveConstants.kDriveKinematics;
    }
    
     public static subsistemaSwerve getInstance(){
        if(instance == null){
            instance = new subsistemaSwerve();
        }
        return instance;
    }
     public Command cargarTrajectoriaPP(PathPlannerTrajectory trajectoria, boolean isFirstPath){
        
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(isFirstPath){
                    this.resetOdometria(trajectoria.getInitialPose());
                }
            }), 
            new PPSwerveControllerCommand(
            trajectoria, 
            this::getPose, 
            getKinematics(), 
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0), 
            this::setModuleStates, 
            this)
        );
    }
    

    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("Potencia 1", frontLeft.getPotenciaAvance());
        SmartDashboard.putNumber("Potencia 2", adelanteDer.getPotenciaAvance());
        SmartDashboard.putNumber("Potencia 3", atrasIzq.getPotenciaAvance());
        SmartDashboard.putNumber("Potencia 4", atrasDer.getPotenciaAvance());

        SmartDashboard.putNumber("Giro Robot", getHeading());


        odometry.update(getRotation2d(), 
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            adelanteDer.getPosition(),
            atrasIzq.getPosition(),
            atrasIzq.getPosition()
         });
 
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putData("Field", field);

        System.out.println(odometry.getPoseMeters());
         
}
    

    public void pararModulos() {
        frontLeft.parar();
        adelanteDer.parar();
        atrasIzq.parar();
        atrasDer.parar();
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        adelanteDer.resetEncoders();
        atrasIzq.resetEncoders();
        atrasDer.resetEncoders();

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kVelocidadTeoricaMaximaMPS);
        frontLeft.setDesiredState(desiredStates[0]);
        adelanteDer.setDesiredState(desiredStates[1]);
        atrasIzq.setDesiredState(desiredStates[2]);
        atrasDer.setDesiredState(desiredStates[3]);
    }
}
