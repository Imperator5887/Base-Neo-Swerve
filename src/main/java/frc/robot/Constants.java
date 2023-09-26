package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.desiredPositions;
import frc.lib.util.limelightOffsets;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeterss = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 7.13;
        public static final double kTurningMotorGearRatio = 1 /  13.71;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeterss;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.247;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

     

        public static final double kVelocidadTeoricaMaximaMPS = 6.5;
        public static final double kVelocidadGiroTeoricaRPS = 3 * 2 * Math.PI;

        public static final double kLimitadorDriveMPS = kVelocidadTeoricaMaximaMPS / 5;
        public static final double kLimitadorAngularRPS = //
                kVelocidadGiroTeoricaRPS / 5;
        public static final double kLimitadorAceleracionDrive = 3;
        public static final double kLimitadorAceleracionAngular = 3.5;
    }

    public static class telescopicConstants{
           
        public static int motorID = 12;
       
        public static int leftLimitSwitchID = 6;
        public static int rightLimitSwitchID = 7;
       
        public static int boreEncoderID = 8;

        public static double kP_Telescopic = 0;
        public static double kI_Telescopic = 0;
        public static double kD_Telescopic = 0;

        public static final double gearRatio = 0;
        public static final double tamboDiameter = Units.inchesToMeters(2);

        public static double kTelescopicEncoder2Meters = gearRatio * Math.PI * tamboDiameter;

        public static double maxVelocity = 5;
        public static double maxAcceleration = 5;

        public static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);



    }

    public static class wristConstants{
        public static int motorID = 15;

        public static int encoderID = 9;

        public static int limitSwitchID = 10;

        public static double kP;
        public static double kI;
        public static double kD;

        public static double maxVelocity = 5;
        public static double maxAcceleration = 5;

        public static final double gearRatio = 1 / 230;
        public static final double kWristEncoder2Rad = gearRatio * 2 * Math.PI;
        public static double kWristEncoder2Deg = Units.radiansToDegrees(kWristEncoder2Rad);



        
        public static TrapezoidProfile.Constraints constraints =
                      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    }

    public static class intakeConstants{

        public static int motorID = 14;

    }


    public static class MODS {
  


        public static final class frontLeftModule {
                  
            public static int driveMotorID = 1;
            public static int turningMotorID = 2;
            public static boolean driveMotorInverted = false;
            public static boolean turningMotorInverted = false;
            public static double absoluteEncoderOffsetRad = (2 * Math.PI) - 4.07698;
            public static boolean absoluteEncoderReversed = true;
                       
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
                  
         }
                  
        public static final class frontRightModule {
                  
            public static int driveMotorID = 3;
            public static int turningMotorID = 4;
            public static boolean driveMotorInverted = false;
            public static boolean turningMotorInverted = false;
            public static int absoluteEncoderID = 2;
            public static double absoluteEncoderOffsetRad = (2 * Math.PI) - 0.34;
            public static boolean absoluteEncoderReversed = true;
                      
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
                  
        public static final class rearLeftModule {
                  
            public static int driveMotorID = 5;
            public static int turningMotorID = 6;
            public static boolean driveMotorInverted = false;
            public static boolean turningMotorInverted = false;
            public static int absoluteEncoderID = 3;
            public static double absoluteEncoderOffsetRad = (2 * Math.PI) - 1.2432;
            public static boolean absoluteEncoderReversed = true;
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
                  
        public static final class rearRightModule {
                  
            public static int driveMotorID = 7;
            public static int turningMotorID = 8;
            public static boolean driveMotorInverted = false;
            public static boolean turningMotorInverted = false;
            public static int absoluteEncoderID =  4;
            public static double absoluteEncoderOffsetRad = (2 * Math.PI) - 5.6684;
            public static boolean absoluteEncoderReversed = true; 
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kVelocidadTeoricaMaximaMPS / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kVelocidadGiroTeoricaRPS / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        
        
       
    }

    public static final class limelightConstants {

        public static final class aprilTag{

            public static double driveOffset = 5.4;
            public static double strafeOffset = -1;
            public static double rotationOffset = 17;

            public static final limelightOffsets offsets =  
        new limelightOffsets(driveOffset, strafeOffset, rotationOffset);

        }

        public static final class reflectiveTape{

            public static double driveOffset = 0.08;
            public static double strafeOffset = -5.16;
            public static double rotationOffset = 17;

            public static final limelightOffsets offsets =  
        new limelightOffsets(driveOffset, strafeOffset, rotationOffset);

        }

        
    }

    public static final class goalConstants {

        public static final class upperNodeGoal {
            public static final double telescopicArmGoal = 1;
            public static final double pivotingArmGoal = 30;
            public static final double wristGoal = 270;
            public static final double intakeVelocity = 0.1;
            
            public static final desiredPositions constants = 
            new desiredPositions(telescopicArmGoal, pivotingArmGoal, wristGoal, intakeVelocity);
        }

        public static final class middleNodeGoal {
            public static final double telescopicArmGoal = .5;
            public static final double pivotingArmGoal = 30;
            public static final double wristGoal = 300;
            public static final double intakeVelocity = 0.1;
            
            public static final desiredPositions constants = 
            new desiredPositions(telescopicArmGoal, pivotingArmGoal, wristGoal, intakeVelocity);
        }

        public static final class lowerNodeGoal {
            public static final double telescopicArmGoal = 0.14;
            public static final double pivotingArmGoal = 10;
            public static final double wristGoal = 285;
            public static final double intakeVelocity = 0.1;
            
            public static final desiredPositions constants = 
            new desiredPositions(telescopicArmGoal, pivotingArmGoal, wristGoal, intakeVelocity);
        }

        public static final class floorGoal {
            public static final double telescopicArmGoal = 0.23;
            public static final double pivotingArmGoal = 5;
            public static final double wristGoal = 0;
            public static final double intakeVelocity = -0.1;
            
            public static final desiredPositions constants = 
            new desiredPositions(telescopicArmGoal, pivotingArmGoal, wristGoal, intakeVelocity);
        }

        public static final class homePosition {
            public static final double telescopicArmGoal = 0.02;
            public static final double pivotingArmGoal = 0;
            public static final double wristGoal = 0.5;
            public static final double intakeVelocity = 0;
            
            public static final desiredPositions constants = 
            new desiredPositions(telescopicArmGoal, pivotingArmGoal, wristGoal, intakeVelocity);
        }


    }

    public static final class pivotingConstants{

        public static final int buttonDesiredPosition1 = 2;
        public static final int buttonHomePosition = 1;


        public static final int motor1ID = 9;
        public static final int motor2ID = 10;
        public static final int motor3ID = 11;

        public static final double kP_Pivot = 4;
        public static final double kI_Pivot = 0;
        public static final double kD_Pivot = 0;

        public static final double offsetEncoder = 4.86;

        public static final double desiredPosition = Math.PI;
        public static final double homePosition = 0.02;
        public static final double placePosition = 0.1;
        public static final double floorPosition = 0.23;
        public static final double maxPosition = 0.5;

        public static final double encoderSprocketConversionFactor = 1 /11.55;
        public static final double encoderRadiansCF = encoderSprocketConversionFactor ;

        public static final double motorSprocketConversionFactor = 1 / 288.75;
        public static final double motorRadiansCF = motorSprocketConversionFactor ;
        

        

        public static final double maxVelocity = 25;
        public static final double maxAcceleration = 18;
        public static final TrapezoidProfile.Constraints constraints = 
                        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
                    

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kPlacerControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class Shuffleboard {

        public static final ShuffleboardTab kShuffleboardTab = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Imperator");
    
        }

}