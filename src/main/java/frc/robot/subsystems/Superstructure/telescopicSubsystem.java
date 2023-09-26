package frc.robot.subsystems.Superstructure;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.telescopicConstants;


public class telescopicSubsystem extends SubsystemBase{
    
    private static telescopicSubsystem instance;

    private final CANSparkMax motor;

    private final RelativeEncoder relativeEncoder;

    private final AbsoluteEncoder absoluteEncoder;
    
    private final ProfiledPIDController pidTelescopic;


    public telescopicSubsystem(){

        motor = new CANSparkMax(telescopicConstants.motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        
        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPositionConversionFactor(telescopicConstants.kTelescopicEncoder2Meters);
        relativeEncoder.setPosition(0);


        pidTelescopic = new ProfiledPIDController(
            telescopicConstants.kP_Telescopic, 
            telescopicConstants.kI_Telescopic, 
            telescopicConstants.kD_Telescopic, telescopicConstants.constraints);  
            

    }


    public static telescopicSubsystem getInstance(){
        if(instance == null){
            instance = new telescopicSubsystem();
        }

        return instance;
    }

    public double getPosition(){
        return absoluteEncoder.getPosition();
    }

    public void resetEncoders(){

        relativeEncoder.setPosition(0);
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Extending Position", relativeEncoder.getPosition());
    }

   

    public void setGoal(double extendingPosition){

        pidTelescopic.setGoal(extendingPosition);
        motor.set(pidTelescopic.calculate(getPosition()));
    }

    public void stopMotor(){

        motor.set(0);
    }

    public void setVelocity(double velociity){
        motor.set(velociity);
    }

    public boolean isInGolePosition(){

        boolean isInGoal;

        if(pidTelescopic.getSetpoint().velocity == 0){
            isInGoal = true;
        } else {
            isInGoal = false;
        }

        return isInGoal;
    } 

    
}
