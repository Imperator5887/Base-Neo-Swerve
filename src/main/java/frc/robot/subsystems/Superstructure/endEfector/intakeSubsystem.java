package frc.robot.subsystems.Superstructure.endEfector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class intakeSubsystem extends SubsystemBase{

    private static intakeSubsystem instance;

    private final CANSparkMax motor;


    public intakeSubsystem(){

        motor = new CANSparkMax(intakeConstants.motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();    
    

    }

    public static intakeSubsystem getInstance(){
        if(instance == null){
            instance = new intakeSubsystem();
        }
        return instance;
    }

    public void isPickingCone(boolean isPickingCone){

        if(isPickingCone){
            motor.set(0.15);
        } else {
            motor.set(-0.15);
        }

    }


    public void setVelocity(double velocity){
        motor.set(velocity);
    }

}
