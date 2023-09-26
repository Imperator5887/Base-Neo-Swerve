package frc.robot.subsystems.Superstructure.endEfector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.wristConstants;

public class wristSubsystem extends SubsystemBase{
    
    private static wristSubsystem instance;

    private final CANSparkMax motor;
    
    private final RelativeEncoder encoderSpark;

    private final AbsoluteEncoder absoluteEncoder;

    private final ProfiledPIDController profiledPIDController;

    public wristSubsystem(){

        motor = new CANSparkMax(wristConstants.motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        encoderSpark = motor.getEncoder();
        encoderSpark.setPositionConversionFactor(wristConstants.kWristEncoder2Deg);
        resetEncoders();

        absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

        profiledPIDController = new ProfiledPIDController(
            0.2, 0, 0, 
            wristConstants.constraints);
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Wrist Positiion", getPosition());

    }

    public static wristSubsystem getInstance(){
        if(instance == null){
            instance = new wristSubsystem();
        }
        return instance;
    }
   
    public double getPosition(){

        return absoluteEncoder.getPosition();

    }


    public void resetEncoders(){
        encoderSpark.setPosition(getPosition());
    }

    public void setWristPosition(double setpoint){
        profiledPIDController.setGoal(setpoint);
        motor.set(profiledPIDController.calculate(absoluteEncoder.getPosition()));
    }



}
