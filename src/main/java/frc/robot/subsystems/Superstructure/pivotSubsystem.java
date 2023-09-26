package frc.robot.subsystems.Superstructure;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Functions;
import frc.robot.Constants.pivotingConstants; 

public class pivotSubsystem extends SubsystemBase {
  /** Creates a new pivotArm. */

  private final CANSparkMax motor1;
  private final CANSparkMax motor2;
  private final CANSparkMax motor3;

  private final MotorControllerGroup motors;

  private final AbsoluteEncoder encoder;
  private final RelativeEncoder relativeEncoderr;

  private final ProfiledPIDController pidController;
  private final SparkMaxAbsoluteEncoder.Type encodeerabstype;

  private final DigitalInput limitSwitchUp;
  private final DigitalInput limitSwitchDown;


  private static pivotSubsystem instance;

  public pivotSubsystem() {


    motor1 = new CANSparkMax(pivotingConstants.motor1ID, MotorType.kBrushless);
    motor2 = new CANSparkMax(pivotingConstants.motor2ID, MotorType.kBrushless);
    motor3 = new CANSparkMax(pivotingConstants.motor3ID, MotorType.kBrushless);

    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();
    motor3.restoreFactoryDefaults();

    motor2.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor3.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    encodeerabstype = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
    encoder = motor1.getAbsoluteEncoder(encodeerabstype);
    encoder.setPositionConversionFactor(pivotingConstants.encoderSprocketConversionFactor);
    encoder.setZeroOffset(pivotingConstants.offsetEncoder);

    
    motors = new MotorControllerGroup(motor1, motor2, motor3);

    limitSwitchUp = new DigitalInput(1);
    limitSwitchDown = new DigitalInput(2);


    relativeEncoderr =motor2.getEncoder();
    relativeEncoderr.setPositionConversionFactor(pivotingConstants.motorSprocketConversionFactor);
    relativeEncoderr.setPosition(0);
 

    pidController = new ProfiledPIDController(
        pivotingConstants.kP_Pivot, 
        pivotingConstants.kI_Pivot, 
        pivotingConstants.kD_Pivot, 
        pivotingConstants.constraints)
        ;

    pidController.enableContinuousInput(-Math.PI, Math.PI);

    

    }


    public static pivotSubsystem getInstance(){
        if(instance == null){
            instance = new pivotSubsystem();
        }
        return instance;
      }
    
      public void stopMotors(){
        motors.stopMotor();
      }
    
      public double getPosition(){
        return encoder.getPosition();
      }

      public double getAppliedOutput(){
        return motor1.getAppliedOutput();
      }
    
      public boolean getLimitSwitchUpState(){
        return limitSwitchUp.get();
      }

      public boolean getLimiitSwitchDownState(){
        return limitSwitchDown.get();
      }

    
      public void setDesiredPosition(double goal){
        
        double pidOutput = pidController.calculate(relativeEncoderr.getPosition());
        while(goal < pivotingConstants.maxPosition){

          pidController.setGoal(goal);  

        }
       if(getLimiitSwitchDownState()){
        Functions.clamp(pidOutput, -0.75, 0);
       } else if(getLimitSwitchUpState()){
        Functions.clamp(pidOutput, 0, 0.75);
       }
       
        motors.set(pidOutput); 
      }

      public boolean isInGolePosition(){

        boolean isHome;
        if(pidController.getSetpoint().velocity == 0.0 ){
            isHome = true;
        } else {
            isHome = false;
        }

        return isHome;
      }
      
      @Override
      public void periodic() {
        
        SmartDashboard.putNumber("Pivot Position", getPosition());
        SmartDashboard.putNumber("Motor 1 Pivot Output", getAppliedOutput());
        SmartDashboard.putNumber("PID velocity", pidController.getSetpoint().velocity);
        SmartDashboard.putBoolean("Command Is Finiished?", isInGolePosition());
    
        }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }

}

