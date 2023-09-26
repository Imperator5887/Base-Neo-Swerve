package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.limelight.reflectiveTapeAlign;

public class solenoidSubsystem extends SubsystemBase {

  private final Solenoid solenoide;

  private static solenoidSubsystem instance;

    public solenoidSubsystem() {

      solenoide = new Solenoid(PneumaticsModuleType.REVPH, 15);
    
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Solenoide", solenoide.get());

  }

  public static solenoidSubsystem getInstance(){
    if(instance == null){
      instance = new solenoidSubsystem();
    }
    return instance;
  }

  public void disableLock(boolean openLock){

    solenoide.set(openLock);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
