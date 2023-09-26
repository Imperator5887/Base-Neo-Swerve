package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightObject extends SubsystemBase{

    private static LimeLightObject instance;
    private  boolean objetivoVisto = false;
    private  double x;
    private static double y;
    private  double a;
    private static double v;
    private static double s;
    private  double yaw;
    private boolean pipeline;
    private int pipelineNumber;



    @Override
    public void periodic(){

        
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

       table.getEntry("pipeline").setNumber(pipelineNumber);

        double[] robotPose = table.getEntry("botpose").getDoubleArray(new double[6]);


        NetworkTableEntry tx = table.getEntry("tx");
    
        NetworkTableEntry ty = table.getEntry("ty");
    
        NetworkTableEntry ta = table.getEntry("ta");
    
        NetworkTableEntry tv = table.getEntry("tv");

        NetworkTableEntry ts = table.getEntry("ts");
    
        double xMeters = robotPose[0];

        double yMeters = robotPose[1];

        double zMeters = robotPose[2];

        double roll = robotPose[3];

        double pitch = robotPose[4];

        yaw = -robotPose[5];



        

         x = -tx.getDouble(0.0);
    
         y = ty.getDouble(0.0);
    
         a = ta.getDouble(0.0);
    
         v = tv.getDouble(0.0);

         s = ts.getDouble(0.0);
        if(v > 0){
            objetivoVisto = true;
          } else{
            objetivoVisto = false;
          } 


    SmartDashboard.putNumber("X Meters", xMeters);
    SmartDashboard.putNumber("y Meters", yMeters);
    SmartDashboard.putNumber("z Meters", zMeters);
    SmartDashboard.putNumber("Roll", roll);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Yaw", yaw);







    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("S", s);
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("A", a);
    SmartDashboard.putBoolean("objetivo?", objetivoVisto);
        
    }
    
    public void alingToAprilTag(boolean alingToAprilTag){
        if(!alingToAprilTag){
            pipelineNumber = 1;      
        } else {
            pipelineNumber = 0;
        }
    }

    public double getXLimelight(){        
        return x;
    }

    public double getYLimelight(){
        return y;
    }

    public  double getALimelight(){
        return a;
    }

    public double getVLimelight(){
        return v;
    }

    public  double getYaw(){
        return yaw;
    }

    public  boolean getObjetivoVisto(){
        return objetivoVisto;
    }

    public static LimeLightObject getInstance(){
        if(instance == null){
            instance = new LimeLightObject();
        }
        return instance;
    }
   
}
