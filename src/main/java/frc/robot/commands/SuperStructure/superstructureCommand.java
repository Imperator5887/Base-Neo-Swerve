package frc.robot.commands.SuperStructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.desiredPositions;
import frc.robot.subsystems.Superstructure.pivotSubsystem;


public class superstructureCommand extends CommandBase{

    private final pivotSubsystem pivot;
    
    

    private final double pivotPosition;
    private final double telescopicPosition;
    private final double intakeVelocity;
    private final double wristPosition;

    public superstructureCommand(desiredPositions desiredPositionsConstants){
        pivot = pivotSubsystem.getInstance();
      

        this.pivotPosition = desiredPositionsConstants.pivotingArmGoal;
        this.telescopicPosition = desiredPositionsConstants.telescopicArmGoal;
        this.wristPosition  = desiredPositionsConstants.wristGoal;
        this.intakeVelocity = desiredPositionsConstants.intakeVelocity;
        
        addRequirements(pivot);
        
    }

    @Override
    public void initialize() {     
    }

    @Override
    public void execute() {

        pivot.setDesiredPosition(pivotPosition);
        
        

    } 

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


    

