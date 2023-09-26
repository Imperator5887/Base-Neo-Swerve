package frc.robot.commands.SuperStructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.desiredPositions;
import frc.robot.subsystems.Superstructure.pivotSubsystem;

public class pivotCommand extends CommandBase {

    private final pivotSubsystem arm;
    
    private final double desiredPosition;
    


  /**
   * Creates a new pivotCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public pivotCommand(desiredPositions desiredPositions) {

    this.desiredPosition = desiredPositions.pivotingArmGoal;
    arm = pivotSubsystem.getInstance();

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    arm.setDesiredPosition(desiredPosition);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isInGolePosition();
  }
}
