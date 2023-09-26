package frc.robot.commands.SuperStructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure.endEfector.intakeSubsystem;

public class intakeCommand extends CommandBase {

    private final intakeSubsystem intake;
   // private final boolean pickingCone;
   private final double velocity;
  /**  
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public intakeCommand(double velocity) {

    this.velocity = velocity;
    this.intake = intakeSubsystem.getInstance();
    
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.setVelocity(velocity);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
