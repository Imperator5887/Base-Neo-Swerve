package frc.robot.commands.SuperStructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.desiredPositions;
import frc.robot.subsystems.Superstructure.telescopicSubsystem;

public class telescopicCommand extends CommandBase {

    private final telescopicSubsystem arm;
    //private final double desiredGoal;
    private final Supplier<Double> velocity;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public telescopicCommand(Supplier<Double> velocity) {

    arm = telescopicSubsystem.getInstance();

    this.velocity = velocity;
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    arm.resetEncoders();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.setVelocity(velocity.get() * 0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isInGolePosition();
  }
}
