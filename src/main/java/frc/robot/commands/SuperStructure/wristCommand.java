package frc.robot.commands.SuperStructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.desiredPositions;
import frc.robot.subsystems.Superstructure.endEfector.wristSubsystem;

public class wristCommand extends CommandBase {

    private static wristSubsystem wrist;
    private final double desiredPosition;

  /**
   * Creates a new wristCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public wristCommand(desiredPositions desiredPositions) {

    this.wrist = wristSubsystem.getInstance();
    this.desiredPosition = desiredPositions.wristGoal;
    
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    wrist.setWristPosition(desiredPosition);
    
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
