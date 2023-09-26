package frc.robot.commands.SuperStructure;

import frc.robot.subsystems.Superstructure.solenoidSubsystem;

import javax.swing.plaf.metal.MetalIconFactory.FileIcon16;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class solenoidCommand extends CommandBase {

    private final solenoidSubsystem solenoide;
    private final boolean disableLock;
  /**
   * Creates a new solenoidCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public solenoidCommand(boolean disableLock) {
    
    this.solenoide = solenoidSubsystem.getInstance();

    this.disableLock = disableLock;

    addRequirements(solenoide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    solenoide.disableLock(disableLock);
    
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
