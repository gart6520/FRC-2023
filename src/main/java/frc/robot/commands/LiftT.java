package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SubsystemInstance.*;

public class LiftT extends CommandBase {
  /** Creates a new AutoBalance. */

  public LiftT() {
    addRequirements(m_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Turret.rotate(-0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Turret.rotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
