package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SubsystemInstance.*;

public class Grab extends CommandBase {
  /** Creates a new AutoBalance. */
  private double speed;
  public Grab(double s) {
    speed = s;
    addRequirements(m_Grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Grabber.grabV(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Grabber.grabV(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
