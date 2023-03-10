// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SubsystemInstance.*;
import static frc.robot.Constants.Controller.*;

public class TurretControl extends CommandBase {
  /** Creates a new TurrertControl. */
  public TurretControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (JOYSTICK1.getRawAxis(TurretAxis) > 0.1 || JOYSTICK1.getRawAxis(TurretAxis) < -0.1) {
      m_Turret.rotate(JOYSTICK1.getRawAxis(TurretAxis) * (JOYSTICK1.getRawAxis(3) > 0.1 ? 0.4 : 0.2));
    }

    // Software stall or brake
    if (JOYSTICK1.getRawAxis(TurretAxis) <= 0.1 && JOYSTICK1.getRawAxis(TurretAxis) >= -0.1) {
      m_Turret.rotate(JOYSTICK1.getRawAxis(4) > 0.1 ? 0.1 : 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
