// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.NavX;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  NavX navX;
  Drivebase Drivebase;

  public AutoBalance(Drivebase __Drivebase, NavX __navX) {
    navX = __navX;
    addRequirements(navX);
    Drivebase = __Drivebase;
    addRequirements(Drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = navX.getPitch() * 0.05;
    Drivebase.driveWithField(speed, 0, 0, navX.getRotation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivebase.driveWithField(0, 0, 0, navX.getRotation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}