// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Controller.*;
import static frc.robot.Constants.SubsystemInstance.*;

public class DriveJoystick extends CommandBase {
  /** Creates a new ManualControl. */
  public DriveJoystick() {
    addRequirements(m_Drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = -JOYSTICK0.getRawAxis(YAXISLEFT); // Y joystick is inverted
    double xSpeed = JOYSTICK0.getRawAxis(XAXISLEFT); // X drive is inverted
    double rSpeed = JOYSTICK0.getRawAxis(XAXISRIGHT);

    SmartDashboard.putNumber("xSpeed", xSpeed * xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed * ySpeed);

    double co = 0.6;

    if (JOYSTICK0.getRawAxis(BOOST) > 0.2) co = 0.8;
    if (JOYSTICK0.getRawAxis(SLOW) > 0.2) co = 0.4;

    m_Drivebase.driveWithField(ySpeed * co, xSpeed * co, rSpeed * co, m_Gyro.getRotation2d().unaryMinus()); // X and Y is swapped in Controller vs Robot axis
    //m_Drivebase.drive(ySpeed * co, xSpeed * co, rSpeed * co);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivebase.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}