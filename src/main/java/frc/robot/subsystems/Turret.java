// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;

public class Turret extends SubsystemBase {
  public CANSparkMax turret = new CANSparkMax(TURRET, MotorType.kBrushless);

  /** Creates a new Turret. */
  public Turret() {}

  public void rotate(double x) {
    turret.setVoltage(x);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret velocity", turret.get());
    SmartDashboard.putNumber("Talon Turret Temp", turret.getMotorTemperature());
  }
}
