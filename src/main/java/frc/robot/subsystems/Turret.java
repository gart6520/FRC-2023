// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;

public class Turret extends SubsystemBase {
  public WPI_TalonSRX turret = new WPI_TalonSRX(TURRET);
  /** Creates a new Turret. */
  public Turret() {
    turret.setNeutralMode(NeutralMode.Brake);
  }

  public void rotate(double x) {
    turret.set(x);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret velocity", turret.get());
    SmartDashboard.putNumber("Talon Turret Temp", turret.getTemperature());
  }
}
