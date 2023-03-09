// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;
import static frc.robot.Constants.Controller.*;

public class Turret extends SubsystemBase {
  public CANSparkMax turret = new CANSparkMax(TURRET, MotorType.kBrushless);

  /** Creates a new Turret. */
  public Turret() {}

  public void rotate(double x) {
    turret.set(x);
  }
  @Override
  public void periodic() {
    if (JOYSTICK1.getRawAxis(TurretAxis) > 0.1 || JOYSTICK1.getRawAxis(TurretAxis) < -0.1) {
      rotate(JOYSTICK1.getRawAxis(TurretAxis) * (JOYSTICK1.getRawAxis(3) > 0.1 ? 0.4 : 0.2));
    }

    // Software stall or brake
    if (JOYSTICK1.getRawAxis(TurretAxis) <= 0.1 && JOYSTICK1.getRawAxis(TurretAxis) >= -0.1) {
      rotate(JOYSTICK1.getRawAxis(4) > 0.1 ? 0.1 : 0);
    }

    SmartDashboard.putNumber("Turret velocity", turret.get());
    SmartDashboard.putNumber("Spark Max Turret Temp", turret.getMotorTemperature());
  }
}
