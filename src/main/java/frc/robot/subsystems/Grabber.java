// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;

public class Grabber extends SubsystemBase {
  TalonFX grabber = new TalonFX(GRABBER);

  /** Creates a new Grabber. */
  public Grabber() {
    grabber.setNeutralMode(NeutralMode.Brake);
  }

  public void grab(double X) {
    grabber.set(TalonFXControlMode.PercentOutput, X);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Grabber output percent", grabber.getMotorOutputPercent());
    SmartDashboard.putNumber("Grabber falcon temp", grabber.getTemperature());
  }
}
