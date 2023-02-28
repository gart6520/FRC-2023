// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;

public class Extender extends SubsystemBase {
  public TalonFX extender = new TalonFX(EXTEND);
  /** Creates a new Extender. */
  public Extender() {
    extender.setNeutralMode(NeutralMode.Brake);
  }

  public void extendV(double X) {
    extender.set(ControlMode.PercentOutput, X);
  }

  public void extendP(double x) {
    extender.set(ControlMode.Position, x);
  }
  @Override
  public void periodic() {
    ControlMode ctrlmode = extender.getControlMode();
    TalonFXSensorCollection sc = extender.getSensorCollection();

    if (ctrlmode == ControlMode.PercentOutput) {
      SmartDashboard.putString("Extender control mode", "PercentOutput");
    }

    else if (ctrlmode == ControlMode.Position) {
      SmartDashboard.putString("Extender control mode", "Position");
    }

    SmartDashboard.putNumber("Extender output percent", extender.getMotorOutputPercent());
    SmartDashboard.putNumber("Extender encoder velocity", sc.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Extender encoder position", sc.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Extender falcon temp", extender.getTemperature());
  }
}
