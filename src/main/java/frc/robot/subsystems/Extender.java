// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;

public class Extender extends SubsystemBase {
  public TalonFX extender = new TalonFX(EXTEND);
  /** Creates a new Extender. */
  public Extender() {}

  public void extendV(double X) {
    extender.set(ControlMode.PercentOutput, X);
  }

  public void extendP(double x) {
    extender.set(ControlMode.Position, x);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
