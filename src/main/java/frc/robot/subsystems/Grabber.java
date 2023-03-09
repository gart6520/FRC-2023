// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private WPI_TalonSRX Grabber = new WPI_TalonSRX(GRABBER); 
  public Grabber() {}

  public void grabV(double x) {
    Grabber.set(ControlMode.PercentOutput,x);
  }
  public void grabP(double x) {
    Grabber.set(ControlMode.Position,x);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

