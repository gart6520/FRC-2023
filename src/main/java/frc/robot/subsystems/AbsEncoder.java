// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsEncoder extends SubsystemBase {
  /** Creates a new AbsEncoder. */
  private AnalogEncoder absEncoder;

  public AbsEncoder(int channel) {
    absEncoder = new AnalogEncoder(channel);
  }

  /** Reset absolute encoder */
  public void reset() {
    absEncoder.reset();
  }

  /**
   * Get absolute encoder value
   * @return current angle in radian
  */
  public double get() {
    return absEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute encoder value (radian)", get());
  }
}
