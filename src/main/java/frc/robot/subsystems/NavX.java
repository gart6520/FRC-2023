// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  private AHRS gyro = new AHRS();
  public NavX() {}

  /** Angle is not continuous from -180 to 180 degree */
  public double getYaw()
  {
    return gyro.getYaw();
  }

  /** Get pitch */
  public double getPitch()
  {
    return gyro.getPitch();
  }

  /** Get roll */
  public double getRoll()
  {
    return gyro.getRoll();
  }

  /** Return Rotation2d object, angle is continuous */
  public Rotation2d getRotation2d()
  {
    return gyro.getRotation2d();
  }

  /** Reset gyro */
  public void reset()
  {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Pitch", getPitch());
    SmartDashboard.putNumber("Current Yaw", getYaw());
    SmartDashboard.putNumber("Current Roll", getRoll());
    SmartDashboard.putNumber("NavX temp", gyro.getTempC());
  }
}