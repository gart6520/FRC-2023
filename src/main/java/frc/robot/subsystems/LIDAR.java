// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LIDAR extends SubsystemBase {
  /** Creates a new LIDAR. */

  // Constants
  private final static int LIDAR_ADDR = 0x62;
  private final static int LIDAR_CONFIG_REGISTER = 0x00;
  private final static int LIDAR_DISTANCE_REGISTER = 0x8f;

  // Variables
  private I2C lidar;
  private byte[] distance;

  public LIDAR() {
    // Initial I2C connection
    lidar = new I2C(Port.kOnboard, LIDAR_ADDR);
    distance = new byte[2];
  }

  private void update() {
    lidar.write(LIDAR_CONFIG_REGISTER, 0x04b);
    Timer.delay(0.05);
    lidar.read(LIDAR_DISTANCE_REGISTER, 2, distance);
    Timer.delay(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
    
    int d_cm = (int)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);
    SmartDashboard.putNumber("Lidar distance (m)", d_cm / 100.0);
  }
}
