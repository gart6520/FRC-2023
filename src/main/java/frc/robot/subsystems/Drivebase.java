// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_ID.*;
import static frc.robot.Constants.Function.*;

public class Drivebase extends SubsystemBase {
  
  //private PathPlannerTrajectory traj = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(LEFT_FRONT);
  private WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(LEFT_BACK);
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RIGHT_FRONT);
  private WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(RIGHT_BACK);
  private MecanumDrive mecanum;
   
  //private ProfiledPIDController m_PIDController = new ProfiledPIDController(rP,rI,rD, new TrapezoidProfile.Constraints(rMaxSpeed, rMaxAccel));

  public Drivebase() {
    // leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //Brake mode
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);

    //Disable safety
    // leftFrontMotor.setSafetyEnabled(false);
    // rightFrontMotor.setSafetyEnabled(false);
    // leftBackMotor.setSafetyEnabled(false);
    // rightBackMotor.setSafetyEnabled(false);

    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);
    mecanum = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
  }

  /**
   * Drive method for Mecanum platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is
   * independent of its angle or rotation rate.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param gyroAngle The gyro heading around the Z axis. Use this to implement field-oriented
   *     controls.
   */
  public void driveWithField(double x, double y, double rotation, Rotation2d gyroAngle)
  {
    if (notNoise(x) || notNoise(y) || notNoise(rotation))
    {
      mecanum.driveCartesian(x, y, rotation, gyroAngle);
    }
  }

  /**
   * Drive method for Mecanum platform.
   *
   * <p>Angles are measured counterclockwise from the positive X axis. The robot's speed is
   * independent of its angle or rotation rate.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   */

  public void drive(double x, double y, double rotation) {
    if (notNoise(x) || notNoise(y) || notNoise(rotation))
    {
      mecanum.driveCartesian(x, y, rotation);
    }
  }

  /**
   * Drive method for Mecanum platform.
   * <p>1 is left front
   * <p>2 is left back
   * <p>3 is right front
   * <p>4 is right back
   * @param motorNumber the motor number
   * @param speed the speed of the motor
   */

  @Override
  public void periodic() {
    // Put velocity
    SmartDashboard.putNumber("Motor Left Front", leftFrontMotor.get());
    SmartDashboard.putNumber("Motor Left Back", leftBackMotor.get());
    SmartDashboard.putNumber("Motor Right Front", rightFrontMotor.get());
    SmartDashboard.putNumber("Motor Right Back", rightBackMotor.get());

    // Put encoder measured velocity
    SmartDashboard.putNumber("Encoder Left Front", leftFrontMotor.getSelectedSensorVelocity());

    // Put talon temp
    SmartDashboard.putNumber("Talon Left Front Temp", leftFrontMotor.getTemperature());
    SmartDashboard.putNumber("Talon Left Back Temp", leftBackMotor.getTemperature());
    SmartDashboard.putNumber("Talon Right Front Temp", rightFrontMotor.getTemperature());
    SmartDashboard.putNumber("Talon Right Back Temp", rightBackMotor.getTemperature());
  }
}