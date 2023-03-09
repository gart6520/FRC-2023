// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.Crash;
//import frc.robot.commands.GotoLocation;

import static frc.robot.Constants.SubsystemInstance.*;
import static frc.robot.Constants.Controller.*;
import static frc.robot.Constants.VisionConfigs.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Buttons
  private final JoystickButton driveToScoreButton = new JoystickButton(JOYSTICK0, DRIVE_TO_SCORE);
  private final JoystickButton driveToSubstationButton = new JoystickButton(JOYSTICK0, DRIVE_TO_SUBSTATION);

  private final JoystickButton GrabButton = new JoystickButton(JOYSTICK0, GrabB);
  private final JoystickButton ReleaseButton = new JoystickButton(JOYSTICK0, RealeaseB);
  
  // Commands
  private final Crash m_Crash = new Crash();
  //private final GotoAprilTag driveToScore = new GotoAprilTag(-1); // Choose best apriltag
  //private final GotoLocation driveToScore = new GotoLocation(doubleSubstationLocation1, Rotation2d.fromDegrees(180));
  //private final GotoLocation driveToSubstation = new GotoLocation(doubleSubstationLocation1, Rotation2d.fromDegrees(180));

  private Command extendP1 = new StartEndCommand(() -> {
    m_Extender.extendP(-2000);
  }, () -> {
    ;}
  , m_Extender);

  private Command extendP2 = new StartEndCommand(() -> {
    m_Extender.extendP(-20000);
  }, () -> {
    //m_Extender.extendV(-0);
    ;
  }, m_Extender);

  private Command extendP3 = new StartEndCommand(() -> {
    m_Extender.extendP(-50000);
  }, () -> {
    //m_Extender.extendV(0);
    ;
  }, m_Extender);

  private Command extendP4 = new StartEndCommand(() -> {
    m_Extender.extendP(-90000);
  }, () -> {
    ;
  }, m_Extender);

  private Command extendV1 = new StartEndCommand(() -> {
    m_Extender.extendV(-0.2);
  }, () -> {
    m_Extender.extendV(0);
  }, m_Extender);

  private Command extendV2 = new StartEndCommand(() -> {
    m_Extender.extendV(0.2);
  }, () -> {
    m_Extender.extendV(0);
  }, m_Extender);

  private Command Grab = new StartEndCommand(() -> {
    m_Grabber.grabV(0.5);
  }, () -> {
    m_Grabber.grabV(0);
  }, m_Grabber);

  private Command Release = new StartEndCommand(() -> {
    m_Grabber.grabV(-0.5);
  }, () -> {
    m_Grabber.grabV(0);
  }, m_Grabber);

  private Command ResetGyro = new StartEndCommand(() -> {
    m_Gyro.reset();
  }, () -> {
    ;
  }, m_Grabber);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Extender
    new JoystickButton(JOYSTICK1, LEFT).whileTrue(extendP1);
    new JoystickButton(JOYSTICK1, DOWN).whileTrue(extendP4);
    new JoystickButton(JOYSTICK1, RIGHT).whileTrue(extendP3);
    new JoystickButton(JOYSTICK1, UP).whileTrue(extendP2);
    new JoystickButton(JOYSTICK1, L1).whileTrue(extendV1);
    new JoystickButton(JOYSTICK1, R1).whileTrue(extendV2);

    // Grabber
    GrabButton.whileTrue(Grab);
    ReleaseButton.whileTrue(Release);

    // Vision semi-auto
    //driveToScoreButton.whileTrue(driveToScore);
    //driveToSubstationButton.whileTrue(driveToSubstation);

    // Crash button
    new JoystickButton(JOYSTICK0, 7).and(new JoystickButton(JOYSTICK0, 8)).onTrue(m_Crash);
    new JoystickButton(JOYSTICK1, 9).and(new JoystickButton(JOYSTICK1, 10)).onTrue(m_Crash);

    // Gyro reset
    new JoystickButton(JOYSTICK0, 8).onTrue(ResetGyro);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
