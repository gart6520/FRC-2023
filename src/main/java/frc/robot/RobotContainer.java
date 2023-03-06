// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.Controller.*;

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
  // Subsystem instances
  private final Turret m_Turret = new Turret();
  private final Extender m_Extender = new Extender();
  private final Grabber m_Grabber = new Grabber();
  // Buttons
  private final JoystickButton driveToScoreButton = new JoystickButton(JOYSTICK1, DRIVE_TO_SCORE);
  private final JoystickButton extend1Button = new JoystickButton(JOYSTICK1, ExtendOut);
  private final JoystickButton LiftButton = new JoystickButton(JOYSTICK0, LiftB);
  private final JoystickButton LowerButton = new JoystickButton(JOYSTICK0, LowerB);
  private final JoystickButton GrabButton = new JoystickButton(JOYSTICK1, GrabB);
  private final JoystickButton ReleaseButton = new JoystickButton(JOYSTICK1, RealeaseB);
  // Commands
  // private Command endAll = new InstantCommand(()->
  // {
  // CommandScheduler.getInstance().cancelAll();
  // ((Drivebase)null).periodic();
  // });
  private Command extendP1 = new StartEndCommand(() -> {
    //m_Extender.extendP(-2000);
  },
      () -> {
        ;
      }, m_Extender);
  private Command extendP2 = new StartEndCommand(() -> {
    //m_Extender.extendP(-20000);
  },
      () -> {
        //m_Extender.extendV(-0);
        ;
      }, m_Extender);
  private Command extendP3 = new StartEndCommand(() -> {
    //m_Extender.extendP(-50000);
  },
      () -> {
        //m_Extender.extendV(0);
        ;
      }, m_Extender);
  private Command extendP4 = new StartEndCommand(() -> {
    //m_Extender.extendP(-90000);
  },
      () -> {
        ;
      }, m_Extender);
  private Command extendV1 = new StartEndCommand(() -> {
        m_Extender.extendV(-0.2);
      },
          () -> {
            m_Extender.extendV(0);
          }, m_Extender);
  private Command extendV2 = new StartEndCommand(() -> {
        m_Extender.extendV(0.2);
      },
          () -> {
            m_Extender.extendV(0);
            
          }, m_Extender);

  private Command Lift = new StartEndCommand(() -> {
    m_Turret.rotate(0.6);
  },
      () -> {
        m_Turret.rotate(0);
      }, m_Turret);

  private Command Lower = new StartEndCommand(() -> {
    m_Turret.rotate(-0.6);
  },
      () -> {
        m_Turret.rotate(0);
      }, m_Turret);
  private Command Grab = new StartEndCommand(() -> {
    m_Grabber.grabV(1);
  },
      () -> {
        m_Grabber.grabV(0);
      }, m_Grabber);
  private Command Release = new StartEndCommand(() -> {
    m_Grabber.grabV(-1);
  },
      () -> {
        m_Grabber.grabV(0);
      }, m_Grabber);
  /*
   * The AprilTag ID in the Blue's substation is 4, while the AprilTag ID in the
   * Red's is 5
   * Get the current team color (set in DriverStation), then decide the correct
   * AprilTag ID to aim
   */

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
    new JoystickButton(JOYSTICK1, LEFT).whileTrue(extendP1);
    new JoystickButton(JOYSTICK1, DOWN).whileTrue(extendP2);
    new JoystickButton(JOYSTICK1, RIGHT).whileTrue(extendP3);
    new JoystickButton(JOYSTICK1, UP).whileTrue(extendP4);
    new JoystickButton(JOYSTICK1, 5).whileTrue(extendV1);
    new JoystickButton(JOYSTICK1, 6).whileTrue(extendV2);
    LiftButton.whileTrue(Lift);
    LowerButton.whileTrue(Lower);
    GrabButton.whileTrue(Grab);
    ReleaseButton.whileTrue(Release);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Lift;

    // new PIDCommand(
    // new PIDController(0.1,0,0),
    // m_Drivebase.test::getSelectedSensorVelocity,
    // 4,
    // m_Drivebase::rotate,
    // m_Drivebase
    // );
  }
}
