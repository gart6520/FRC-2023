// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.NavX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Controller ensitivity */
  public static class Sensitivity {
    /** Joystick sensitivity */
    public static final double JOYSTICK_SENSE = 0.15;
  }

  /** Motor coefficients */
  public static class MotorCoefficients {
    /** Normal speed coefficient */
    public static double NORMAL_SPEED = 0.4;
    /** Boosted speed coefficient */
    public static double BOOST_SPEED = 0.8;
  }

  /** PID coefficients */
  public static class PID {
    /** Integral coefficient */
    public static final double KI = 0.4;
    /** Propotional coefficient */
    public static final double KP = 0.3;
    /** Derivative coefficient */
    public static final double KD = 0.05;
    /** Max positional error */
    public static final double KTOLERANCE = 1.5;
    /** Max velocity error */
    public static final double KTOLERANCEVELOCITY = 0.8;
  }

  /** CAN devices ID */
  public static class CAN_ID {
    /** Left Front Motor */
    public static final int LEFT_FRONT = 1;
    /** Left Back Motor */
    public static final int LEFT_BACK = 2;
    /** Right Front Motor */
    public static final int RIGHT_FRONT = 3;
    /** Right Back Motor */
    public static final int RIGHT_BACK = 4;
    public static final int EXTEND = 6;
    public static final int TURRET = 5;
  }

  /** Kinematics Meters */
  public static class KinematicsMeters {
    /** Left Front Wheel */
    public static Translation2d LEFT_FRONT_CENTER = new Translation2d();
    /** Left Back Wheel */
    public static Translation2d LEFT_BACK_CENTER = new Translation2d();
    /** Right Front Wheel */
    public static Translation2d RIGHT_FRONT_CENTER = new Translation2d();
    /** Right Back Wheel */
    public static Translation2d RIGHT_BACK_CENTER = new Translation2d();
  }

  /**
   * Constraints for PID controllers
   */
  public static class Constraints {
    public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
  }

  /** Controller button mapping */
  public static class Controller {
    // Joysticks
    
    /** First Joystick */
    public static final Joystick JOYSTICK0 = new Joystick(0);
    /** Second Joystick */
    //public static Joystick JOYSTICK1 = new Joystick(1);

    // Axises

    /** X axis of the left joystick */
    public static final int XAXISLEFT = 0;
    /** Y axis of the left joystick */
    public static final int YAXISLEFT = 1;
    /** X axis of the right joystick */
    public static final int XAXISRIGHT = 5;
    /** Y axis of the right joystick */
    public static final int YAXISRIGHT = 2;

    // Buttons
    /** Boost speed button */
    public static final int BOOST = 4;
    /** Auto drive to scoring zone button */
    public static final int DRIVE_TO_SCORE = 1;
    /** Auto drive to double substation */
    public static final int DRIVE_TO_SUBSTATION = 2;
    public static final int ExtendOut = 3;
    public static final int RetreatB = 1;
    public static final int LiftB = 4;
    public static final int LowerB = 2;
    
  }

  /** Vision configs */
  public static class VisionConfigs {
    /**
     * Limelight camera
     * The camera name must be the same as the name of the camera in the PhotonVision GUI
     */
    public static PhotonCamera limelight = new PhotonCamera("limelight");

    /**
     * Second camera (faced robot's back) plugged into Limelight's USB port
     * The camera name must be the same as the name of the camera in the PhotonVision GUI
     */
    public static PhotonCamera backcam = new PhotonCamera("backcam");

    /**
     * Field layout file path
     * The field layout file is from WPILib
     */
    public static final String fieldLayoutFile = AprilTagFields.k2023ChargedUp.m_resourceFile;

    /**
     * ROBOT_TO_LIMELIGHT 3D vector
     * This 3D vector stores the location of the robot in relative to the Limelight camera
     * We haven't completed the real robot to test yet, so we put the example configuration
     * here from PhotonVision library:
     * 
     * Cam mounted facing forward, half a meter forward of center, half a meter up from center
     */
    public static final Transform3d ROBOT_TO_LIMELIGHT = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    /**
     * ROBOT_TO_BACKCAM 3D vector
     * This 3D vector stores the location of the robot in relative to the Limelight camera
     * We haven't completed the real robot to test yet, so we put the example configuration
     * here from PhotonVision library, but with modifications:
     * 
     * Cam mounted facing backward, half a meter backward of center, half a meter up from center
     */
    public static final Transform3d ROBOT_TO_BACKCAM = new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, 0, 180));

    /**
     * TAG_TO_GOAL 3D vector
     * This 3D vector stores the location of the destination point the robot *will* go to in
     * relative to the apriltag's location. We haven't completed the robot yet, so we are not
     * sure how long exactly the robot hand will be, so we will just put a random value here
     * But there are something to note (in this value):
     * 
     * - The robot dest point is to be 1m backward to the apriltag
     * - Robot will face up with the apriltag, so yaw angle should be 180 degree
     * - Robot will be -0.6 meter compared to the apriltag (refer to manual)
     *   It doesn't matter, because we will convert this to Pose2d and ignore the Z :)
     */
    public static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(-1, 0.0, -0.6), new Rotation3d(0, 0, 180));
  }

  /**
   * Subsystem instances
   * It's surely stupid to do this, but Roborio doesn't have that much RAM, so we need to
   * reduce the memory usage by creating only one instance of a subsystem. Also we want
   * unification of subsystem instances :)  
   * 
   * Idea credit: KhiemGOM :)
   */

  
  public final static class Function{
    /**Return whether the value is bigger than noise value (sensitivity) **/
    public static boolean notNoise (double val)
    {
      return Math.abs(val) > 0.05;
    }
    
    public static double signof(double val)
    {
        if (val>0)
        {
          return 1;
        }
        if (val<0)
        {
          return -1;
        }
        return 0;
    }
    public static double signedSqr(double val)
    {
      return signof(val) * val * val;
    }
  }
  public final static class SingleInstance
  {
    public static NavX GYRO = new NavX();
    public static Drivebase DRIVER_BASE = new Drivebase();
  }
}
