//https://github.com/CyanideRS/2024STAGS// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
  //Drive Team Controller Constants
  public static final CommandXboxController DriverController =
            new CommandXboxController(DriveteamConstants.kDriverControllerPort);
  public static final CommandXboxController OperatorController =
            new CommandXboxController(DriveteamConstants.kOperatorControllerPort);
  //Robot Constants for calculations
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static class OperatorConstants {
    //Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class DriveteamConstants {
    //Drive Team Controller USB Port Assignments
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class GroundIntakeConstants {
    //Intake Motor CAN IDs
    public static final int IntakeDeployMotorID = 9;
    public static final int IntakeRollerMotorID = 10;
  }

  public static class FeederConstants {
    //Feeder Motor CAN IDs
    public static final int FeederRollerMotorID = 11;
  }

  public static class ShooterConstants{
    //Shooter Motor CAN IDs
    public static final int ShooterAngleMotorID = 12;
    public static final int ShooterTopMotorID = 13;
    public static final int ShooterBottomMotorID = 14;
  }

  public static class ClimberConstants{
    //Climber Motor CAN IDs
    public static final int ClimberLeftMotorID = 15;
    public static final int ClimberRightMotorID = 16;
  }

  public static final class AutonConstants {
    //Constants for Autonomous Motor Control
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0, 0, 0);
  }

  public static final class DrivebaseConstants {
    //Time in seconds to engage motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10;
  }
}