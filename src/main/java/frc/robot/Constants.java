// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  /** Set to true to log Joystick data. Set to false otherwise. */
  public static final boolean LOG_JOYSTICK_DATA = true;

  /** Set to true to send telemetry data to Live Window. Set to false to disable it. */
  public static final boolean LW_TELEMETRY_ENABLE = false;

  // Set a Global Constant to either Show or Hide extended logging data for each of the subsystems
  // Set to true to show extended logging data.
  // Set to false to hide extended logging data.

  /** Set to true to show extended logging data for the arm subsystem. */
  public static final boolean SD_SHOW_ARM_EXTENDED_LOGGING_DATA = true;

  /** Set to true to enable loop timing logging. */
  public static final boolean LOOP_TIMING_LOG = false;

  /** Set to true to enable using Tunable Numbers. */
  public static final boolean TUNING_MODE = true;

  /** Set to true to log each frame of command execution. Set to false to disable. */
  public static final boolean COMMAND_EXECUTE_LOG = false;

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    /** USB port ID for the controller. */
    public static final int CONTROLLER_PORT = 0;
  }

  /** Constants used for the Arm subsystem. */
  public static final class ArmConstants {

    private ArmConstants() {
      throw new IllegalStateException("ArmConstants Utility Class");
    }

    public static final int MOTOR_PORT = 1;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;

    // Constants tunable through TunableNumbers
    public static final double ARM_KP = 6.0;
    public static final double ARM_KS = 0.0;
    public static final double ARM_KG = 0.1;
    public static final double ARM_KV_VOLTS_PER_RAD_PER_SEC = 2.3;
    public static final double ARM_MAX_VELOCITY_DEG_PER_SEC = 180.0;
    public static final double ARM_MAX_ACCELERATION_DEG_PER_SEC2 = 540.0;

    public static final double GEAR_RATIO = 100;
    public static final double ARM_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI / GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = ARM_RAD_PER_ENCODER_ROTATION / 60;

    // Arm positions.  Horizontal = 0 radians. Assume arm starts at lowest (rest) position
    public static final double ARM_RETRACTED_RADS = Units.degreesToRadians(90.0);
    public static final double ARM_EXTENDED_RADS = Units.degreesToRadians(10.0);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(0.0);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(180.0);

    // Offset of the relative encoder which is equal to the angle in the start position
    public static final double ARM_OFFSET_RADS = MIN_ANGLE_RADS;

    public static final double POS_INCREMENT = Units.degreesToRadians(1.0); // For small adjustments
    public static final double POSITION_TOLERANCE = Units.degreesToRadians(4.0);
    public static final double VELOCITY_TOLERANCE = Units.degreesToRadians(10.0);
    public static final double ABSOLUTE_OFFSET_DEGREES = 222.6;
  }
}
