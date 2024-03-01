// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a csonvenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class OIConstants{
    public static final double kDriveDeadband = 0.10;
    public static final Joystick kdriveJoyButton = new Joystick(0);

    public static final CommandXboxController m_driverController = new CommandXboxController(0);
    public static final Joystick kauxController = new Joystick(1);

  }

  public static class VisionConstants {
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(0, 0, 0, new Rotation3d());
    public static final Transform3d ROBOT_TO_REAR_CAMERA = new Transform3d(0, 0, 0, new Rotation3d());
    public static final double POSE_CONFIDENCE_FILTER_THRESHOLD = 0.2;
    public static final double VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD = 0.5;
  }
  
}
