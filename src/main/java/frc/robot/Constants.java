// Copyright 2021-2025 FRC 6328
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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static boolean disableHAL = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int HOOD_ID = 0;
  public static final double HOOD_SUPPLY_CURRENT_LIMIT = 15;
  public static final double HOOD_STATOR_CURRENT_LIMIT = 30;

  public static final int FLYWHEELLEFT_ID = 0;
  public static final int FLYWHEELRIGHT_ID = 0;
  public static final double FLYWHEELS_CIRCUMFERENCE = 0;
  public static final double FLYWHEELS_SUPPLY_CURRENT_LIMIT = 30;
  public static final double FLYWHEELS_STATOR_CURRENT_LIMIT = 60;

  public static final int INTAKE_ID = 0;
  public static final double INTAKE_GEAR_RATIO = 0;
  public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 20;
  public static final double INTAKE_STATOR_CURRENT_LIMIT = 45;

  public static final int EXTENSION_ID = 0;
  public static final double EXTENSION_GEAR_RATIO = 0;
  public static final double EXTENSION_SUPPLY_CURRENT_LIMIT = 20;
  public static final double EXTENSION_STATOR_CURRENT_LIMIT = 45;
  public static final int EXTENSION_CANCODER_ID = 0;

  public static final int INDEXER_MOTOR_ID = 67; // Please look over these idk what the limits should be - Sam🔥
  public static final int INDEXER_STATOR_CURRENT_LIMIT = 20;
  public static final int INDEXER_SUPPLY_CURRENT_LIMIT = 25;
}
