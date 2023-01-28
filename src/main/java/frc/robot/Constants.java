// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class TestArmPositions {
    public static final double rotateOne = -4;
    public static final double rotateTwo = 0;
    public static final double rotateThree = 4;
  }

  public static class ArmPositions {
    public static final double rotateOne = -4;
    public static final double rotateTwo = 0;
    public static final double rotateThree = 4;
    public static final double extendOne = 0;
    public static final double extendTwo = 4;
    public static final double extendThree = 8;
  }

  public static class PneumaticsChannels {
    public static final int highGear = 0;
    public static final int lowGear = 1;
    public static final int clawClose = 2;
    public static final int clawOpen = 3;
  }

  public static class CANBusIDs {
    public static class ArmCANBusIDs {
      public static final int leftRotatorID = 4;
      public static final int rightRotatorID = 5;
      public static final int rightExtenderID = 6;
      public static final int leftExtenderID = 7;
    }

    public static class DriveCANBusIDs {
      public static final int leftMotorID = 2;
      public static final int rightMotorID = 3;
    }
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
  }
}
