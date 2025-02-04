// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(-5, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(18.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;

    public static final int DriverUSBPort = 0;
    public static final int OperatorUSBPort = 2;
  }
  

  public static class ElevatorConstants
  {
    public static final int leftMotorCANID = 10;
    public static final int rightMotorCANID = 11;

    public static final int elevatorBottomLimitSwitchIO = 1;
    public static final int elevatorTopLimitSwitchIO = 2;

    public static final int AlmostDownValue = 10; //slow down when close to bottom
    public static final int AlmostUpValue = 58; //slow down when close to top
    public static final int UpLimit = 62; //top limit for elevator
    public static final double SlowDown = 0.2; //slow down to 20% if close to limit

    public static final double deadband = 3;
    public static final double TransferHeight = 20;
    public static final double troughHeight = 18;
    public static final double L1Height = 30;
    public static final double L2Height = 40;
    public static final double L3Height = 45;
    public static final double L4Height = 50;
    public static final double AutoUpSpeed = .4;
    public static final double AutoDownSpeed = -.3;
    public static final double TransferDownSpeed = -0.1;
    public static final double HoldElevatorSpeed = 0;

    public static final int TransferButton = 1;
    public static final int BumpDownTestButton = 2;
    public static final int ReadyTestButton = 3;
    public static final double ResetArmDelay = 0.5;
    public static final int L4JoystickButton = 4;

    public static final double JoystickDeadband = 0.05;
    public static final double goSlow = .65; //slow down elevator joystick input
  }

  public static class ArmConstants
  {
    public static final int armMotorCANID = 12;
    public static final int GripperCANID = 21;

    public static final int armBottomLimitSwitchIO = 3;
    public static final int armTopLimitSwitchIO = 4;

    public static final int AlmostUpValue = 10; //slow down when close to top
    public static final int AlmostDownValue = 55; //slow down when close to tray at bottom
    public static final int ArmAtTray = 60; //value of encoder when arm is vertically down at tray
    public static final double SlowDown = 0.2; //slow down by 20% if close to limit

    public static final double ArmDownSpeed = -.4;
    public static final double ArmUpSpeed = .5;

    public static final double GripperInSpeed = 1;
    public static final double GripperOutSpeed = -1;

    public static final int gripperInButton = 5;
    public static final int gripperOutButton = 6;

    public static final double goSlow = .5; //slow down arm joystick input
  }


}
