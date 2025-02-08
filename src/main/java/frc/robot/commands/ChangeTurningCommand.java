// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
public class ChangeTurningCommand extends Command {
  /** Creates a new ChangeTurningCommand. */
  private final SwerveSubsystem swerve;

  public ChangeTurningCommand(SwerveSubsystem m_swerve, CommandXboxController commandXboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = m_swerve;

    addRequirements(swerve);
  }
  boolean mode = Constants.DrivebaseConstants.turningMode;
  public final SwerveSubsystem drivebase = RobotContainer.drivebase;

  final CommandXboxController driverXbox = RobotContainer.driverXbox;
  final CommandXboxController operatorXbox = RobotContainer.operatorXbox;

 // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
 //       () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.DEADBAND),
 //       () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
  //      () -> -driverXbox.getRightX(),
 //       () -> -driverXbox.getRightY());

 //   Command standardDrive = drivebase.driveCommand(
 //       () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.DEADBAND),
 //       () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
 //       () -> MathUtil.applyDeadband(Constants.DrivebaseConstants.SlowDownTurn*-driverXbox.getRightX(), OperatorConstants.DEADBAND));

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //code HERREEE REMEMBERRRRR
    //if (mode=true) {
    //  drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    //  mode = false;
  //} else {
  //  drivebase.setDefaultCommand(standardDrive);
  //  mode = true;
  };
 
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
