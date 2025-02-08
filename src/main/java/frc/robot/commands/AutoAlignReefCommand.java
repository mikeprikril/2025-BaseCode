// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class AutoAlignReefCommand extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final CommandXboxController driverController;
  //private final XboxController operatorController;
  private final Timer timer;
  private final ChassisSpeeds autoDriveSpeeds;

  public AutoAlignReefCommand(SwerveSubsystem m_swerveDrive, CommandXboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
   swerveDrive = m_swerveDrive;
    driverController = m_driverController;
    //operatorController = m_operatorController;
    timer = new Timer();

    autoDriveSpeeds = new ChassisSpeeds(0, 0, 0);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    autoDriveSpeeds.vxMetersPerSecond = Constants.DrivebaseConstants.TeleopAutoForwardSpeed;
    autoDriveSpeeds.vyMetersPerSecond = -Constants.DrivebaseConstants.NoteKP*swerveDrive.TrackNote(); //multiply Limelight value by P factor
    swerveDrive.drive(autoDriveSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverController.getHID().getRawButton(2); //go back to regular driving after letting go of button #2

  }
}
