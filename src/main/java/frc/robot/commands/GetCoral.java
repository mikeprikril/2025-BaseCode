// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetCoral extends Command {

  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  public final CommandXboxController operatorJoystick;
  private final Timer timer;

  public GetCoral(ElevatorSubsystem m_elevator, ArmSubsytem m_arm, CommandXboxController m_operatorJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;
    operatorJoystick = m_operatorJoystick;
    timer = new Timer();
    
    addRequirements(elevator);
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
    if (elevator.GetElevatorEncoderPosition() > Constants.ElevatorConstants.troughHeight){
      arm.GripperIntake();
      elevator.AutoElevator(Constants.ElevatorConstants.TransferDownSpeed);
    }
    else{
      arm.StopGripper();
      elevator.StopElevator();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !operatorJoystick.getHID().getRawButton(Constants.ElevatorConstants.TransferButton);
    //end command when operator lets go of button
  }
}
