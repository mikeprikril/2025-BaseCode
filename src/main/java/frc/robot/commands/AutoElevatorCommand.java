// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoElevatorCommand extends Command {
  /** Creates a new AutoElevatorCommand. */
  public final ElevatorSubsystem elevator;
  public final CommandXboxController elevatorJoystick;

  public AutoElevatorCommand(ElevatorSubsystem m_elevator, CommandXboxController m_elevatorJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    elevatorJoystick = m_elevatorJoystick;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.GetElevatorEncoderPosition() < (Constants.ElevatorConstants.L4Height - Constants.ElevatorConstants.deadband)) { //is elevator below target
      elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed);
    }
    else if (elevator.GetElevatorEncoderPosition() > (Constants.ElevatorConstants.L4Height + Constants.ElevatorConstants.deadband)){ //is elevator above target
      elevator.AutoElevator(Constants.ElevatorConstants.AutoDownSpeed);
    }
    else {
      elevator.AutoElevator(Constants.ElevatorConstants.HoldElevatorSpeed); //if at target then hold
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !elevatorJoystick.getHID().getRawButton(Constants.ElevatorConstants.L4JoystickButton); //end command when operator lets go of button
  }
}
