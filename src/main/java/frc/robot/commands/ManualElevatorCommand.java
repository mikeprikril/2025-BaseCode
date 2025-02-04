// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevatorCommand extends Command {
  /** Creates a new ManualElevatorCommand. */
  public final ElevatorSubsystem elevator;
  public final CommandXboxController elevatorJoystick;

  public ManualElevatorCommand(ElevatorSubsystem m_elevator, CommandXboxController m_elevatorController) {
  elevator = m_elevator;
  elevatorJoystick = m_elevatorController;

  addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.ElevatorJoystickControl(elevatorJoystick.getLeftY()); //Left y-axis controls elevator
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
