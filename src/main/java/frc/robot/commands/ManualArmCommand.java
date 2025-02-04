// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualArmCommand extends Command {
  /** Creates a new ManualElevatorCommand. */
  public final ArmSubsytem arm;
  public final CommandXboxController armJoystick;

  public ManualArmCommand(ArmSubsytem m_arm, CommandXboxController m_armController) {
  arm = m_arm;
  armJoystick = m_armController;

  addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.ArmJoystickControl(armJoystick.getRightY()); //right y-axis controls arm

  if (armJoystick.getHID().getLeftBumperButton() == true){ //buttons to controll gripper
      arm.GripperIntake();
    }
    else if (armJoystick.getHID().getRightBumperButton() == true){
      arm.GripperSpitOut();
    }
    else arm.StopGripper();

    
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
