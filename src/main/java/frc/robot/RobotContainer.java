// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmReady;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.BumpDown;
import frc.robot.commands.ChangeTurningCommand;
import frc.robot.commands.GetCoral;
import frc.robot.commands.AutoAlignReefCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.TransferPosition;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  
  // Joysticks
  public final static CommandXboxController driverXbox = new CommandXboxController(Constants.OperatorConstants.DriverUSBPort);
  public final static CommandXboxController operatorXbox = new CommandXboxController(Constants.OperatorConstants.OperatorUSBPort);

  // The robot's subsystems and commands are defined here...

  //Subsystems
  public final static SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsytem arm = new ArmSubsytem();

  // Commands
  private final ManualElevatorCommand manualElevator;
  private final ManualArmCommand manualArm;
  private final AutoElevatorCommand autoElevator;
  private final AutoAlignReefCommand AlignReef;
  private final ChangeTurningCommand changeTurning;
  private final TransferPosition transfer;
  private final BumpDown bumpDown;
  private final ArmReady armReady;
  private final GetCoral getCoral;
  
  


  private final SequentialCommandGroup autoTransfer;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);





  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
  //initialize all teleop and auto commands here
    manualElevator = new ManualElevatorCommand(elevator, operatorXbox);
    manualArm = new ManualArmCommand(arm, operatorXbox);
    autoElevator = new AutoElevatorCommand(elevator, operatorXbox);
    transfer = new TransferPosition(elevator, arm, operatorXbox);
    bumpDown = new BumpDown(elevator, arm, operatorXbox);
    armReady = new ArmReady(elevator, arm, operatorXbox);
    AlignReef = new AutoAlignReefCommand(drivebase, driverXbox);
    getCoral = new GetCoral(elevator, arm, operatorXbox);
    changeTurning = new ChangeTurningCommand(drivebase, driverXbox);

    autoTransfer = new SequentialCommandGroup(transfer, bumpDown, armReady); //sequential command group for auto transfer
    


    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    Command standardDrive = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(Constants.DrivebaseConstants.SlowDownTurn*-driverXbox.getRightX(), OperatorConstants.DEADBAND));
    
    drivebase.setDefaultCommand(standardDrive);
    //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  
   
   // Default Commands
    //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    elevator.setDefaultCommand(manualElevator); //should these be in the section above the bracket?
    arm.setDefaultCommand(manualArm);
  
   
  // Button Mapping
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(Commands.none());

      driverXbox.leftBumper().whileTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      //driverXbox.rightStick().onTrue(changeTurning);//On click change turning modes(W.I.P.)
      driverXbox.b().onTrue(AlignReef);
      //operatorXbox.back().onTrue(autoTransfer);
   

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Mid - 21-10");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
