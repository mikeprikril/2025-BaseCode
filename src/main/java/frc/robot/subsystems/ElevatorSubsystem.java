// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax leftElevatorMotor;
  SparkMax rightElevatorMotor;

  //SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
  //SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
  RelativeEncoder elevatorEncoder;

  DigitalInput elevatorBottomLimitSwitch;
  //DigitalInput elevatorTopLimitSwitch;

  private SparkClosedLoopController pidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public ElevatorSubsystem() {
    
    /*leftMotorConfig.inverted(false) //dont invert left motor
    .idleMode(IdleMode.kBrake); //keep brake on
    leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //define encoder for control (might have to change)
    .pid(.0001, 0, 0) //PID constants
    .outputRange(-1, 1); //PID elevator range

    rightMotorConfig.inverted(true) //invert right motor so it spins opposite way
    .idleMode(IdleMode.kBrake) //keep brake on
    .follow(leftElevatorMotor); //follow the left motor
    */
    
    leftElevatorMotor = new SparkMax(Constants.ElevatorConstants.leftMotorCANID, MotorType.kBrushless);
    //leftElevatorMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorEncoder = leftElevatorMotor.getEncoder();

    rightElevatorMotor = new SparkMax(Constants.ElevatorConstants.rightMotorCANID, MotorType.kBrushless);
    //rightElevatorMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = leftElevatorMotor.getClosedLoopController();

    elevatorBottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.elevatorBottomLimitSwitchIO);
    //elevatorTopLimitSwitch = new DigitalInput(Constants.ElevatorConstants.elevatorTopLimitSwitchIO);
  }

  public void ElevatorJoystickControl(double elevatorCommandSpeed){
    if(elevatorCommandSpeed > 0 && elevatorBottomLimitSwitch.get() == false){ //dont move down if pushing lower limit switch
      leftElevatorMotor.stopMotor();
      rightElevatorMotor.stopMotor();
    }
     else if(elevatorCommandSpeed > 0 && -elevatorEncoder.getPosition() < Constants.ElevatorConstants.AlmostDownValue){//go downn slow if close to low limit
      leftElevatorMotor.set(Constants.ElevatorConstants.SlowDown*elevatorCommandSpeed);
      rightElevatorMotor.set(Constants.ElevatorConstants.SlowDown*-elevatorCommandSpeed);
    }
    else if(elevatorCommandSpeed < 0 && -elevatorEncoder.getPosition() > Constants.ElevatorConstants.AlmostUpValue && -elevatorEncoder.getPosition() < Constants.ElevatorConstants.UpLimit){//go up slow if close to high limit
      leftElevatorMotor.set(Constants.ElevatorConstants.SlowDown*elevatorCommandSpeed);
      rightElevatorMotor.set(Constants.ElevatorConstants.SlowDown*-elevatorCommandSpeed);
    } 
    else if(elevatorCommandSpeed < 0 && -elevatorEncoder.getPosition() > Constants.ElevatorConstants.UpLimit){//dont move up if encoder says youre at the top
      leftElevatorMotor.stopMotor();
      rightElevatorMotor.stopMotor();
    }
    else if (elevatorCommandSpeed > -Constants.ElevatorConstants.JoystickDeadband && elevatorCommandSpeed < Constants.ElevatorConstants.JoystickDeadband){
      leftElevatorMotor.stopMotor();
      rightElevatorMotor.stopMotor();
    }
    else{
      leftElevatorMotor.set(elevatorCommandSpeed*Constants.ElevatorConstants.goSlow);
      rightElevatorMotor.set(-elevatorCommandSpeed*Constants.ElevatorConstants.goSlow);
    }

    if (elevatorBottomLimitSwitch.get() == false){
      leftElevatorMotor.getEncoder().setPosition(0); //reset encoder if elevator goes to bottom
    }
  }

  public void AutoElevator (double requestSpeed){
    leftElevatorMotor.set(requestSpeed);
    rightElevatorMotor.set(-requestSpeed);
  }

  public void PIDElevator(double ElevatorSetpoint){
    pidController.setReference(ElevatorSetpoint, ControlType.kPosition);
  
  }

  public void StopElevator(){
    leftElevatorMotor.stopMotor();
    rightElevatorMotor.stopMotor();
  }

  public double GetElevatorEncoderPosition(){
    return -elevatorEncoder.getPosition();
   }
  
  //public void ResetElevatorEncoder(){
  //  if (elevatorBottomLimitSwitch.get() == false){
  //    leftElevatorMotor.getEncoder().setPosition(0);
  //  }
  // }
  
  public boolean GetTopLimitSwitch(){
    return -elevatorEncoder.getPosition() > Constants.ElevatorConstants.UpLimit;
   }
    
  public boolean GetBottomLimitSwitch(){
    return elevatorBottomLimitSwitch.get();
   }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Value", -elevatorEncoder.getPosition()); //show elevator encoder on dashboard
    SmartDashboard.putBoolean("Elevator Bottom Limit", elevatorBottomLimitSwitch.get());
    SmartDashboard.putBoolean("Elevator Top Limit", -elevatorEncoder.getPosition() > Constants.ElevatorConstants.UpLimit);
    SmartDashboard.putNumber("Left Elevator Current ", leftElevatorMotor.getOutputCurrent()); //display motor current draw in amps
    SmartDashboard.putNumber("Right Elevator Current ", rightElevatorMotor.getOutputCurrent());

    SmartDashboard.putNumber("Elevator Encoder Velocity", -elevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Left Elevator Applied Output", leftElevatorMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right Elevator Applied Output", rightElevatorMotor.getAppliedOutput());

  }
}
