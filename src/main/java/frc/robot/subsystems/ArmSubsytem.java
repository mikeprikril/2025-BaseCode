// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsytem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  SparkMax armMotor;
  RelativeEncoder shoulderEncoder;
  
  WPI_VictorSPX gripperWheel;

  SparkMaxConfig armMotorConfig = new SparkMaxConfig();

  //DigitalInput armBottomLimitSwitch;
  DigitalInput armTopLimitSwitch;
  //SparkLimitSwitch armUpSwitch;

  private SparkClosedLoopController pidController;

  public ArmSubsytem() {
    armMotorConfig.inverted(false) //dont invert shoulder motor
    .idleMode(IdleMode.kBrake); //keep brake on
    armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder) //define encoder for control (might have to change)
    .pid(.0001, 0, 0) //PID constants
    .outputRange(-.3, 1); //arm PID range

    armMotor = new SparkMax(Constants.ArmConstants.armMotorCANID, MotorType.kBrushless);
    armMotor.configure(armMotorConfig, null, null);

    //armUpSwitch = armMotor.getForwardLimitSwitch();

    shoulderEncoder = armMotor.getEncoder();

    gripperWheel = new WPI_VictorSPX(Constants.ArmConstants.GripperCANID);

    //armBottomLimitSwitch = new DigitalInput(Constants.ArmConstants.armBottomLimitSwitchIO);
    armTopLimitSwitch = new DigitalInput(Constants.ArmConstants.armTopLimitSwitchIO);
  }

    public void ArmJoystickControl(double armCommandSpeed){
    if(armCommandSpeed > 0 && armTopLimitSwitch.get() == false){ //dont move up if pushing upper limit switch
      armMotor.stopMotor();
    }
    else if(armCommandSpeed > 0 && -shoulderEncoder.getPosition() < Constants.ArmConstants.AlmostUpValue && armTopLimitSwitch.get() == true){//go up slow if close to upper limit
      armMotor.set(Constants.ArmConstants.SlowDown*armCommandSpeed);
    }
    else if(armCommandSpeed < 0 && -shoulderEncoder.getPosition() > Constants.ArmConstants.AlmostDownValue && -shoulderEncoder.getPosition() < Constants.ArmConstants.ArmAtTray){//go up slow if close to low limit
      armMotor.set(Constants.ArmConstants.SlowDown*armCommandSpeed);
    }
    else if(armCommandSpeed < 0 && -shoulderEncoder.getPosition() > Constants.ArmConstants.ArmAtTray){//stop if arm is at tray
      armMotor.stopMotor();
    }
    else if (armCommandSpeed > -Constants.ElevatorConstants.JoystickDeadband && armCommandSpeed < Constants.ElevatorConstants.JoystickDeadband){
      armMotor.stopMotor();
    }
    else{
      armMotor.set(armCommandSpeed*Constants.ArmConstants.goSlow); //this assumes motor up is pulling back on joystick (positive values)
    }

    if (armTopLimitSwitch.get() == false){
      shoulderEncoder.setPosition(0); //reset encoder to zero when upper limit is pressed
    }
  }

  public void AutoArmMove (double requestSpeed){
    armMotor.set(requestSpeed);
  }

  public void PIDArm(double ArmSetpoint){
    pidController.setReference(ArmSetpoint, ControlType.kPosition);
  
  }

  public void StopArm(){
    armMotor.stopMotor();
  }

  public double GetArmEncoderPosition(){
    return -shoulderEncoder.getPosition();
   }
  
  public void ResetArmEncoder(){
    if (armTopLimitSwitch.get() == false){
      shoulderEncoder.setPosition(0); //reset encoder to zero when upper limit is pressed
    }
   }
  
  public boolean GetTopLimitSwitch(){
    return !armTopLimitSwitch.get();
   }
    
  public boolean GetBottomLimitSwitch(){
    return -shoulderEncoder.getPosition() > Constants.ArmConstants.ArmAtTray;
    }

  public void GripperIntake(){
    gripperWheel.set(Constants.ArmConstants.GripperInSpeed);
  }
  public void GripperSpitOut(){
    gripperWheel.set(Constants.ArmConstants.GripperOutSpeed);

  }  public void StopGripper(){
    gripperWheel.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder Value", -shoulderEncoder.getPosition()); //show arm encoder on dashboard
    SmartDashboard.putBoolean("Arm Bottom Limit", -shoulderEncoder.getPosition() > Constants.ArmConstants.ArmAtTray); //true if enconcder shows we're down at bottom
    SmartDashboard.putBoolean("Arm Top Limit", !armTopLimitSwitch.get());
  }
}
