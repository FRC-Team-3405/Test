// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PneumaticsChannels;
import frc.robot.Constants.CANBusIDs.ArmCANBusIDs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // For the dashboard
  private NetworkTableEntry extension;
  private NetworkTableEntry rotation;

  // Set up four arm motors over CAN (Neo motor type is brushless)
  CANSparkMax rotator = new CANSparkMax(ArmCANBusIDs.leftRotatorID, MotorType.kBrushless);
  CANSparkMax rotatorFollower = new CANSparkMax(ArmCANBusIDs.rightRotatorID, MotorType.kBrushless);
  CANSparkMax extender = new CANSparkMax(ArmCANBusIDs.leftExtenderID, MotorType.kBrushless);
  CANSparkMax extenderFollower = new CANSparkMax(ArmCANBusIDs.rightExtenderID, MotorType.kBrushless);

  // Forward channel closes claw, reverse channel opens claw
  DoubleSolenoid claw =  new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsChannels.clawClose, PneumaticsChannels.clawOpen);

  // PID controller to control the rotation and extension of the arm
  SparkMaxPIDController rotatorPID = rotator.getPIDController();
  SparkMaxPIDController extenderPID = extender.getPIDController();

  // Encoders to output current rotation and extension of the arm
  RelativeEncoder rotatorEncoder = rotator.getEncoder();
  RelativeEncoder extenderEncoder = extender.getEncoder();
  
  // Break beam sensors to automatically close the claw
  public DigitalInput breakBeamOne = new DigitalInput(0);
  public DigitalInput breakBeamTwo = new DigitalInput(1);

  /** Creates a new Arm. */
  public Arm() {
    rotatorFollower.follow(rotator, true);
    extenderFollower.follow(extender, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotation.setDouble(rotatorEncoder.getPosition());
    extension.setDouble(extenderEncoder.getPosition());
  }

  public void setRotatePosition(double position) {
    rotatorPID.setReference(position, ControlType.kPosition);
  }

  public void setExtendPosition(double position) {
    extenderPID.setReference(position, ControlType.kPosition);
  }

  public void closeClaw() {
    claw.set(Value.kForward);
  }

  public void openClaw() {
    claw.set(Value.kReverse);
  }

  public void toggleClaw() {
    claw.toggle();
  }
}
