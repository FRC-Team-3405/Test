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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // For the Shuffleboard
  private NetworkTableEntry extension;
  private NetworkTableEntry rotation;

  // Set up four arm motors over CAN (Neo motor type is brushless)
  CANSparkMax rotator = new CANSparkMax(ArmCANBusIDs.leftRotatorID, MotorType.kBrushless);
  CANSparkMax rotatorFollower = new CANSparkMax(ArmCANBusIDs.rightRotatorID, MotorType.kBrushless);
  CANSparkMax extender = new CANSparkMax(ArmCANBusIDs.leftExtenderID, MotorType.kBrushless);
  CANSparkMax extenderFollower = new CANSparkMax(ArmCANBusIDs.rightExtenderID, MotorType.kBrushless);

  // Forward channel opens claw, reverse channel closes claw
  DoubleSolenoid claw =  new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsChannels.clawOpen, PneumaticsChannels.clawClose);

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
    extender.setIdleMode(IdleMode.kCoast);
    extenderFollower.setIdleMode(IdleMode.kCoast);
    extenderFollower.follow(extender, true);
    extenderPID.setP(0.06);
    extenderPID.setI(0.0);
    extenderPID.setD(0.08);
    extenderPID.setFF(0.0);

    rotator.setIdleMode(IdleMode.kCoast);
    rotatorFollower.setIdleMode(IdleMode.kCoast);
    rotatorFollower.follow(rotator, true);
    rotatorPID.setP(0.0);
    rotatorPID.setI(0.0);
    rotatorPID.setD(0.0);
    rotatorPID.setFF(0.0);

    // Make a NetworkTable
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable( "Arm Encoders");
    rotation = table.getEntry("Rotation");
    extension = table.getEntry("Extension");
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
    claw.set(Value.kReverse);
  }

  public void openClaw() {
    claw.set(Value.kForward);
  }

  public void toggleClaw() {
    claw.toggle();
  }
}
