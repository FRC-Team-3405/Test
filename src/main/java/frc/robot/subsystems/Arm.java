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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // For the Shuffleboard
  private NetworkTableEntry leftExtension;
  private NetworkTableEntry rightExtension;
  private NetworkTableEntry leftRotation;
  private NetworkTableEntry rightRotation;

  // Set up four arm motors over CAN (Neo motor type is brushless)
  CANSparkMax rotator = new CANSparkMax(ArmCANBusIDs.leftRotatorID, MotorType.kBrushless);
  CANSparkMax rotatorFollower = new CANSparkMax(ArmCANBusIDs.rightRotatorID, MotorType.kBrushless);
  CANSparkMax extender = new CANSparkMax(ArmCANBusIDs.leftExtenderID, MotorType.kBrushless);
  CANSparkMax extenderFollower = new CANSparkMax(ArmCANBusIDs.rightExtenderID, MotorType.kBrushless);

  // Forward channel opens claw, reverse channel closes claw
  private boolean isClosed = false;
  DoubleSolenoid claw =  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsChannels.clawOpen, PneumaticsChannels.clawClose); // Change back to REV PH when we get the chance

  // PID controller to control the rotation and extension of the arm
  SparkMaxPIDController rotatorPID = rotator.getPIDController();
  SparkMaxPIDController extenderPID = extender.getPIDController();

  // Encoders to output current rotation and extension of the arm
  RelativeEncoder leftRotateEncoder = rotator.getEncoder();
  RelativeEncoder rightRotateEncoder = rotatorFollower.getEncoder();
  RelativeEncoder leftExtendEncoder = extender.getEncoder();
  RelativeEncoder rightExtendEncoder = extenderFollower.getEncoder();
  
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
    rotatorPID.setP(0.06);
    rotatorPID.setI(0.0);
    rotatorPID.setD(0.08);
    rotatorPID.setFF(0.0);

    // Make a NetworkTable
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable( "Arm Encoders");
    leftRotation = table.getEntry("Left rotation");
    rightRotation = table.getEntry("Right rotation");
    leftExtension = table.getEntry("Left extension");
    rightExtension = table.getEntry("Right extension");

    claw.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftRotation.setDouble(leftRotateEncoder.getPosition());
    rightRotation.setDouble(rightRotateEncoder.getPosition());
    leftExtension.setDouble(leftExtendEncoder.getPosition());
    rightExtension.setDouble(rightExtendEncoder.getPosition());
  }

  public void setRotatePosition(double position) {
    rotatorPID.setReference(position, ControlType.kPosition);
  }

  public void setExtendPosition(double position) {
    extenderPID.setReference(position, ControlType.kPosition);
  }

  public void toggleClaw() {
    claw.toggle();
    isClosed = !isClosed;
  }

  public CommandBase ToggleClaw() {
    return runOnce(
      () -> {
        toggleClaw();
      });
  }
}
