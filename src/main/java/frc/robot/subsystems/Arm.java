// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CANBusIDs.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // Set up four arm motors over CAN (Neo motor type is brushless)
  CANSparkMax leftRotate = new CANSparkMax(ArmConstants.leftRotatorID, MotorType.kBrushless);
  CANSparkMax rightRotate = new CANSparkMax(ArmConstants.rightRotatorID, MotorType.kBrushless);
  CANSparkMax leftExtend = new CANSparkMax(ArmConstants.leftExtenderID, MotorType.kBrushless);
  CANSparkMax rightExtend = new CANSparkMax(ArmConstants.rightExtenderID, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
