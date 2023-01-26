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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // Set up four arm motors over CAN (Neo motor type is brushless)
  CANSparkMax rotator = new CANSparkMax(ArmCANBusIDs.leftRotatorID, MotorType.kBrushless);
  CANSparkMax rotatorFollower = new CANSparkMax(ArmCANBusIDs.rightRotatorID, MotorType.kBrushless);
  CANSparkMax extender = new CANSparkMax(ArmCANBusIDs.leftExtenderID, MotorType.kBrushless);
  CANSparkMax extenderFollower = new CANSparkMax(ArmCANBusIDs.rightExtenderID, MotorType.kBrushless);

  // Forward channel closes claw, reverse channel opens claw
  DoubleSolenoid claw =  new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsChannels.clawClose, PneumaticsChannels.clawOpen);

  SparkMaxPIDController rotatorPID = rotator.getPIDController();
  SparkMaxPIDController extenderPID = extender.getPIDController();

  RelativeEncoder rotatorEncoder = rotator.getEncoder();
  RelativeEncoder extenderEncoder = extender.getEncoder();  

  /** Creates a new Arm. */
  public Arm() {
    rotatorFollower.follow(rotator, true);
    extenderFollower.follow(extender, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
