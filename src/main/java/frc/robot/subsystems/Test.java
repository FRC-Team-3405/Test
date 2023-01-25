// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(24, MotorType.kBrushless);
  SparkMaxPIDController control = motor.getPIDController();

  /** Creates a new Test. */
  public Test() {
    motor.setIdleMode(IdleMode.kCoast);
    control.setP(0.06);
    control.setI(0);
    control.setD(0);
    control.setFF(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double position) {
    control.setReference(position, ControlType.kPosition);
  }
}
