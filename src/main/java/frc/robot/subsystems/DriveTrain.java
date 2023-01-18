// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {
  public final WPI_TalonSRX leftMotor = new WPI_TalonSRX(DriveConstants.leftMotorID);
  public final WPI_TalonSRX rightMotor = new WPI_TalonSRX(DriveConstants.rightMotorID);

  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }
}
