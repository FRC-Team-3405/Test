// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs.DriveCANBusIDs;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {
  private final DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveCANBusIDs.P_HIGHGEAR, DriveCANBusIDs.P_LOWGEAR);
  public static boolean LowGear = false;
  public final WPI_TalonSRX frontLeft = new WPI_TalonSRX(DriveCANBusIDs.frontleftMotorID);
  public final WPI_TalonSRX backLeft = new WPI_TalonSRX(DriveCANBusIDs.backleftMotorID);
  public final WPI_TalonSRX frontRight = new WPI_TalonSRX(DriveCANBusIDs.frontrightMotorID);
  public final WPI_TalonSRX backRight = new WPI_TalonSRX(DriveCANBusIDs.backrightMotorID);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightMotors.setInverted(true);
    shifter.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void shift() {
    shifter.toggle();
    LowGear = !LowGear;
  }

  public CommandBase ShiftGears() {
    return runOnce(
      () -> {
        shift();
      });
  }
}
