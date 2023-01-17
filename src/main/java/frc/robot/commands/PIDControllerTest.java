// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDControllerTest extends CommandBase {
  Timer t;
  int id = 26;
  double Kp,Ki,Kd,Kff;
  double setpoint;
  CANSparkMax spark = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);
  RelativeEncoder neoEncoder = spark.getEncoder();
  SparkMaxPIDController PID = spark.getPIDController();
  /** Creates a new PIDControllerTest. */
  public PIDControllerTest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PID.setP(Kp);
    PID.setI(Ki);
    PID.setD(Kd);
    PID.setFF(Kff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PID.setReference(setpoint, ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
