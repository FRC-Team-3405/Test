// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDControllerTest extends CommandBase {
  PIDController PID = new PIDController(0,0,0);
  Timer t;
  int port = 26;
  // Not sure if this should be WPI_TalonFX or just TalonFX. (Both classes exist)
  WPI_TalonFX talon = new WPI_TalonFX(port);
  TalonFXSensorCollection encoder = new TalonFXSensorCollection(talon);
  /** Creates a new PIDControllerTest. */
  public PIDControllerTest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PID.setSetpoint(1.0);
    t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talon.set(TalonFXControlMode.PercentOutput, PID.calculate(encoder.getIntegratedSensorPosition()));
    if (PID.atSetpoint()) {
      PID.reset();
    } else {
      t.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t.hasElapsed(2.0);
  }
}
