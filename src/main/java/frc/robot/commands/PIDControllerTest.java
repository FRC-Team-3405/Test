// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDControllerTest extends CommandBase {
  PIDController PID = new PIDController(0,0,0);
  Timer t;
  int id = 26;
  CANSparkMax spark = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);
  RelativeEncoder neoEncoder = spark.getEncoder();
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
    spark.set(PID.calculate(neoEncoder.getPosition()));
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
