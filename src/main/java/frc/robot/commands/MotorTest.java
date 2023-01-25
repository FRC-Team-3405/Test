// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class MotorTest extends CommandBase {
  static double targetPosition = 0;

  /** Creates a new MotorTest. */
  public MotorTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.testSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_driverController.a().onTrue(new DecrementPosition());
    RobotContainer.m_driverController.y().onTrue(new IncrementPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MotorTest.targetPosition += RobotContainer.m_driverController.getLeftY() * -0.1;
    RobotContainer.testSystem.setPosition(MotorTest.targetPosition);
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
