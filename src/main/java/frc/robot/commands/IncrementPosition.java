// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPositions;

public class IncrementPosition extends CommandBase {
  public boolean finished = false;

  /** Creates a new IncrementPosition. */
  public IncrementPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.testSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (MotorTest.targetPosition < ArmPositions.rotateOne) {
      MotorTest.targetPosition = ArmPositions.rotateOne;
    } else if (MotorTest.targetPosition >= ArmPositions.rotateOne &&
    MotorTest.targetPosition < ArmPositions.rotateTwo) {
      MotorTest.targetPosition = ArmPositions.rotateTwo;
    } else if (MotorTest.targetPosition >= ArmPositions.rotateTwo &&
    MotorTest.targetPosition < ArmPositions.rotateThree) {
      MotorTest.targetPosition = ArmPositions.rotateThree;
    }
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
