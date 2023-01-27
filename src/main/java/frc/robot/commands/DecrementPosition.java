// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TestArmPositions;

public class DecrementPosition extends CommandBase {
  /** Creates a new DecrementPosition. */
  public boolean finished = false; 
  public DecrementPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.testSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (MotorTest.targetPosition > TestArmPositions.rotateThree) {
      MotorTest.targetPosition = TestArmPositions.rotateThree;
    } else if (MotorTest.targetPosition <= TestArmPositions.rotateThree 
    && MotorTest.targetPosition > TestArmPositions.rotateTwo) {
      MotorTest.targetPosition = TestArmPositions.rotateTwo;
    } else if (MotorTest.targetPosition <= TestArmPositions.rotateTwo 
    && MotorTest.targetPosition > TestArmPositions.rotateOne) {
      MotorTest.targetPosition = TestArmPositions.rotateOne;
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
