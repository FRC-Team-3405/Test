// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TestArmPositions;

public class IncrementPosition extends CommandBase {
  String component;
  public boolean finished = false;

  /** Creates a new IncrementPosition. */
  public IncrementPosition(String comp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.testSystem);
    component = comp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (component.equals("test")) {
      if (MotorTest.targetPosition < TestArmPositions.rotateOne) {
        MotorTest.targetPosition = TestArmPositions.rotateOne;
      } else if (MotorTest.targetPosition >= TestArmPositions.rotateOne &&
      MotorTest.targetPosition < TestArmPositions.rotateTwo) {
        MotorTest.targetPosition = TestArmPositions.rotateTwo;
      } else if (MotorTest.targetPosition >= TestArmPositions.rotateTwo &&
      MotorTest.targetPosition < TestArmPositions.rotateThree) {
        MotorTest.targetPosition = TestArmPositions.rotateThree;
      }
    } else if (component.equals("rotate")) {

    } else if (component.equals("extend")) {
      
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
