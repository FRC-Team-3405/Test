// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.TestArmPositions;

public class DecrementPosition extends CommandBase {
  String component;
  public boolean finished = false;

  /** Creates a new DecrementPosition. */ 
  public DecrementPosition(String component) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.testSystem);
    addRequirements(RobotContainer.arm);
    this.component = component;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (component.equals("test")) {
      if (MotorTest.targetPosition > TestArmPositions.rotateThree) {
        MotorTest.targetPosition = TestArmPositions.rotateThree;
      } else if (MotorTest.targetPosition <= TestArmPositions.rotateThree 
      && MotorTest.targetPosition > TestArmPositions.rotateTwo) {
        MotorTest.targetPosition = TestArmPositions.rotateTwo;
      } else if (MotorTest.targetPosition <= TestArmPositions.rotateTwo 
      && MotorTest.targetPosition > TestArmPositions.rotateOne) {
        MotorTest.targetPosition = TestArmPositions.rotateOne;
      }
    } else if (component.equals("rotate")) {
      if (ArmTeleopControl.rotateTarget > ArmPositions.rotateFour) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateFour;
      } else if (ArmTeleopControl.rotateTarget <= ArmPositions.rotateFour
      && ArmTeleopControl.rotateTarget > ArmPositions.rotateThree) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateThree;
      } else if (ArmTeleopControl.rotateTarget <= ArmPositions.rotateThree 
      && ArmTeleopControl.rotateTarget > ArmPositions.rotateTwo) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateTwo;
      } else if (ArmTeleopControl.rotateTarget <= ArmPositions.rotateTwo 
      && ArmTeleopControl.rotateTarget > ArmPositions.rotateOne) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateOne;
      }
    } else if (component.equals("extend")) {
      if (ArmTeleopControl.extendTarget > ArmPositions.extendFour) {
        ArmTeleopControl.extendTarget = ArmPositions.extendFour;
      } else if (ArmTeleopControl.extendTarget <= ArmPositions.extendFour
      && ArmTeleopControl.extendTarget > ArmPositions.extendThree) {
        ArmTeleopControl.extendTarget = ArmPositions.extendThree;
      } else if (MotorTest.targetPosition <= ArmPositions.extendThree 
      && MotorTest.targetPosition > ArmPositions.extendTwo) {
        ArmTeleopControl.extendTarget = ArmPositions.extendTwo;
      } else if (MotorTest.targetPosition <= ArmPositions.extendTwo 
      && MotorTest.targetPosition > ArmPositions.extendOne) {
        ArmTeleopControl.extendTarget = ArmPositions.extendOne;
      }
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
