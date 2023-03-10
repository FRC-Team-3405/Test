// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPositions;

public class IncrementPosition extends CommandBase {
  String component;
  public boolean finished = false;

  /** Creates a new IncrementPosition. */
  public IncrementPosition(String component) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
    this.component = component;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (component.equals("rotate")) {
      if (ArmTeleopControl.rotateTarget < ArmPositions.rotateOne) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateOne;
      } else if (ArmTeleopControl.rotateTarget >= ArmPositions.rotateTwo &&
      ArmTeleopControl.rotateTarget < ArmPositions.rotateTwo) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateThree;
      } else if (ArmTeleopControl.rotateTarget >= ArmPositions.rotateThree &&
      ArmTeleopControl.rotateTarget < ArmPositions.rotateFour) {
        ArmTeleopControl.rotateTarget = ArmPositions.rotateFour;
      }
    } else if (component.equals("extend")) {
      if (ArmTeleopControl.extendTarget < ArmPositions.extendOne) {
        ArmTeleopControl.extendTarget = ArmPositions.extendOne;
      } else if (ArmTeleopControl.extendTarget >= ArmPositions.extendOne &&
      ArmTeleopControl.extendTarget < ArmPositions.extendTwo) {
        ArmTeleopControl.extendTarget = ArmPositions.extendTwo;
      } else if (ArmTeleopControl.extendTarget >= ArmPositions.extendTwo &&
      ArmTeleopControl.extendTarget < ArmPositions.extendThree) {
        ArmTeleopControl.extendTarget = ArmPositions.extendThree;
      } else if (ArmTeleopControl.extendTarget >= ArmPositions.extendThree 
      && ArmTeleopControl.extendTarget < ArmPositions.extendFour) {
        ArmTeleopControl.extendTarget = ArmPositions.extendFour;
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
