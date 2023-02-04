// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClawControl extends CommandBase {
  String action;
  boolean finished = false;

  /** Creates a new ClawControl. */
  public ClawControl(String action) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.action = action;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (action.equals("close")) {
      RobotContainer.arm.closeClaw();
    } else if (action.equals("open")) {
      RobotContainer.arm.openClaw();
    } else if (action.equals("toggle")) {
      RobotContainer.arm.toggleClaw();
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
