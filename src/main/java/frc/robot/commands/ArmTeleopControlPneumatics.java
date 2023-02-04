// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPositions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// ArmTeleopControl with only pneumatics
public class ArmTeleopControlPneumatics extends CommandBase {
  /** Creates a new ArmTeleopControl. */
  public ArmTeleopControlPneumatics() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_armController.rightBumper().onTrue(new ClawControl("toggle"));

    BooleanSupplier breakBeamsState = () -> {
      return (!RobotContainer.arm.breakBeamOne.get() && !RobotContainer.arm.breakBeamTwo.get());
    };
    Trigger breakBeams = new Trigger(breakBeamsState);
    breakBeams.onTrue(new ClawControl("close"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
