// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPositions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmTeleopControlRotate extends CommandBase {
  static double rotateTarget = ArmPositions.defaultRotate;
  static double extendTarget = ArmPositions.defaultExtend;

  /** Creates a new ArmTeleopControl. */
  public ArmTeleopControlRotate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Parameter "component" determines whether the rotation or the extension of the arm will be affected
    RobotContainer.m_armController.a().onTrue(new DecrementPosition("rotate"));
    RobotContainer.m_armController.y().onTrue(new IncrementPosition("rotate"));

    // RobotContainer.m_armController.rightBumper().onTrue(new ClawControl("toggle"));
    // RobotContainer.m_armController.leftBumper().onTrue(new ArmReset());

    /* BooleanSupplier breakBeamsState = () -> {
      return (!RobotContainer.arm.breakBeamOne.get() && !RobotContainer.arm.breakBeamTwo.get());
    }; */
    // Trigger breakBeams = new Trigger(breakBeamsState);
    // breakBeams.onTrue(new ClawControl("close"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    ArmTeleopControl.rotateTarget += RobotContainer.m_armController.getLeftY() * -0.1;
    RobotContainer.arm.setRotatePosition(ArmTeleopControl.rotateTarget);
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
