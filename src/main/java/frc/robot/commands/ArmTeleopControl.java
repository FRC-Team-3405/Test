// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTeleopControl extends CommandBase {
  static double rotateTarget = 0;
  static double extendTarget = 0;

  /** Creates a new ArmTeleopControl. */
  public ArmTeleopControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.arm.breakBeamOne.get() && !RobotContainer.arm.breakBeamTwo.get()) {
      RobotContainer.arm.closeClaw();
    }
    
    rotateTarget += RobotContainer.m_driverController.getLeftY();
    extendTarget += RobotContainer.m_driverController.getRightY();

    RobotContainer.arm.setRotatePosition(rotateTarget);
    RobotContainer.arm.setExtendPosition(extendTarget);
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
