// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTeleopControl extends CommandBase {
  /*
   * Note: it is not always clear where the motor's zero point will be,
   * so it would be wise to determine that before permanently attaching
   * any components. Current code will make the motors go back to the zero
   * position automatically when the robot is enabled.
   */
  static double rotateTarget = 0;
  static double extendTarget = 0;

  /** Creates a new ArmTeleopControl. */
  public ArmTeleopControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Parameter "comp" determines whether the rotation or the extension of the arm will be affected
    RobotContainer.m_armController.a().onTrue(new DecrementPosition("rotate"));
    RobotContainer.m_armController.y().onTrue(new IncrementPosition("rotate"));
    RobotContainer.m_armController.x().onTrue(new DecrementPosition("extend"));
    RobotContainer.m_armController.b().onTrue(new IncrementPosition("extend"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    rotateTarget += RobotContainer.m_armController.getLeftY() * -0.1;
    extendTarget += RobotContainer.m_armController.getRightY() * -0.1;
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
