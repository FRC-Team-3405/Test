package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PIDControllerTest extends CommandBase {
  /** Creates a new PIDControllerTest. */
  public PIDControllerTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.PIDTestSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_driverController.getAButton()) {
      RobotContainer.PIDTestSystem.setPosition(1.0);
    } else if (RobotContainer.m_driverController.getBButton()) {
      RobotContainer.PIDTestSystem.setPosition(-1.0);
    } else if (RobotContainer.m_driverController.getXButton()) {
      RobotContainer.PIDTestSystem.setPosition(0.0);
    } else if (RobotContainer.m_driverController.getYButton()) {
      RobotContainer.PIDTestSystem.setPosition(2.0);
    }
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
