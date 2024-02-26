// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.trobot5013lib.TrobotUtil;

public class ClimbCommand extends Command {
  Climber climber;
  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climber climber) {
    addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CommandXboxController operatorController = RobotContainer.getInstance().getOperatorController();
    double leftStickInput = operatorController.getLeftY();
    double rightStickInput = operatorController.getRightY();

    TalonFX leftMotor = climber.getLeftMotor();
    TalonFX rightMotor = climber.getRightMotor();

    leftMotor.set(TrobotUtil.modifyAxis(leftStickInput, .05));
    rightMotor.set(TrobotUtil.modifyAxis(rightStickInput, .05));

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
