// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LauncherShoulder;

public class AmpCommand extends Command {
  LauncherShoulder launcher;
  IntakeRollers intakeRollers;
  IntakeWrist intake;
  public AmpCommand(LauncherShoulder launcher, IntakeRollers intakeRollers, IntakeWrist intake) {
    this.intake = intake;
    this.intakeRollers = intakeRollers;
    this.launcher = launcher;
    addRequirements(intake, intakeRollers, launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.ampAngle();
    intake.amp();
    if (intake.atGoal() && launcher.atGoal()){
      intakeRollers.ampOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.retract();
    intakeRollers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeRollers.doesntHaveGamePiece();
  }
}
