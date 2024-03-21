// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.LauncherShoulder;

public class ShootWhileMoving extends Command {
  private final LauncherRollers launcherRollers;
  private final LauncherShoulder launcherShoulder;
  private final IntakeRollers intakeRollers;

  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(LauncherRollers launcherRollers, LauncherShoulder launcherShoulder, IntakeRollers intakeRollers) {
    this.launcherRollers = launcherRollers;
    this.launcherShoulder = launcherShoulder;
    this.intakeRollers = intakeRollers;
    addRequirements(launcherRollers, launcherShoulder, intakeRollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
