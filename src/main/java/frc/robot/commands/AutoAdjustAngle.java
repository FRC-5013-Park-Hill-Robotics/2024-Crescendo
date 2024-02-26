// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.LauncherShoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.trobot5013lib.LinearInterpolator;
import frc.robot.RobotContainer;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.TargetConstants;

public class AutoAdjustAngle extends Command {
  private final LauncherRollers launcherRollers;
  private final LauncherShoulder launcherShoulder;
  private boolean goalSet = false;

  /** Creates a new AutoAdjustAngle. */
  public AutoAdjustAngle(LauncherRollers launcherRollers, LauncherShoulder launcherShoulder) {
    this.launcherRollers = launcherRollers;
    this.launcherShoulder = launcherShoulder;
    addRequirements(launcherRollers, launcherShoulder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Limelight frontLL = RobotContainer.getInstance().getFrontLimelight();
    double ty = frontLL.getVerticalAngleOfErrorDegrees();
    LinearInterpolator LAUNCHER_TY_ANGLE_INTERPOLATOR = LauncherConstants.TargetConstants.LAUNCHER_TY_ANGLE_INTERPOLATOR;
    LinearInterpolator LAUNCHER_TY_SHOOTER_SPEED_INTERPOLATOR = LauncherConstants.TargetConstants.LAUNCHER_TY_SHOOTER_SPEED_INTERPOLATOR;

    double requiredDegreeAngle = LAUNCHER_TY_ANGLE_INTERPOLATOR.getInterpolatedValue(ty);
    double requiredShooterSpeed = LAUNCHER_TY_SHOOTER_SPEED_INTERPOLATOR.getInterpolatedValue(ty);

    launcherShoulder.goToSetpointCommand(Math.toRadians(requiredDegreeAngle));
    goalSet = true;
    launcherRollers.setSpeedCommand(requiredShooterSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalSet && launcherShoulder.atGoal();
  }
}
