// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.Limelight;


public class TurnToAngle extends Command {
    private CommandSwerveDrivetrain m_Drivetrain;
  private Supplier<Double> m_degreeAngle;
  private CommandXboxController m_gamepad;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(2.5);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(2.5);
  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
  
  /** Creates a new TurnToAngle. */
  public TurnToAngle(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, Supplier<Double> degreeAngle) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_gamepad = controller;
    m_degreeAngle = degreeAngle;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(Math.toRadians(1.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		double throttle = modifyAxis(m_gamepad.getRightTriggerAxis());

		double translationX = modifyAxis(-m_gamepad.getLeftY());
		double translationY = modifyAxis(-m_gamepad.getLeftX());
		if (!(translationX == 0.0 && translationY == 0.0)) {
			double angle = calculateTranslationDirection(translationX, translationY);
			translationX = Math.cos(angle) * throttle;
			translationY = Math.sin(angle) * throttle;
		}
    
    SmartDashboard.putNumber("Angle Pointing to", Math.toRadians(m_degreeAngle.get()));
    m_Drivetrain.setControl(drive
      .withVelocityX(-CommandSwerveDrivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(translationX)))
			.withVelocityY(CommandSwerveDrivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(translationY)))
			.withTargetDirection(new Rotation2d(Math.toRadians(m_degreeAngle.get()))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("TurnToAngle running", false);
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0));
  }

  private static double modifyAxis(double value) {
	
		return modifyAxis(value, 1);
	}
	
	private static double modifyAxis(double value, int exponent) {
		// Deadband
		value = MathUtil.applyDeadband(value, ControllerConstants.DEADBAND);

		 value = Math.copySign(Math.pow(value, exponent), value);

		return value;
	}
	
	private double calculateTranslationDirection(double x, double y) {
		// Calculate the angle.
		// Swapping x/y
		return Math.atan2(x, y) + Math.PI / 2;
	}
}
