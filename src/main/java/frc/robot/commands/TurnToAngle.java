// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.Limelight;

public class TurnToAngle extends Command {
    private CommandSwerveDrivetrain m_Drivetrain;
  private Supplier<Double> m_degreeAngle;
  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
  /** Creates a new TurnToAngle. */
  public TurnToAngle(CommandSwerveDrivetrain drivetrain,  Supplier<Double> degreeAngle) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_degreeAngle = degreeAngle;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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
    double thetaOutput = 0;
    double xOutput = 0;
    double yOutput = 0;
		double setpoint = Math.toRadians(m_degreeAngle.get())+ m_Drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);
			if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
			} 

    m_Drivetrain.setControl(drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("TurnToAngle running", false);
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
