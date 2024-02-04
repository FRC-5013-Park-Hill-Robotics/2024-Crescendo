// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.GamepadDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.LauncherShoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.StatusLED;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private Climber m_climber = new Climber(); //creates the climber instance variable

  private IntakeRollers m_intakeRollers = new IntakeRollers(); //creates the intake rollers instance variable
  private IntakeWrist m_intakeWrist = new IntakeWrist(); //creates the intake wrist instance variable

  private LauncherRollers m_launcherRollers = new LauncherRollers(); //creates the launcher rollers instance variable
  private LauncherShoulder m_launcherShoulder = new LauncherShoulder(); //creates the launcher shoulder variable

  private Limelight m_LimelightFront = new Limelight(); //creates the limelight front instance variable
  private Limelight m_LimelightBack = new Limelight(); //creates the limelight back instance variable

  private StatusLED m_statusLED = new StatusLED(); //creates the status led instance variable

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private static RobotContainer instance;

  public RobotContainer() {
    super();
    instance = this;
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new GamepadDrive(drivetrain, joystick));
    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
        //.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyroscope()));
 /* 
    joystick.a().whileTrue(m_launcherRollers.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    joystick.b().whileTrue(m_launcherRollers.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    joystick.x().whileTrue(m_launcherRollers.sysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.y().whileTrue(m_launcherRollers.sysIdDynamic(SysIdRoutine.Direction.kReverse));
*/
    joystick.a().whileTrue(new InstantCommand(() -> m_launcherRollers.setSpeed(1)));



    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public LauncherShoulder getLauncherShoulder() {
    return m_launcherShoulder;
  }

}
