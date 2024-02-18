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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GamepadDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.LauncherShoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.StatusLED;
import frc.sysID.LauncherShoulderId;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private Climber m_climber = new Climber(); //creates the climber instance variable

  private IntakeRollers m_intakeRollers = new IntakeRollers(); //creates the intake rollers instance variable
  private IntakeWrist m_intakeWrist = new IntakeWrist(m_intakeRollers); //creates the intake wrist instance variable

  private LauncherRollers m_launcherRollers = new LauncherRollers(); //creates the launcher rollers instance variable
  private LauncherShoulder m_launcherShoulder = new LauncherShoulder(); //creates the launcher shoulder variable
  
  private Limelight m_LimelightFront = new Limelight("limelight-front",true); //creates the limelight front instance variable
  private Limelight m_LimelightBack = new Limelight("limelight-back",true); //creates the limelight back instance variable

  private StatusLED m_statusLED = new StatusLED(); //creates the status led instance variable

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private static RobotContainer instance;
  //private final frc.sysID.IntakeWristId sysIdIntakeWrist = new frc.sysID.IntakeWristId();

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
    
    //joystick.a().whileTrue(sysIdIntakeWrist.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //joystick.b().whileTrue(sysIdIntakeWrist.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    //joystick.x().whileTrue(sysIdIntakeWrist.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //joystick.y().whileTrue(sysIdIntakeWrist.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    /*joystick.a().whileTrue(drivetrain.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
    joystick.b().whileTrue(drivetrain.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));

    joystick.x().whileTrue(drivetrain.runDriveDynamTest(SysIdRoutine.Direction.kForward));
    joystick.y().whileTrue(drivetrain.runDriveDynamTest(SysIdRoutine.Direction.kReverse));

    joystick.povDown().whileTrue(drivetrain.runSteerQuasiTest(SysIdRoutine.Direction.kForward));
    joystick.povRight().whileTrue(drivetrain.runSteerQuasiTest(SysIdRoutine.Direction.kReverse));

    joystick.povLeft().whileTrue(drivetrain.runSteerDynamTest(SysIdRoutine.Direction.kForward));
    joystick.povUp().whileTrue(drivetrain.runSteerDynamTest(SysIdRoutine.Direction.kReverse));*/

    //joystick.a().whileTrue(m_intakeWrist.deployCommand()).onFalse(m_intakeWrist.stopCommand());
    //joystick.b().whileTrue(m_intakeWrist.retractCommand());

    //joystick.a().whileTrue(m_intakeRollers.takeIn()).onFalse(m_intakeRollers.stopC());
    //joystick.b().whileTrue(m_intakeRollers.throwOut()).onFalse(m_intakeRollers.stopC());

    //oystick.x().whileTrue(m_intakeRollers.intakeGamepieceCommand()).onFalse(m_intakeRollers.stopC());
    //new Trigger(m_intakeRollers::hasGamePiece).onTrue(rumbleSequence());

    //joystick.a().whileTrue(m_launcherShoulder.goToSetpointCommand(45));
    //joystick.b().whileTrue(m_launcherShoulder.goToSetpointCommand(30));


    //decrease rps by 5
    joystick.povLeft().onTrue(m_launcherRollers.incrementSpeedCommand(-5));

    //increase rps by 5
    joystick.povRight().onTrue(m_launcherRollers.incrementSpeedCommand(5));

    //shooter angle increase by 5 deg += 5
    joystick.povUp().onTrue(m_launcherShoulder.incrementAngleCommand(Math.toRadians(5)));

    //shooter angle decrease by 5 deg
    joystick.povDown().onTrue(m_launcherShoulder.incrementAngleCommand(Math.toRadians(-5)));

    //joystick.x().whileTrue(m_shoulderId.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //joystick.y().whileTrue(m_shoulderId.sysIdDynamic(SysIdRoutine.Direction.kReverse));


    joystick.rightBumper().onTrue(m_intakeWrist.intakeGamePiece().andThen(rumbleSequence()));

    joystick.a().onTrue(m_intakeRollers.throwOut());

    joystick.b().onTrue(m_launcherRollers.startCommand());
    joystick.x().onTrue(m_launcherRollers.stopCommand());

    joystick.y().onTrue(new InstantCommand(m_intakeRollers::feedOut)).onFalse(new InstantCommand(m_intakeRollers::stop));

   // new Trigger(m_intakeRollers::hasGamePiece).onTrue(m_launcherRollers.startCommand());

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
  
  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public Command rumbleSequence(){
    Command rumbleCommand =  new InstantCommand(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 1));
    Command stopRumbleCommand =  new InstantCommand(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 0));
    return rumbleCommand.andThen(new WaitCommand(0.5)).andThen(stopRumbleCommand);
   
  }
}
