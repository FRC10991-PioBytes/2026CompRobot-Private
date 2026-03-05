// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AprilTagCommands.AprilTagTrackAndMoveCommand;
import frc.robot.commands.AprilTagCommands.FindAndTrackTagCommand;
import frc.robot.commands.AprilTagCommands.FindAprilTagCommand;
import frc.robot.commands.AprilTagCommands.TrackAprilTagCommand;
import frc.robot.commands.AutomaticShootingCommands.MoveToScoreDistance;
import frc.robot.commands.AutomaticShootingCommands.MoveToScorePosCommand;
import frc.robot.commands.AutomaticShootingCommands.ShootFromScorePosCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommands.RunIntakeInCommand;
import frc.robot.commands.IntakeCommands.RunIntakeOutCommand;
import frc.robot.commands.IntakeCommands.RunIntakePivotCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();

  // Driver controller
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);
  
  // Operator controller
  CommandJoystick m_manipulatorController = new CommandJoystick(OIConstants.kManipulatorControllerPort);

  // Auto chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> pathAutoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("StopFeeder", m_feeder.runOnce(() -> m_feeder.stop()));
    NamedCommands.registerCommand("StopShooter", m_shooter.runOnce(() -> m_shooter.stop()));
    NamedCommands.registerCommand("ExtendIntake", Commands.print("Extended Intake"));
    NamedCommands.registerCommand("RevShooter", m_shooter.runOnce(() -> m_shooter.setVelocity(4200)));
    NamedCommands.registerCommand("RunFeederAndShoot", m_feeder.runOnce(() -> m_feeder.setVelocity(3000)));
    NamedCommands.registerCommand("RunIntakeRoller", Commands.print("Running intake roller"));

    /*
    NamedCommands.registerCommand("ExtendIntake", new RunIntakePivotOutCommand(m_intake));
    NamedCommands.registerCommand("RevShooter", m_shooter.runOnce(() -> m_shooter.setVelocity(4200)));
    NamedCommands.registerCommand("RunFeederAndShoot", m_feeder.runOnce(() -> m_feeder.setVelocity(3000)));
    NamedCommands.registerCommand("RunIntakeRoller", new RunIntakeInCommand(m_intake));
    */

    pathAutoChooser = AutoBuilder.buildAutoChooser("CenterStart-Score");

    /*
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

    
    autoChooser.addOption("Drive Forward", new DriveForwardAuto());
    autoChooser.addOption("Track April Tag", new TrackAprilTagCommand(m_robotDrive));
    autoChooser.addOption("April Tag Track and Move Command", new AprilTagTrackAndMoveCommand(m_robotDrive));
    autoChooser.addOption("April Tag finder", new FindAprilTagCommand(m_robotDrive));
    autoChooser.addOption("Find and Track Tag Command", new FindAndTrackTagCommand(m_robotDrive));
    autoChooser.addOption("MoveToScoreDistance", new MoveToScoreDistance(m_robotDrive));

    SmartDashboard.putData("Auto/Auto Mode", autoChooser);
    */
    SmartDashboard.putData("Auto/PP Autos", pathAutoChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive,
        () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband) * 0.57,
        () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband) * 0.57,
        () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband) * 0.57,
        () -> true));

    m_driverController.button(OIConstants.bumperLeft)
        .whileTrue(new DriveCommand(m_robotDrive, 
            () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband) * 0.29,
            () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband) * 0.29,
            () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband) * 0.29,
            () -> true));

    m_driverController.button(OIConstants.buttonB).and(m_driverController.button(OIConstants.bumperLeft)).and(m_driverController.button(OIConstants.bumperRight))
        .onTrue(m_robotDrive.runOnce(() -> m_robotDrive.zeroHeading()));
    
    // Move to closest scoring position
    m_driverController.button(OIConstants.buttonA)
        .whileTrue(new MoveToScorePosCommand(m_robotDrive));

    // Set X formation
    m_driverController.button(OIConstants.buttonX)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    // Run shooter
    m_manipulatorController.axisGreaterThan(OIConstants.rightTrigger, 0.5)
        .onTrue(new ShootFromScorePosCommand(m_shooter, m_robotDrive))
        .onFalse(m_shooter.runOnce(() -> m_shooter.stop()));
    
    // Run feeder
    m_manipulatorController.axisGreaterThan(OIConstants.leftTrigger, 0.5)
        .onTrue(m_feeder.runOnce(() -> m_feeder.setVelocity(3000)))
        .onFalse(m_feeder.runOnce(() -> m_feeder.stop()));

    // Run intake
    m_manipulatorController.button(OIConstants.buttonX)
        .toggleOnTrue(new RunIntakeInCommand(m_intake));

    m_manipulatorController.button(OIConstants.buttonY)
        .toggleOnTrue(new RunIntakeOutCommand(m_intake));

    // Run agitator
    /*
    m_manipulatorController.button(OIConstants.buttonA)
        .whileTrue(m_feeder.runOnce(() -> m_feeder.runAgitator(-1)));
    */

    // Run intake
    m_intake.setDefaultCommand(new RunIntakePivotCommand(
        m_intake,
        () -> MathUtil.applyDeadband(m_manipulatorController.getRawAxis(OIConstants.rightStickY), 0.1)));
    

    // Sys Id Routines
    /*
    m_driverController.button(OIConstants.buttonA)
        .whileTrue(m_feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    m_driverController.button(OIConstants.buttonB)
        .whileTrue(m_feeder.sysIdDynamic(SysIdRoutine.Direction.kForward));

    m_driverController.button(OIConstants.buttonX)
        .whileTrue(m_feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    m_driverController.button(OIConstants.buttonY)
        .whileTrue(m_feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(3.538, 4.035, new Rotation2d(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        null,
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.82, 4.035, new Rotation2d(180)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
    */
    
    //return pathAutoChooser.getSelected();
    return new PathPlannerAuto("CenterStart-Score");
  }

}