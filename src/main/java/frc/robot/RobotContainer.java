// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AprilTagCommands.AprilTagTrackAndMoveCommand;
import frc.robot.commands.AprilTagCommands.FindAndTrackTagCommand;
import frc.robot.commands.AprilTagCommands.FindAprilTagCommand;
import frc.robot.commands.AprilTagCommands.TrackAprilTagCommand;
import frc.robot.commands.AutomaticShootingCommands.MoveToScoreDistance;
import frc.robot.commands.AutomaticShootingCommands.MoveToScorePosCommand;
import frc.robot.commands.AutomaticShootingCommands.ShootFromScorePosCommand;
import frc.robot.commands.IntakeCommands.RunIntakeInCommand;
import frc.robot.commands.IntakeCommands.RunIntakePivotCommand;
import frc.robot.commands.IntakeCommands.RunIntakePivotDownCommand;
import frc.robot.commands.IntakeCommands.RunIntakePivotUpCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    NamedCommands.registerCommand("RevShooter", m_shooter.runOnce(() -> m_shooter.setVelocity(4000)));
    NamedCommands.registerCommand("RunFeederAndShoot", m_feeder.runOnce(() -> m_feeder.setVelocity(2000)));
    NamedCommands.registerCommand("StopFeeder", m_feeder.runOnce(() -> m_feeder.stop()));
    NamedCommands.registerCommand("StopShooter", m_shooter.runOnce(() -> m_shooter.stop()));
    NamedCommands.registerCommand("ExtendIntake", m_intake.runOnce(() -> m_intake.setPivotPosition(IntakeConstants.kIntakeExtendedEncoderPosition)));
    NamedCommands.registerCommand("RunIntakeRoller", new RunIntakeInCommand(m_intake));
    
    PathPlannerAuto auto1 = new PathPlannerAuto("CenterStart-CenterScore");
    PathPlannerAuto auto2 = new PathPlannerAuto("LeftScore-NeutralZone");
    PathPlannerAuto auto3 = new PathPlannerAuto("LeftStart-Depot-LeftScore");
    PathPlannerAuto auto4 = new PathPlannerAuto("LeftStart-LeftScore-Depot-LeftScore");
    pathAutoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

    //autoChooser.addOption("Drive Forward", new DriveForwardAuto());
    autoChooser.addOption("Track April Tag", new TrackAprilTagCommand(m_robotDrive));
    autoChooser.addOption("April Tag Track and Move Command", new AprilTagTrackAndMoveCommand(m_robotDrive));
    autoChooser.addOption("April Tag finder", new FindAprilTagCommand(m_robotDrive));
    autoChooser.addOption("Find and Track Tag Command", new FindAndTrackTagCommand(m_robotDrive));
    autoChooser.addOption("MoveToScoreDistance", new MoveToScoreDistance(m_robotDrive));

    SmartDashboard.putData("Auto/Auto Mode", autoChooser);
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
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband),
        () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband),
        () -> Constants.DriveConstants.kSlowSpeedMultiplier,
        () -> true));

    m_driverController.button(OIConstants.bumperLeft)
        .whileTrue(new DriveCommand(m_robotDrive, 
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband),
            () -> Constants.DriveConstants.kFastSpeedMultiplier,
            () -> true));

    // Reset odometry
    m_driverController.button(OIConstants.buttonY)
        .onTrue(m_robotDrive.runOnce(() -> m_robotDrive.resetHeading(90)));
    
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
        .onTrue(m_feeder.runOnce(() -> m_feeder.setVelocity(2000)))
        .onFalse(m_feeder.runOnce(() -> m_feeder.stop()));

    // Run intake
    m_manipulatorController.button(OIConstants.buttonX)
        .whileTrue(new RunIntakeInCommand(m_intake));

    
    m_intake.setDefaultCommand(new RunIntakePivotCommand(
        m_intake,
        () -> MathUtil.applyDeadband(m_manipulatorController.getRawAxis(OIConstants.rightStickY), 0.1)));
    
    /*
    // Extend Intake
    m_driverController.button(OIConstants.buttonY)
        .onTrue(m_intake.runOnce(() -> m_intake.setPivotPosition(IntakeConstants.kIntakeExtendedEncoderPosition)));

    // Retract Intake
    m_driverController.button(OIConstants.buttonA)
        .onTrue(m_intake.runOnce(() -> m_intake.setPivotPosition(IntakeConstants.kIntakeRetractedEncoderPosition)));

    // Run intake in
    m_driverController.button(OIConstants.bumperRight)
        .whileTrue(new RunIntakeInCommand(m_intake));
      */
    

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
    return pathAutoChooser.getSelected();
  }

}