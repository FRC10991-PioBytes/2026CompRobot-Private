// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.IntakeCommands.RunIntakeInCommand;
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
  // Drive subsystem
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
  //private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();

  // Driver controller
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

    //autoChooser.addOption("Drive Forward", new DriveForwardAuto());
    autoChooser.addOption("Track April Tag", new TrackAprilTagCommand(m_robotDrive));
    autoChooser.addOption("April Tag Track and Move Command", new AprilTagTrackAndMoveCommand(m_robotDrive));
    autoChooser.addOption("April Tag finder", new FindAprilTagCommand(m_robotDrive));
    autoChooser.addOption("Find and Track Tag Command", new FindAndTrackTagCommand(m_robotDrive));

    SmartDashboard.putData("Auto/Auto Mode", autoChooser);

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
        () -> MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband),
        () -> Constants.DriveConstants.kSlowSpeedMultiplier,
        () -> true));

    m_driverController.button(OIConstants.bumperLeft)
        .whileTrue(new DriveCommand(m_robotDrive, 
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband),
            () -> Constants.DriveConstants.kFastSpeedMultiplier,
            () -> true));

    /*
    // Find and track April tag
    m_driverController.button(OIConstants.buttonX)
        .whileTrue(new FindAndTrackTagCommand(m_robotDrive));
    */

    /*
    m_driverController.button(OIConstants.buttonA)
        .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    m_driverController.button(OIConstants.buttonB)
        .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));

    m_driverController.button(OIConstants.buttonX)
        .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    m_driverController.button(OIConstants.buttonY)
        .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */

    m_driverController.button(OIConstants.buttonA)
        .onTrue(m_shooter.runOnce(() -> m_shooter.setVelocity(4000)));

    m_driverController.button(OIConstants.buttonB)
        .onTrue(m_shooter.runOnce(m_shooter::stop));

    m_driverController.button(OIConstants.buttonX)
        .onTrue(m_feeder.runOnce(() -> m_feeder.setVelocity(2000)));

    m_driverController.button(OIConstants.buttonY)
        .onTrue(m_feeder.runOnce(m_feeder::stop));
    
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
    // Set X formation
    /*
    m_driverController.button(OIConstants.buttonB)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}