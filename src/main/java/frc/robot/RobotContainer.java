package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoIntake;

import frc.robot.commands.Climbers.*;
import frc.robot.autos.AutoSpeakerShoot;

import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climb.LeftClimber;
import frc.robot.subsystems.Climb.RightClimber;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final CommandXboxController baseDriver = new CommandXboxController(0);
    private final CommandXboxController armDriver = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(baseDriver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(baseDriver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    Swerve s_Swerve = new Swerve();
    IntakeSubsystem intake = new IntakeSubsystem();
    ShooterSubsystem shooter = new ShooterSubsystem();
    LeftClimber leftClimber = new LeftClimber();
    RightClimber rightClimber = new RightClimber();
    Vision vision = new Vision();
    RevBlinkin blink = new RevBlinkin();

    /* Auto Chooser */
    SendableChooser<Command> autoChooser;

    public void registerCommands() {
        NamedCommands.registerCommand("shoot", new ShootIntoSpeaker(shooter));
        NamedCommands.registerCommand("intake down", new IntakeDownCommand(intake));
        NamedCommands.registerCommand("intake up", new IntakeUpCommand(intake));
        NamedCommands.registerCommand("intake", new AutoIntake(intake));
        NamedCommands.registerCommand("outtake", new OuttakeCommand(intake));
        NamedCommands.registerCommand("zero gyro", new InstantCommand(() -> s_Swerve.zeroHeading()));
        NamedCommands.registerCommand("SpeakerShoot", new AutoSpeakerShoot(shooter, intake));
    }


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        registerCommands();
        CameraServer.startAutomaticCapture();
    
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -baseDriver.getRawAxis(translationAxis), 
                () -> -baseDriver.getRawAxis(strafeAxis), 
                () -> -baseDriver.getRawAxis(rotationAxis), 
                () -> baseDriver.leftBumper().getAsBoolean()
            )
        );

        intake.setDefaultCommand(new ManualPivotIntake(intake, () -> armDriver.getLeftY()));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // groundIntake.setDefaultCommand(
        //     new ManualPivotIntake(
        //         groundIntake, 
        //        () -> armDriver.getRawAxis(translationAxis)));

        // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() ->{
        //     Pose2d currentPose = s_Swerve.getPose();

        //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        //     Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2,0)), new Rotation2d());
        //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);

        //     PathPlannerPath path = new PathPlannerPath(
        //     bezierPoints,
        //      new PathConstraints(3, 3, 2*Math.PI, 4*Math.PI),
        //      new GoalEndState(0, Rotation2d.fromDegrees(0))
        //     );
        //     path.preventFlipping = true;

        //     AutoBuilder.followPath(path).schedule();
        // }));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        baseDriver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        baseDriver.a().whileTrue(new TeleopSwerve(
                s_Swerve,
                () -> -baseDriver.getRawAxis(translationAxis),
                () -> -baseDriver.getRawAxis(strafeAxis),
                () -> vision.calculateOffset(),
                () -> baseDriver.leftBumper().getAsBoolean()
            ));

        baseDriver.leftBumper().whileTrue(new LeftClimberDown(leftClimber));
        baseDriver.leftTrigger(0.25).whileTrue(new LeftClimberUp(leftClimber));
        baseDriver.rightBumper().whileTrue(new RightClimberDown(rightClimber));
        baseDriver.rightTrigger(0.25).whileTrue(new RightClimberUp(rightClimber));
        
        // Operator Buttons 
        armDriver.leftTrigger(0.15).whileTrue(new ShootIntoSpeaker(shooter));

        armDriver.rightTrigger(.15).whileTrue(new IntakeCommand(intake));
        armDriver.rightBumper().whileTrue(new OuttakeCommand(intake));
        armDriver.leftBumper().whileTrue(new ReverseShooter(shooter));

        armDriver.y().onTrue(new IntakeDownCommand(intake));
        armDriver.b().onTrue(new AmpAngle(intake));
        armDriver.a().onTrue(new IntakeUpCommand(intake));
        armDriver.x().onTrue(FastIntake());
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }

    public Command FastIntake() {
        return new SequentialCommandGroup(
            new IntakeDownCommand(intake),
            new IntakeCommand(intake),
            new InstantCommand(() -> blink.isIntake()),
            new IntakeUpCommand(intake)
        );
    }
}