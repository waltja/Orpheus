package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.AmpBar.*;
import frc.robot.commands.Climbers.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Button Labels */
    JoystickButton DA, DB, DX, DY, DLB, DRB, DRT, DLT, DM1, DM2;
    JoystickButton AA, AB, AX, AY, ALB, ARB, ALT, ART, AM1, AM2;

    /* Controllers */
    private final XboxController baseDriver = new XboxController(0);
    private final XboxController armDriver = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(baseDriver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(baseDriver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final AmpBar ampBar = new AmpBar();
    private final LeftClimber leftClimber = new LeftClimber();
    private final RightClimber rightClimber = new RightClimber();
    private final GroundIntake groundIntake = new GroundIntake();
    private final Shooter shooter = new Shooter();
  


    /* Commands */
    private final AmpBarIn ampBarIn;
    private final AmpBarOut ampBarOut;
    private final LeftClimberDown leftClimberDown;
    private final LeftClimberUp leftClimberUp;
    private final RightClimberDown rightClimberDown;
    private final RightClimberUp rightClimberUp;
    private final Intake intake;
    private final IntakeDown intakeDown;
    private final IntakeUp intakeUp;
    private final AmpAngle ampAngle;
    private final Outtake outtake;
    private final ShootIntoSpeaker shootIntoSpeaker;


    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -baseDriver.getRawAxis(translationAxis), 
                () -> -baseDriver.getRawAxis(strafeAxis), 
                () -> -baseDriver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        groundIntake.setDefaultCommand(
            new ManualPivotIntake(
                groundIntake, 
                () -> armDriver.getRawAxis(translationAxis)));
        
        ampBar.setDefaultCommand(
            new ManualAmpBar(
                ampBar, 
                () -> armDriver.getRawAxis(XboxController.Axis.kRightY.value))
        );

        ampBarIn = new AmpBarIn(ampBar, shooter);
        ampBarIn.addRequirements(ampBar);
        ampBarOut = new AmpBarOut(ampBar, shooter);
        ampBarOut.addRequirements(ampBar);
        leftClimberDown = new LeftClimberDown(leftClimber);
        leftClimberDown.addRequirements(leftClimber);
        leftClimberUp = new LeftClimberUp(leftClimber);
        leftClimberUp.addRequirements(leftClimber);
        rightClimberDown = new RightClimberDown(rightClimber);
        rightClimberDown.addRequirements(rightClimber);
        rightClimberUp = new RightClimberUp(rightClimber);
        rightClimberUp.addRequirements(rightClimber);
        intake = new Intake(groundIntake);
        intake.addRequirements(groundIntake);
        intakeDown = new IntakeDown(groundIntake);
        intakeDown.addRequirements(groundIntake);
        intakeUp = new IntakeUp(groundIntake);
        intakeUp.addRequirements(groundIntake);
        ampAngle = new AmpAngle(groundIntake);
        ampAngle.addRequirements(groundIntake);
        outtake = new Outtake(groundIntake);
        outtake.addRequirements(groundIntake);
        shootIntoSpeaker = new ShootIntoSpeaker(shooter);
        shootIntoSpeaker.addRequirements(shooter);

         // Declare Driver Controller Buttons
         DA = new JoystickButton(baseDriver, 1);
         DB = new JoystickButton(baseDriver, 2);
         DX = new JoystickButton(baseDriver, 3);
         DY = new JoystickButton(baseDriver, 4);
         DLB = new JoystickButton(baseDriver, 5);
         DRB = new JoystickButton(baseDriver, 6);
         DM1 = new JoystickButton(baseDriver, 7);
         DM2 = new JoystickButton(baseDriver, 8);
 
         // Declare Arm Controller Buttons
         AA = new JoystickButton(armDriver, 1);
         AB = new JoystickButton(armDriver, 2);
         AX = new JoystickButton(armDriver, 3);
         AY = new JoystickButton(armDriver, 4);
         ALB = new JoystickButton(armDriver, 5);
         ARB = new JoystickButton(armDriver, 6);
         AM1 = new JoystickButton(armDriver, 8);
         AM2 = new JoystickButton(armDriver, 10);

        NamedCommands.registerCommand("shoot", shootIntoSpeaker);
        NamedCommands.registerCommand("intake down", intakeDown);
        NamedCommands.registerCommand("intakeUp", intakeUp);
        NamedCommands.registerCommand("intake", intake);
        NamedCommands.registerCommand("outtake", outtake);
        NamedCommands.registerCommand("zero gyro", new InstantCommand(() -> s_Swerve.zeroHeading()));


        // Configure the button bindings
        configureButtonBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
/* 
        SmartDashboard.putData("On-the-fly path", Commands.runOnce(() ->{
            Pose2d currentPose = s_Swerve.getPose();

            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2,0)), new Rotation2d());
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);

            PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
             new PathConstraints(3, 3, 2*Math.PI, 4*Math.PI),
             new GoalEndState(0, Rotation2d.fromDegrees(0))
            );
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        }));
        */
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() { 
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        
        DLB.whileTrue(leftClimberDown);
        DLT.whileTrue(leftClimberUp);
        DRB.whileTrue(rightClimberDown);
        DRT.whileTrue(rightClimberUp);
        // Operator Buttons 
        ALT.whileTrue(shootIntoSpeaker);
        ALB.whileTrue(intake);
        
        ARB.whileTrue(outtake);
        
        AX.onTrue(ampAngle);
        AY.onTrue(intakeUp);
        AA.onTrue(intakeDown);
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
       //return new PathPlannerAuto("Leave Zone");
    }
}