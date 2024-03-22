package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
 
  private static CANSparkMax intakePivot;
  private static CANSparkMax intakePivotFollower;
  private static TalonFX intakeMotor;
  private static PIDController controller;
  private static SparkAbsoluteEncoder sparkencoder;
  private static RelativeEncoder relEnc;
  private static boolean ampAngle = false;


  public IntakeSubsystem() {
    intakePivot = new CANSparkMax(Constants.GroundIntake.INTAKE_PIVOT_ID, MotorType.kBrushless);
    intakePivotFollower = new CANSparkMax(Constants.GroundIntake.Intake_PIVOT_FOLLOWER_ID, MotorType.kBrushless);
    intakePivot.restoreFactoryDefaults();
    intakePivotFollower.restoreFactoryDefaults();
    intakePivot.setIdleMode(IdleMode.kBrake);
    intakePivotFollower.setIdleMode(IdleMode.kBrake);
    intakePivotFollower.follow(intakePivot, true);

    intakeMotor = new TalonFX(Constants.GroundIntake.INTAKE_MOTOR_ID);
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    intakeMotor.setSafetyEnabled(true);
  
    controller = new PIDController(
      Constants.GroundIntake.p,
      Constants.GroundIntake.i, 
      Constants.GroundIntake.d);
    controller.setTolerance(Constants.GroundIntake.tolerance);

    sparkencoder = intakePivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    relEnc = intakePivot.getEncoder();
    relEnc.setPosition(sparkencoder.getPosition() *20);

    intakePivot.enableSoftLimit(SoftLimitDirection.kForward, true);
    intakePivot.enableSoftLimit(SoftLimitDirection.kReverse, true);
    intakePivot.setSoftLimit(SoftLimitDirection.kForward,(float)((Constants.GroundIntake.retractAngle) /18)); //changed math for new ratio
    intakePivot.setSoftLimit(SoftLimitDirection.kReverse,(float) ((Constants.GroundIntake.deployAngle) /18));
  }

  /* IntakePivot Methods */
  public void setRotation(double angle) {
    intakePivot.set(controller.calculate(relEnc.getPosition()/20, angle));
  }

  public void manualRotate(double speed) {
    ampAngle = false;
    intakePivot.set(speed);
  }

  public void deploy() {
    ampAngle = false;
    setRotation(Constants.GroundIntake.deployAngle/360);
  }

  public void retract() {
    ampAngle = false;
    setRotation(Constants.GroundIntake.retractAngle/360);
  }

  public void ampPosition() {
    ampAngle = true;
    setRotation(Constants.GroundIntake.ampAngle/360);
    
  }

  public boolean pivotIsFinished() {
    return controller.atSetpoint();
  }

  public boolean isAtAmpAngle() {
    return ampAngle;
  }

  public void stopPivot() {
    intakePivot.stopMotor();
  }

  /* IntakeRoller Methods */
  public void intake(double speed){
    intakeMotor.set(speed);
  }
  
  public void outtake(double speed){
    intakeMotor.set(speed);
  }
  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ground Intake Motor Position", relEnc.getPosition() *18);
    SmartDashboard.putBoolean("amp angle", ampAngle);
  }
}
