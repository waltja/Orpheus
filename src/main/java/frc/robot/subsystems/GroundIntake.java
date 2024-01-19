package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private static TalonFX intakeMotor;

  private static CANSparkMax intakePivot;
  private static SparkPIDController controller;
  private static double setpoint;
  private static SparkAbsoluteEncoder sparkencoder;

  public GroundIntake() {
    intakeMotor = new TalonFX(Constants.GroundIntake.INTAKE_MOTOR_ID);
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    intakeMotor.setSafetyEnabled(true);

    intakePivot = new CANSparkMax(Constants.GroundIntake.INTAKE_PIVOT_ID, MotorType.kBrushless);
    intakePivot.restoreFactoryDefaults();
    controller = intakePivot.getPIDController();
    sparkencoder = intakePivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    controller.setP(Constants.GroundIntake.p);
    controller.setI(Constants.GroundIntake.i);
    controller.setD(Constants.GroundIntake.d);

    intakePivot.getPIDController().setFeedbackDevice(sparkencoder);
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }
  
  public void outtake(double speed){
    intakeMotor.set(speed);
  }

  public void setRotation (double angle){
    setpoint = angle;
    controller.setReference(MathUtil.clamp(angle, Constants.GroundIntake.retractAngle, Constants.GroundIntake.deployAngle), CANSparkBase.ControlType.kPosition);
  }

  public void deploy(){
    setRotation(Constants.GroundIntake.deployAngle);
  }

  public void retract(){
    setRotation(Constants.GroundIntake.retractAngle);
  }

  public boolean pivotIsFinished(double tolerance){
    return Math.abs(setpoint - sparkencoder.getPosition()) < tolerance;
  }

  public void stop(){

    intakeMotor.stopMotor();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ground Intake Motor Position", sparkencoder.getPosition());
  }
}
