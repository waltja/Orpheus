package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private static TalonFX intakeMotor;

  private static CANSparkMax intakePivot;
  private static CANSparkMax intakePivotFollower;
  private static PIDController controller;
  private static SparkAbsoluteEncoder sparkencoder;
  private static RelativeEncoder relEnc;

  public GroundIntake() {
    intakeMotor = new TalonFX(Constants.GroundIntake.INTAKE_MOTOR_ID);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setSafetyEnabled(true);

    intakePivot = new CANSparkMax(Constants.GroundIntake.INTAKE_PIVOT_ID, MotorType.kBrushless);
    intakePivotFollower = new CANSparkMax(Constants.GroundIntake.Intake_PIVOT_FOLLOWER_ID, MotorType.kBrushless);
    intakePivot.restoreFactoryDefaults();
    intakePivotFollower.restoreFactoryDefaults();
    intakePivot.setIdleMode(IdleMode.kBrake);
    intakePivotFollower.setIdleMode(IdleMode.kBrake);
    intakePivotFollower.follow(intakePivot, true);
  
    controller = new PIDController(
      Constants.GroundIntake.p,
      Constants.GroundIntake.i, 
      Constants.GroundIntake.d);


      controller.setTolerance(.05);

    sparkencoder = intakePivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    relEnc = intakePivot.getEncoder();
    relEnc.setPosition(sparkencoder.getPosition() *60);

    
    intakePivot.enableSoftLimit(SoftLimitDirection.kForward, true);
    intakePivot.enableSoftLimit(SoftLimitDirection.kReverse, true);
    intakePivot.setSoftLimit(SoftLimitDirection.kForward,(float)(( Constants.AmpBar.deployAngle+10) /6));
    intakePivot.setSoftLimit(SoftLimitDirection.kReverse,(float) ((Constants.AmpBar.retractAngle-10) /6));
  }
  

  public void intake(double speed){
    intakeMotor.set(speed);
  }
  
  public void outtake(double speed){
    intakeMotor.set(speed);
  }

  public void setRotation (double angle){
    intakePivot.set(controller.calculate(relEnc.getPosition()/60, angle));
  }

  public void manualRotate(double speed){
    intakePivot.set(speed);
  }

  public void deploy(){
    setRotation(Constants.GroundIntake.deployAngle/360);
  }

  public void retract(){
    setRotation(Constants.GroundIntake.retractAngle/360);
  }

  public void ampPosition(){
    setRotation(Constants.GroundIntake.ampAngle/360);
  }

  public boolean pivotIsFinished(){
    return controller.atSetpoint();
  }

  public void stop(){

    intakeMotor.stopMotor();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ground Intake Motor Position", sparkencoder.getPosition());
  }
}
