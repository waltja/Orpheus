// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class AmpBar extends SubsystemBase {
  CANSparkMax AmpBarMotor;
  SparkPIDController controller;
  Rotation2d setpoint = new Rotation2d();
  double abspos;
  SparkAbsoluteEncoder sparkencoder;


  /** Creates a new Climbers. */
  public AmpBar() {
    // Instantiate a Spark MAX and get a handle to a PWM absolute encoder
    AmpBarMotor = new CANSparkMax(Constants.AmpBar.AMP_BAR_MOTOR_ID, MotorType.kBrushless);
    controller = AmpBarMotor.getPIDController();
    sparkencoder = AmpBarMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    controller.setP(Constants.AmpBar.p);
    controller.setI(Constants.AmpBar.i);
    controller.setD(Constants.AmpBar.d);

    // Reads the PWM Canandcoder position in rotations
    abspos = sparkencoder.getPosition();

    AmpBarMotor.restoreFactoryDefaults();
    
    // configure the Spark Max to use the PWM-connected Canandcoder for 
    // closed-Loop control
    AmpBarMotor.getPIDController().setFeedbackDevice(sparkencoder);
  }

  public void setRotation (Rotation2d angle){
    setpoint = angle;
    controller.setReference(MathUtil.clamp(angle.getRadians(), Constants.AmpBar.retractAngle.getRadians(), Constants.AmpBar.deployAngle.getRadians()), ControlType.kPosition);
  }

  public void deploy(){
    setRotation(Constants.AmpBar.deployAngle);
  }

  public void retract(){
    setRotation(Constants.AmpBar.retractAngle);
  }

  public boolean isFinished(double tolerance){
    return Math.abs(setpoint.getRadians() - sparkencoder.getPosition()) < tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Amp Bar Motor Position", sparkencoder.getPosition());
  }
}
