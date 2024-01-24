// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

// add soft limits and also play with relative encoder stuff

public class AmpBar extends SubsystemBase {
  private static CANSparkMax AmpBarMotor;
  private static PIDController controller;
  private static SparkAbsoluteEncoder sparkencoder;
  private static RelativeEncoder relEnc;


  /** Creates a new Climbers. */
  public AmpBar() {
    // Instantiate a Spark MAX and get a handle to a PWM absolute encoder

    AmpBarMotor = new CANSparkMax(Constants.AmpBar.AMP_BAR_MOTOR_ID, MotorType.kBrushless);
    AmpBarMotor.restoreFactoryDefaults();
    AmpBarMotor.setIdleMode(IdleMode.kBrake);


    controller = new PIDController(
      Constants.AmpBar.p,
      Constants.AmpBar.i,
      Constants.AmpBar.d);

    sparkencoder = AmpBarMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    //sparkencoder.setPositionConversionFactor(360);
    controller.setTolerance(1);

    relEnc = AmpBarMotor.getEncoder();
    //sparkencoder.setPositionConversionFactor(20);
    relEnc.setPosition(sparkencoder.getPosition() *20);
    

    AmpBarMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    AmpBarMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    AmpBarMotor.setSoftLimit(SoftLimitDirection.kForward, 15);
    AmpBarMotor.setSoftLimit(SoftLimitDirection.kReverse,1);
  }

  public void setRotation (double angle){
    AmpBarMotor.set(controller.calculate(sparkencoder.getPosition() *20, angle));
  }

  public void manualRotate(double speed){
    AmpBarMotor.set(speed);
  }

  public void deploy(){
    setRotation(Constants.AmpBar.deployAngle);
  }

  public void retract(){
    setRotation(Constants.AmpBar.retractAngle);
  }

  public boolean isFinished(){
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Absolute Position", sparkencoder.getPosition() *20);
    // relative encoder testing stuff
    SmartDashboard.putNumber("Relative Position", relEnc.getPosition());
  }
}
