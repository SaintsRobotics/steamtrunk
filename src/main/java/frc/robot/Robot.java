/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with arcade steering.
 */
public class Robot extends TimedRobot {

  private final WPI_TalonSRX m_bottomRightDrive = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_topRightDrive = new WPI_TalonSRX(2);
  private final MotorControllerGroup m_rightDriveGroup = new MotorControllerGroup(m_bottomRightDrive, m_topRightDrive);

  private final WPI_TalonSRX m_kicker = new WPI_TalonSRX(3);
  private final AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(0, 72);
  private final PIDController m_PID = new PIDController(0.06, 0, 0);
  private KickerState m_kickerState = KickerState.WaitingToKick;

  private final WPI_TalonSRX m_bottomLeftDrive = new WPI_TalonSRX(4);
  private final WPI_TalonSRX m_topLeftDrive = new WPI_TalonSRX(5);
  private final MotorControllerGroup m_leftDriveGroup = new MotorControllerGroup(m_bottomLeftDrive, m_topLeftDrive);

  private final WPI_TalonSRX m_topClimber = new WPI_TalonSRX(6);
  private final WPI_TalonSRX m_bottomClimber = new WPI_TalonSRX(7);
  private final MotorControllerGroup m_climberGroup = new MotorControllerGroup(m_topClimber, m_bottomClimber);

  private final DifferentialDrive  m_robotDrive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);
  private final XboxController m_stick = new XboxController(0);

  @Override
  public void teleopInit() {

    m_PID.setTolerance(1);
    m_PID.setSetpoint(52);

  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(m_stick.getRightX() * 0.4, -m_stick.getLeftY() * 0.4);

    double leftTrigger = m_stick.getLeftTriggerAxis();
    double rightTrigger = m_stick.getRightTriggerAxis();
    boolean buttonX = m_stick.getXButton();
    SmartDashboard.putBoolean("X Button", buttonX);
    m_climberGroup.set(rightTrigger - leftTrigger); 

    switch (m_kickerState) {
    case WaitingToKick:
      if (buttonX) {
        m_kickerState = KickerState.Extending;
        m_PID.setSetpoint(37);
      }
      break;

    case Extending:
      if (m_PID.atSetpoint()) {
        m_kickerState = KickerState.Retracting;
        m_PID.setSetpoint(52);
      }
      break;

    case Retracting:
      if (m_PID.atSetpoint()) {
        m_kickerState = KickerState.WaitingToKick;
      }
      break;
    }

    SmartDashboard.putNumber("Potentiometer", m_potentiometer.get());

    double PIDOutput = -m_PID.calculate(m_potentiometer.get());
    SmartDashboard.putNumber("PID Output", PIDOutput);
    m_kicker.set(PIDOutput);

    SmartDashboard.putBoolean("At Setpoint", m_PID.atSetpoint());
  }
}