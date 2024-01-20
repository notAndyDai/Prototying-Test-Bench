// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Orchestra orchestra;

  private RobotContainer m_robotContainer;

  private TalonFX motor1;
  private TalonFX motor2;

  private Slot0Configs motor1Configs;
  private Slot0Configs motor2Configs;

  private double maxRPS = 100;

  private VelocityVoltage request = new VelocityVoltage(0);

  private Joystick stick;

  private SlewRateLimiter limiter;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    motor1 = new TalonFX(1, "CANivore");
    motor2 = new TalonFX(2, "CANivore");

    motor1Configs = new Slot0Configs().withKV(0.12).withKP(0.05);
    motor2Configs = new Slot0Configs().withKV(0.12).withKP(0.05);

    motor2.setInverted(true);

    motor1.getConfigurator().apply(motor1Configs);
    motor2.getConfigurator().apply(motor2Configs);

    stick = new Joystick(0);

    orchestra = new Orchestra();
    orchestra.addInstrument(motor1);
    orchestra.addInstrument(motor2);

    orchestra.loadMusic("funnyCalifornia.chrp");

    // limiter = new SlewRateLimiter(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (stick.getRawButton(1)) {
      orchestra.play();
    } else {
      orchestra.pause();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    orchestra.loadMusic("funnyCalifornia.chrp");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double percent = 0;// = Math.signum(stick.getX()) * Math.abs(Math.pow(stick.getX(), 3));

    double reversed = (stick.getRawButton(2)) ? -1 : 1;
    if (stick.getPOV() == -1) {
      motor1.setControl(new VoltageOut(0));
      motor2.setControl(new VoltageOut(0));
    }

    else {
      // double slewPower = limiter.calculate(power);
      if (stick.getPOV() == 0) {
        percent = 0.95;
      }

      else if (stick.getPOV() == 90) {
        percent = 0.75;
      }

      else if (stick.getPOV() == 270) {
        percent = 0.6;
      }

      else if (stick.getPOV() == 180) {
        percent = 0.22222;
      }

      else {
        percent = 0;
      }

      motor1.setControl(request.withVelocity(maxRPS * percent));
      motor2.setControl(request.withVelocity(maxRPS * percent));
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
