/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;

public class Robot extends TimedRobot {
 
  public VictorSP sag_motor_1 = new VictorSP(0);
  public VictorSP sag_motor_2 = new VictorSP(1);
  public VictorSP sol_motor_3 = new VictorSP(8);
  public VictorSP sol_motor_4 = new VictorSP(9);
  public Joystick kumanda_1 = new Joystick(0);
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    motorGucunuSetEt(kumanda_1.getX(), kumanda_1.getY());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public double solaDonusuHesapla(double solaDonusDegeri){
    return 0.5 * solaDonusDegeri;
  }

  public double sagaDonusuHesapla(double sagaDonusDegeri){
    return 0.5 * sagaDonusDegeri;
  }

  public double motorYonDegerDogrulama(double gucDegeri){
    if (gucDegeri < -1) {
      gucDegeri = -1;
    } else if (gucDegeri > 1) {
      gucDegeri = 1;
    }
    return gucDegeri;
  }

  public void motorGucunuSetEt(double sol, double sag){
    System.out.println("kumanda sol degeri :" + sol);
    System.out.println("kumanda sag degeri :" + sag);
    sol = motorYonDegerDogrulama(sol);
    sag = motorYonDegerDogrulama(sag);

    sol_motor_3.set(sol);
    sol_motor_4.set(sol);

    sag_motor_1.set(sag);
    sag_motor_2.set(sag);
  }
}
