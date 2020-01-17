/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

  public static int sag_on_motor_kodu = 0;
  public static int sag_arka_motor_kodu = 1;

  public static int sol_on_motor_kodu = 8;
  public static int sol_arka_motor_kodu = 9;
 
  public VictorSP sag_on_motor = new VictorSP(sag_on_motor_kodu);
  public VictorSP sag_arka_motor = new VictorSP(sag_arka_motor_kodu);

  public VictorSP sol_on_motor = new VictorSP(sol_on_motor_kodu);
  public VictorSP sol_arka_motor = new VictorSP(sol_arka_motor_kodu);
  public Joystick kumanda_1 = new Joystick(0);
  public Joystick kumanda_2 = new Joystick(1);

  private DifferentialDrive differentialDrive = new DifferentialDrive(sol_on_motor, sag_on_motor);
  private DifferentialDrive differentialDrive_2 = new DifferentialDrive(sol_arka_motor, sag_arka_motor);


  private I2C.Port i2cport=I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cport);

  private ColorMatch colorMatch = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    differentialDrive.setExpiration(1.0);
    differentialDrive.setRightSideInverted(false);

    
    differentialDrive_2.setExpiration(1.0);
    differentialDrive_2.setRightSideInverted(false);

    colorMatch.addColorMatch(kBlueTarget);
    colorMatch.addColorMatch(kGreenTarget);
    colorMatch.addColorMatch(kRedTarget);
    colorMatch.addColorMatch(kYellowTarget);  
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
    Color color = colorSensor.getColor();
    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
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
    motorGucunuSetEt(kumanda_2.getX(), kumanda_2.getY());
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

  public void motorGucunuSetEt(double xYonu, double yYonu){
    yYonu =motorYonDegerDogrulama(yYonu);
    xYonu = motorYonDegerDogrulama(xYonu);

    differentialDrive.setSafetyEnabled(true);
    differentialDrive.arcadeDrive(xYonu, -yYonu);

    differentialDrive_2.setSafetyEnabled(true);
    differentialDrive_2.arcadeDrive(xYonu, -yYonu);

    //robotDrive.mecanumDrive_Cartesian(kumanda_1.getX(), kumanda_1.getY(), kumanda_1.getTwist(), 0.5);
  }
}

