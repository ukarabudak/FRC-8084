/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {
  public static int sag_on_motor_kodu = 0;
  public static int sag_arka_motor_kodu = 1;

  public static int sol_on_motor_kodu = 8;
  public static int sol_arka_motor_kodu = 9;

  public static int toplama_motor_kodu = 2;
  public static int ana_kayis_motor_kodu = 3;
  public static int firlatma_motor_kodu = 4;
  public static int rediktor_motor_kodu = 5;
  public static int renk_bulma_motor_kodu = 6;
 
  public VictorSP sag_on_motor = new VictorSP(sag_on_motor_kodu);
  public VictorSP sag_arka_motor = new VictorSP(sag_arka_motor_kodu);
  public VictorSP sol_on_motor = new VictorSP(sol_on_motor_kodu);
  public VictorSP sol_arka_motor = new VictorSP(sol_arka_motor_kodu);

  public VictorSP ana_kayis_motoru = new VictorSP(ana_kayis_motor_kodu);
  public VictorSP firlatma_motoru = new VictorSP(firlatma_motor_kodu);
  public VictorSP toplama_motoru = new VictorSP(toplama_motor_kodu);
  public VictorSP rediktor_motor = new VictorSP(rediktor_motor_kodu);
  public VictorSP renk_bulma_motoru = new VictorSP(renk_bulma_motor_kodu);

  public Joystick kumanda_1 = new Joystick(0);
  public Joystick kumanda_2 = new Joystick(1);

  public SpeedControllerGroup sol_hiz_kontrol = new SpeedControllerGroup(sol_on_motor, sol_arka_motor);
  public SpeedControllerGroup sag_hiz_kontrol = new SpeedControllerGroup(sag_on_motor, sag_arka_motor);
  private DifferentialDrive differentialDrive = new DifferentialDrive(sol_hiz_kontrol, sag_hiz_kontrol);



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
    motorlariConfigureEt();

    colorMatch.addColorMatch(kBlueTarget);
    colorMatch.addColorMatch(kGreenTarget);
    colorMatch.addColorMatch(kRedTarget);
    colorMatch.addColorMatch(kYellowTarget);  

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    });
  
  }

  public void motorGuvenlikOzelliginiSetEt(boolean isSafety){
    rediktor_motor.setSafetyEnabled(isSafety);
    toplama_motoru.setSafetyEnabled(isSafety);
    ana_kayis_motoru.setSafetyEnabled(isSafety);
    firlatma_motoru.setSafetyEnabled(isSafety);
    

    sag_arka_motor.setSafetyEnabled(isSafety);
    sag_on_motor.setSafetyEnabled(isSafety);
    sol_arka_motor.setSafetyEnabled(isSafety);
    sol_on_motor.setSafetyEnabled(isSafety);

  }
  private void motorlariConfigureEt(){
    motorGuvenlikOzelliginiSetEt(true);
    toplama_motoru.enableDeadbandElimination(true);
    ana_kayis_motoru.enableDeadbandElimination(true);
    rediktor_motor.enableDeadbandElimination(true);
    firlatma_motoru.enableDeadbandElimination(true);
   // renk_bulma_motoru.enableDeadbandElimination(true);

    sag_arka_motor.enableDeadbandElimination(true);
    sag_on_motor.enableDeadbandElimination(true);
    sol_arka_motor.enableDeadbandElimination(true);
    sol_on_motor.enableDeadbandElimination(true);

    differentialDrive.setSafetyEnabled(true);
    differentialDrive.setExpiration(1.0);
    differentialDrive.setRightSideInverted(false);
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

  public boolean renkBulucu(Color arananRenk){
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
    
   // System.out.println("renk :" + match.color.blue + ", aranan renk : " + arananRenk.blue);
  
    if(match.color == arananRenk) {
      return true;
    } 
    return false;
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

   /**
    * sureler saniye cinsindendir.
    */
   private boolean ileri_gidis_tamamlandi = false;
   private boolean ikinci_gidis_tamamlandi = false;
   private boolean donus_tamamlandi = false;
   private boolean ikinci_donus_tamamlandi = false;
   private boolean firlatma_tamamlandi = false;
   private String donus_yonu = "SOL";
   private double baslangic_zamani;
   private boolean otonom_basladi = false;

   public void otonomParametreleriniSifirla(){
    ileri_gidis_tamamlandi = false;
    ikinci_gidis_tamamlandi = false;
    donus_tamamlandi = false;
    ikinci_donus_tamamlandi = false;
    firlatma_tamamlandi = false;
    baslangic_zamani = 0.0;
    otonom_basladi = false;
   }
   
  @Override
  public void disabledInit() {
    if(otonom_basladi == true){
      otonomParametreleriniSifirla();
    }

  }

  @Override
  public void autonomousInit() {
    System.out.println("otonom sürüs basliyor.");
    motorGuvenlikOzelliginiSetEt(false);
    differentialDrive.setSafetyEnabled(false);
    baslangic_zamani = Timer.getFPGATimestamp();

    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

  }

   public boolean otonomIleriGit(double calismaSuresi, double hiz){
    double zaman = Timer.getFPGATimestamp();
    if(zaman - baslangic_zamani < calismaSuresi){
      differentialDrive.tankDrive(hiz, -hiz);
      System.out.println("robot ileri gidiyor");
    } else {
      differentialDrive.tankDrive(0.0, 0.0);
      baslangic_zamani = Timer.getFPGATimestamp();
      System.out.println("İleri gidis tamamlandı.");
      return true;
    }

    return false;

   }

   public boolean otonomSagaDon(double calismaSuresi){
      double zaman = Timer.getFPGATimestamp();
      if(zaman - baslangic_zamani < calismaSuresi){
        differentialDrive.tankDrive(0.4, 0.4);
        System.out.println("robot saga gidiyor");
      } else {
        differentialDrive.tankDrive(0.0, 0.0);
        baslangic_zamani = Timer.getFPGATimestamp();
        System.out.println("saga donusu tamamlandı.");
        return true;
      }

      return false;
   }

   public boolean otonomSolaDon(double calismaSuresi) {
      double zaman = Timer.getFPGATimestamp();
      if(zaman - baslangic_zamani < calismaSuresi){
        differentialDrive.tankDrive(-0.4, -0.4);
        System.out.println("robot sola gidiyor");
      } else {
        differentialDrive.tankDrive(0.0, 0.0);
        baslangic_zamani = Timer.getFPGATimestamp();
        System.out.println("sola donusu tamamlandı.");
        return true;
      }

      return false;
  }

  public boolean otonomTopFirlat(double calismaSuresi) {
    double zaman = Timer.getFPGATimestamp();
    if(zaman - baslangic_zamani < calismaSuresi){
      firlatma_motoru.setSafetyEnabled(false);
      firlatma_motoru.set(1.0);
      System.out.println("firlatma basladi");
    } else {
      System.out.println("Firlatma tamamlandı");
      firlatma_motoru.set(0.0);
      return true;
   }
   return false;
  }

  /**
   * This function is called periodically during autonomous.
   */

   
  @Override
  public void autonomousPeriodic() {
     otonom_basladi =true;
     if(ileri_gidis_tamamlandi == false){
      boolean  durum = otonomIleriGit(5.0,1.0);
      if(durum == true){
        ileri_gidis_tamamlandi = true;
      }
     }

    if(ileri_gidis_tamamlandi == true){
      if(donus_yonu.equals("SAG")){
        if(donus_tamamlandi == false){
          boolean  donus_durumu = otonomSagaDon(1.5);
          if(donus_durumu == true){
              donus_tamamlandi = true;
          }
        } else {
          if(ikinci_gidis_tamamlandi == false){
            boolean  durum = otonomIleriGit(2.0, 1.0);
            if(durum == true){
              ikinci_gidis_tamamlandi = true;
            }
          } else {
            if(ikinci_donus_tamamlandi == false){
              boolean ikinci_donus_durumu = otonomSolaDon(1.3);
              if(ikinci_donus_durumu == true){
                ikinci_donus_tamamlandi = true;
              }
            } 
          }
          
        }
        
      } else if (donus_yonu.equals("SOL")) {
        if(donus_tamamlandi == false){
          boolean  donus_durumu = otonomSolaDon(1.0);
          if(donus_durumu == true){
              donus_tamamlandi = true;
          }
        } else {
          if(ikinci_gidis_tamamlandi == false){
            boolean  durum = otonomIleriGit(2.0, 1.0);
            if(durum == true){
              ikinci_gidis_tamamlandi = true;
            }
          } else {
            if(ikinci_donus_tamamlandi == false){
              boolean ikinci_donus_durumu = otonomSagaDon(1.3);
              if(ikinci_donus_durumu == true){
                ikinci_donus_tamamlandi = true;
              }
            }
          }
          
        }
      } else {
        donus_tamamlandi = true;
        ikinci_donus_tamamlandi = true;
      }

    if(donus_tamamlandi == true && ikinci_donus_tamamlandi == true){
      if(firlatma_tamamlandi == false){
        boolean  durum = otonomTopFirlat(7.0);
        if(durum == true){
          firlatma_tamamlandi = true;
        }
       } else {
         motorlariConfigureEt();
       }
      
    }
  }
}

private boolean renk_bulundu = false;
private double renk_motor_guc_degeri = 0.2;
private double renk_motor_volgate_degeri = 2.4;
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    motorGucunuSetEt(kumanda_1.getX(), kumanda_1.getY());

    if(kumanda_1.getRawButton(11) == true){
      renk_bulma_motoru.set(renk_motor_guc_degeri);
      renk_bulma_motoru.setVoltage(renk_motor_volgate_degeri);
    } else if(kumanda_1.getRawButtonReleased(11) == true){
      renk_bulma_motoru.set(0.0);
      renk_bulma_motoru.setVoltage(0.0);
    }

    if(kumanda_1.getRawButton(10) == true && renk_bulundu == false ){
      System.out.println("Mavi Renk Aranıyor");
      renk_bulma_motoru.set(renk_motor_guc_degeri);
      renk_bulma_motoru.setVoltage(renk_motor_volgate_degeri);
      boolean durum = renkBulucu(kBlueTarget);
      if(durum == true){
        renk_bulundu = true;
        System.out.println("Mavi Renk Bulundu");
        renk_bulma_motoru.set(0.0);
        renk_bulma_motoru.setVoltage(0.0);
      
      } 
    } else if(kumanda_1.getRawButtonReleased(10) == true){
      System.out.println("renk motor sıfır");
      renk_bulma_motoru.set(0.0);
      renk_bulma_motoru.setVoltage(0.0);
      renk_bulundu = false;

    }

    if(kumanda_1.getRawButton(9) == true && renk_bulundu == false ){
      System.out.println("Sari Renk Aranıyor");
      renk_bulma_motoru.set(renk_motor_guc_degeri);
      renk_bulma_motoru.setVoltage(renk_motor_volgate_degeri);
      boolean durum = renkBulucu(kYellowTarget);
      if(durum == true){
        renk_bulundu = true;
        System.out.println("Sari Renk Bulundu");
        renk_bulma_motoru.set(0.0);
        renk_bulma_motoru.setVoltage(0.0);
      
      }
    } else if(kumanda_1.getRawButtonReleased(9) == true && renk_bulundu == false ){
      System.out.println("renk motor sıfır");
      renk_bulma_motoru.set(0.0);
      renk_bulma_motoru.setVoltage(0.0);
      renk_bulundu = false;
    }

    if(kumanda_1.getRawButton(8) == true && renk_bulundu == false ){
      System.out.println("Kırmızı Renk Araniyor");
      renk_bulma_motoru.set(renk_motor_guc_degeri);
      renk_bulma_motoru.setVoltage(renk_motor_volgate_degeri);
      boolean durum = renkBulucu(kRedTarget);
      if(durum == true){
        renk_bulundu = true;
        System.out.println("Kırmızı Renk Bulundu");
        renk_bulma_motoru.set(0.0);
        renk_bulma_motoru.setVoltage(0.0);
      
      }
    } else if(kumanda_1.getRawButtonReleased(8) == true){
      System.out.println("renk motor sıfır");
      renk_bulma_motoru.set(0.0);
      renk_bulma_motoru.setVoltage(0.0);
      renk_bulundu = false;
    }

    if(kumanda_1.getRawButton(7) == true && renk_bulundu == false ){
      System.out.println("Yesil Renk aranıyor");
      renk_bulma_motoru.set(renk_motor_guc_degeri);
      renk_bulma_motoru.setVoltage(renk_motor_volgate_degeri);
      boolean durum = renkBulucu(kGreenTarget);
      if(durum == true){
        renk_bulundu = true;
        System.out.println("Yesil Renk Bulundu");
        renk_bulma_motoru.set(0.0);
        renk_bulma_motoru.setVoltage(0.0);
      
      }
    }else if(kumanda_1.getRawButtonReleased(7) == true){
      System.out.println("renk motor sıfır");
      renk_bulma_motoru.set(0.0);
      renk_bulma_motoru.setVoltage(0.0);
      renk_bulundu = false;
    }
    
    if(kumanda_2.getRawButton(4) == true){
      //System.out.println("ana kayis motoru dongusu. Button : 2");
      anaKayisMotorunaGucVer(0.6);
    } else if(kumanda_2.getRawButton(2) == true){
      anaKayisMotorunaGucVer(-0.6);
    } else {
      //System.out.println("ana kayis motoru dongusu. Button : 4");
      anaKayisMotorunaGucVer(0.0);
    }
    
    if(kumanda_2.getRawButton(6) == true){
      //System.out.println("firlatma motoru dongusu. Button : 3");
      firlatmaMotorunaGucVer(1.0);
    } else if (kumanda_2.getRawButton(3) == true){
      firlatmaMotorunaGucVer(-1.0);
    } else {
     // System.out.println("firlatma motoru dongusu. Button : 1");
      firlatmaMotorunaGucVer(0.0);
    }

    if(kumanda_1.getRawButton(5) == true){
      //System.out.println("rediktor motoru dongusu. Button : 5");
      rediktorMotorunaGucVer(2.0);
    } else if(kumanda_1.getRawButton(3) == true){
      rediktorMotorunaGucVer(-2.0);
    } else {
      //System.out.println("rediktor motoru dongusu. Button : 3");
      rediktorMotorunaGucVer(0.0);
    }

    if(kumanda_2.getRawButton(1) == true){
      //System.out.println("toplama motoru dongusu. Button : 5");
      toplamaMotorunaGucVer(1.0,8.0);
    } else if(kumanda_2.getRawButton(5) == true){
    //  System.out.println("toplama motoru dongusu. Button : 1");
      toplamaMotorunaGucVer(-1.0,-8.0);
    } else {
      toplamaMotorunaGucVer(0.0,0.0);
    }

  }


  /**
   * This function is called periodically during test mode.
   */ 
  @Override
  public void testPeriodic() {
  }
  
  public double motorYonDegerDogrulama(double gucDegeri){
    if (gucDegeri < -2) {
      gucDegeri = -2;
    } else if (gucDegeri > 2) {
      gucDegeri = 2;
    }
    return gucDegeri;
  }

  public void motorGucunuSetEt(double xYonu, double yYonu){
    yYonu = (motorYonDegerDogrulama(yYonu));
    xYonu = (motorYonDegerDogrulama(xYonu));

    if (yYonu != 0.0 || xYonu != 0.0 ) {
      //System.out.println("x yonu : " + xYonu + " y yonu : " + yYonu );
    }

    differentialDrive.arcadeDrive(xYonu, -yYonu);

  }

  public void anaKayisMotorunaGucVer(double motorGucu){
    ana_kayis_motoru.set(motorGucu);
    if(motorGucu >0.0){
      System.out.println("ana kayis motor hizi : " + ana_kayis_motoru.getSpeed());
    }
    
  }

  public void firlatmaMotorunaGucVer(double motorGucu){
    firlatma_motoru.set(motorGucu);
    if(motorGucu >0.0){
      System.out.println("firlatma motor hizi : " + firlatma_motoru.getSpeed());
    }
    
  }
  public void toplamaMotorunaGucVer(double motorGucu,double voltage){
    toplama_motoru.set(motorGucu);
    toplama_motoru.setVoltage(voltage);
    if(motorGucu >0.0){
      System.out.println("toplama motor hizi : " + toplama_motoru.getSpeed());
    }
    
  }

  public void rediktorMotorunaGucVer(double motorGucu){
    rediktor_motor.set(motorGucu);
    if(motorGucu >0.0){
      System.out.println("rediktor motor hizi : " + rediktor_motor.getSpeed());
    }
    
  }
}
