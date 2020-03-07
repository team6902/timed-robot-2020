package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  SendableChooser<String> m_autoChooser = new SendableChooser<String>();

  /* PORTS */

  /* portas USB */
  int kPilotstickPort = 0;
  int kCopilotstickPort = 1;

  /* portas ROBORIO */
  int kMotorLeftPort = 8;
  int kMotorRightPort = 9;
  int kRedLinePort = 1;
  int kMiniCIMPort = 2;
  int kMotorArmPort = 3;
  int kClimbPort = 4;
  int kClimbPort2 = 5;
  int kServoPort = 7;

  /* BUTTONS m_pilotStick */
  int kTurnToZeroGraus = 2;
  int kGrabPowerCell = 1;
  int kTurnTo180Graus = 3;
  int kEstacionar = 4;
  int kIntakeButton = 6;
  int kIntakeReverseButton = 5;

  /* BUTTONS m_copilotStick */
  int kArmButtonUp = 2;// PID//
  int kArmButtonDown = 1;// PID//
  int kButtonMotorClimbActivated = 6;
  int kButtonMotorClimbReverse = 5;

  /* VELOCITY */
  double kIntakeRedlineVelocity = 0.4;
  double kClimbVelocity = 0.6;
  double kArmUpVelocity = -0.5;
  double kArmDownVelocity = 0.4;
  double kArmVelocity = .3;

  /* CONSTANTES */
  private static final int kInfraredPort = 3;
  static final double kHoldDistanceBottomPort = 1.9;
  static final double kHoldDistanceAutonomous = 1; /* ajustar */

  /* CONSTANTES PID INFRAVERMELHO */
  private static final double kInfraredP = 2.5;
  private static final double kInfraredI = 0.5;
  private static final double kInfraredD = 1.5;

  /* TIMER */
  static Timer m_autoTimer = new Timer();
  static Timer m_teleopTimer = new Timer();

  /* BOOLEAN COLLISION */
  boolean collisionDetected = false;

  /* PNEUMATIC */
  Compressor m_compressor = new Compressor();
  DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);

  /* CHASSI MOVEMENT- SPARKS-PID-DOUBLEROTAT */
  Spark m_chassiMotorsLeft = new Spark(kMotorLeftPort);
  Spark m_chassiMotorsRight = new Spark(kMotorRightPort);
  DifferentialDrive m_chassiDrive = new DifferentialDrive(m_chassiMotorsLeft, m_chassiMotorsRight);
  static final double kTurnP = .1;
  static final double kTurnI = .0;
  static final double kTurnD = .001;
  double setpoint = 0.0;
  final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);
  final PIDController m_pidTurnController = new PIDController(kTurnP, kTurnI, kTurnD);
  double zRotation = 0.0;

  /* MOVIMENTO PID INFRAVERMELHO */
  private final MedianFilter m_filter = new MedianFilter(15);
  private final AnalogInput m_infrared = new AnalogInput(kInfraredPort);
  private final PIDController m_pidInfraRedController = new PIDController(kInfraredP, kInfraredI, kInfraredD);

  /* MOTORS */

  /* INTAKE (1 victor) */
  VictorSP m_intakeWindowMotor = new VictorSP(kRedLinePort);
  VictorSP m_intakeRedlineMotor = new VictorSP(kMiniCIMPort);

  /* CLIMB (2 talons) */
  Talon m_climbMotor1 = new Talon(kClimbPort);
  Talon m_climbMotor2 = new Talon(kClimbPort2);
  SpeedControllerGroup m_climbMotors = new SpeedControllerGroup(m_climbMotor1, m_climbMotor2);

  /* ARM (1 victor) */
  VictorSP m_MotorArm = new VictorSP(kMotorArmPort);

  /* JOYSTICK */
  final Joystick m_pilotStick = new Joystick(kPilotstickPort);
  final Joystick m_copilotStick = new Joystick(kCopilotstickPort);
  final Joystick m_pilotStick1 = new Joystick(kPilotstickPort);
  final Joystick m_copilotStick1 = new Joystick(kCopilotstickPort);
  final Joystick m_copilotStick2 = new Joystick(kCopilotstickPort);
  final Joystick m_copilotStick3 = new Joystick(kCopilotstickPort);

  /* Collision detection */
  double lastWorldLinearAccelX;
  double lastWorldLinearAccelY;
  final static double kCollisionThreshold_DeltaG = 0.5f;
  
  static final double kPixyP = .01;
  static final double kPixyI = .0;
  static final double kPixyD = .001;
  final PIDController m_pidPixyController = new PIDController(kPixyP, kPixyI, kPixyD);

  static final double kPixyAreaP = .1;
  static final double kPixyAreaI = .0;
  static final double kPixyAreaD = .001;
  final PIDController m_pidPixyControllerArea = new PIDController(kPixyP, kPixyI, kPixyD);

  /* LIMITADOR DE VELOCIDADE */
  public static double limit(double velocity, double limit) {
    if (velocity > limit)
      return limit;
    if (velocity < -limit)
      return -limit;
    return velocity;
  }

  // AUTO 1 - INICIO NA FRENTE DA BOTTOM PORT
  void auto01() {
    // andar reto por 2 segundos
    if (m_autoTimer.get() < 2) {
      System.out.println("andando reto por dois segundos");
      m_pidTurnController.setSetpoint(0);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.6, limit(zRotation, 0.6));
    }

    // abaixar o braco totalmente
    if (m_autoTimer.get() > 6 && m_autoTimer.get() < 9) {
      System.out.println("abaixando o braco totalmente");
      m_MotorArm.set(.3);
    } else {
      m_MotorArm.set(kArmVelocity);
    }

    // // acionar pid do infravermelho pra estacionar
    if (m_autoTimer.get() > 2 && m_autoTimer.get() < 5) {
      System.out.println("estacionando");
      m_pidInfraRedController.setSetpoint(kHoldDistanceBottomPort);
      double pidOutput = m_pidInfraRedController.calculate(m_filter.calculate(m_infrared.getVoltage()));
      m_chassiDrive.arcadeDrive(-limit(pidOutput, 0.5), 0.0);
    }
    // // ativar intake
    if (m_autoTimer.get() > 9) {
      System.out.println("ativando intake");
      m_intakeWindowMotor.set(.3);
    }
    if (m_autoTimer.get() > 14) {
      System.out.println("desativando intake");
      m_intakeWindowMotor.set(0);
    }
  }
  //toDo ajustar PID infrared
 
  // AUTO 2 - INICIO NA FRENTE DA LOAD STATION
  void auto02() {
    // andar reto por 2 segundos
    if (m_autoTimer.get() < 2.38) {
      m_pidTurnController.setSetpoint(0);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.7, limit(zRotation, 0.6));
    }
    // virar 90 graus e andar reto por mais 2 segundos
    if (m_autoTimer.get() > 2.38 && m_autoTimer.get() < 4) {
      m_pidTurnController.setSetpoint(-90);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.6, limit(zRotation, 0.6));
    }

    // andar reto por 2 segundos
    if (m_autoTimer.get() > 4 && m_autoTimer.get() < 7) {
      m_pidTurnController.setSetpoint(-90);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.6, limit(zRotation, 0.6));
    }

    // abaixar o braco totalmente
    if (m_autoTimer.get() > 4 && m_autoTimer.get() < 11) {
      System.out.println("abaixando o braco totalmente");
      m_MotorArm.set(.3);
    } else {
      m_MotorArm.set(kArmVelocity);
    }

    // // acionar pid do infravermelho pra estacionar
    if (m_autoTimer.get() > 7 && m_autoTimer.get() < 9) {
      System.out.println("estacionando");
      m_pidInfraRedController.setSetpoint(kHoldDistanceBottomPort);
      double pidOutput = m_pidInfraRedController.calculate(m_filter.calculate(m_infrared.getVoltage()));
      m_chassiDrive.arcadeDrive(-limit(pidOutput, 0.5), 0.0);
    }
    // // ativar intake
    if (m_autoTimer.get() > 12) {
      System.out.println("ativando intake");
      m_intakeWindowMotor.set(.3);
    }
    if (m_autoTimer.get() > 14) {
      System.out.println("desativando intake");
      m_intakeWindowMotor.set(0);
    }
  }
// AUTO 3 - INICIO NA PAREDE DA BOTTOM PORT
  void auto03() {

    // andar reto por 4 segundos
    if (m_autoTimer.get() < 1.4) {
      m_pidTurnController.setSetpoint(0);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.7, limit(zRotation, 0.6));
    }
    // virar 90 graus e andar reto por mais 2 segundos
    if (m_autoTimer.get() > 1.5 && m_autoTimer.get() < 4) {
      m_pidTurnController.setSetpoint(90);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.6, limit(zRotation, 0.6));
    }
    if (m_autoTimer.get() > 4 && m_autoTimer.get() < 5) {
      m_pidTurnController.setSetpoint(90);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.6, limit(zRotation, 0.6));
    }
    // abaixar o braco totalmente
    if (m_autoTimer.get() > 2 && m_autoTimer.get() < 11) {
      System.out.println("abaixando o braco totalmente");
      m_MotorArm.set(.3);
    } else {
      m_MotorArm.set(kArmVelocity);
    }

    // // acionar pid do infravermelho pra estacionar
    if (m_autoTimer.get() > 5 && m_autoTimer.get() < 9) {
      System.out.println("estacionando");
      m_pidInfraRedController.setSetpoint(kHoldDistanceBottomPort);
      double pidOutput = m_pidInfraRedController.calculate(m_filter.calculate(m_infrared.getVoltage()));
      m_chassiDrive.arcadeDrive(-limit(pidOutput, 0.6), 0.0);
    }
    // // ativar intake
    if (m_autoTimer.get() > 10) {
      System.out.println("ativando intake");
      m_intakeWindowMotor.set(.3);
    }
    if (m_autoTimer.get() > 14) {
      System.out.println("desativando intake");
      m_intakeWindowMotor.set(0);
    }
  }

  @Override
  public void robotInit() {
    m_pidTurnController.enableContinuousInput(-180, 180);
    m_pilotStick.setXChannel(4);
    // m_compressor.start();
    m_pidTurnController.setTolerance(.05);
    CameraServer.getInstance().startAutomaticCapture();
    m_pilotStick.setThrottleChannel(3);
    m_pilotStick1.setThrottleChannel(2);
    m_copilotStick.setThrottleChannel(2);
    m_copilotStick1.setThrottleChannel(3);
    m_copilotStick2.setYChannel(1);
    m_copilotStick3.setXChannel(4);
    m_autoChooser.addOption("Autonomo 1", "Auto01");
    m_autoChooser.addOption("Autonomo 2", "Auto02");
    m_autoChooser.addOption("Autonomo 3", "Auto03");
    SmartDashboard.putData(m_autoChooser);
    m_pidInfraRedController.setTolerance(0.05);
  }

  @Override
  public void autonomousInit() {
    m_autoTimer.start();
    m_gyro.reset();
  }

  @Override
  public void autonomousPeriodic() {
    if (m_autoChooser.getSelected().equals("Auto01")) {
      auto01();
    }
    if (m_autoChooser.getSelected().equals("Auto02")) {
      auto02();
    }
    if (m_autoChooser.getSelected().equals("Auto03")) {
      auto03();
    }
  }

  @Override
  public void teleopInit() {
    m_teleopTimer.start();
    m_pidTurnController.setTolerance(.08);
  }

  void listenSetpointButtons() {
    if (m_pilotStick.getRawButton(8)) {
      m_gyro.reset();
    } else if (m_pilotStick.getPOV() != -1) {
      setpoint = m_pilotStick.getPOV();
      if (setpoint == 270)
        setpoint = -90;
    }
    m_pidTurnController.setSetpoint(setpoint);
  }

  void grabPowerCell() {
    // ToDo: aumentar ganho e retirar limite do PID, evitar colisoes e ativar intake
    NetworkTable table = tableInstance.getTable("SmartDashboard");
    double x = table.getEntry("x").getDouble(0);
    double y = table.getEntry("y").getDouble(0);
    double width = table.getEntry("width").getDouble(0);
    double height = table.getEntry("height").getDouble(0);
    System.out.println(x + " " + y);
    // double centerX = x + width/2.0;
    double centerX = x - 100;
    double area = width * height;

    double pidOutputX = m_pidPixyController.calculate(centerX);
    double pidOutputArea = m_pidPixyControllerArea.calculate(area);
    m_chassiDrive.arcadeDrive(-limit(pidOutputArea,0.7), limit(-pidOutputX, 0.7));
    System.out.println(centerX + " " + pidOutputX);
    System.out.println(area + " " + pidOutputArea);
  }

  void listenChassiMovementButtons() {
    if (m_pilotStick.getPOV() != -1) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(0.7, limit(zRotation, 0.6));
    } else {
      m_chassiDrive.arcadeDrive(m_pilotStick.getY(), m_pilotStick.getX(), true);
      zRotation = 0;
    }

    if (m_copilotStick2.getY() != 0 || m_copilotStick3.getX() != 0) {
      m_chassiDrive.arcadeDrive(limit(m_copilotStick2.getY(), 0.6), limit(m_copilotStick3.getX(), 0.6), true);
    }
  }

  void listenAditionalMovements() {
    System.out.println(m_pidTurnController.getSetpoint());
    if (m_pilotStick.getRawButton(kGrabPowerCell)) {
      grabPowerCell();
    }
    if (m_pilotStick.getRawButton(kTurnToZeroGraus)) {
      m_pidTurnController.setSetpoint(0);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(0, limit(zRotation, 0.7));
    }
    if (m_pilotStick.getRawButton(kTurnTo180Graus)) {
      m_pidTurnController.setSetpoint(180);
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(0, limit(zRotation, 0.7));
    }
    if (m_pilotStick.getRawButton(kEstacionar)) {
      m_pidInfraRedController.setSetpoint(kHoldDistanceBottomPort);
      double pidOutput = m_pidInfraRedController.calculate(m_filter.calculate(m_infrared.getVoltage()));
      double pidTurnOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      m_chassiDrive.arcadeDrive(-limit(pidOutput, 0.6), limit(pidTurnOutput, 0.7));
    }
  }

  void listenIntakeButtons() {
    if (m_pilotStick.getRawButton(kIntakeButton)) {
      m_intakeRedlineMotor.set(kIntakeRedlineVelocity);
    } else if (m_pilotStick.getRawButton(kIntakeReverseButton)) {
      m_intakeRedlineMotor.set(-kIntakeRedlineVelocity);
    } else
      m_intakeRedlineMotor.set(0);

    if (m_pilotStick.getThrottle() > 0) {
      m_intakeWindowMotor.set(-m_pilotStick.getThrottle());
    } else {
      m_intakeWindowMotor.set(limit(m_pilotStick1.getThrottle(), 0.8));
    }
  }

  void listenArmMovements() {
    if (m_copilotStick.getRawButton(kArmButtonUp)) {
      m_MotorArm.set(kArmUpVelocity);
    } else if (m_copilotStick.getRawButton(kArmButtonDown)) {
      m_MotorArm.set(kArmDownVelocity);
    } else {
      m_MotorArm.set(kArmVelocity);
    }
  }

  void listenClimbButton() {
    if (m_copilotStick.getRawButton(kButtonMotorClimbActivated)) {
      m_climbMotors.set(-kClimbVelocity);
    } else if (m_copilotStick.getRawButton(kButtonMotorClimbReverse)) {
      m_climbMotors.set(kClimbVelocity);
    } else {
      m_climbMotors.set(0);
    }
    if (m_copilotStick1.getThrottle() > 0) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
    if (m_copilotStick.getThrottle() > 0) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

  }

  public void detectCollision() {

    double curr_world_linear_accel_x = m_gyro.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - lastWorldLinearAccelX;
    lastWorldLinearAccelX = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = m_gyro.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - lastWorldLinearAccelY;
    lastWorldLinearAccelY = curr_world_linear_accel_y;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
        || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
      collisionDetected = true;
    }
    SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
  }

  @Override
  public void teleopPeriodic() {
    listenSetpointButtons();

    listenChassiMovementButtons();

    listenIntakeButtons();

    listenArmMovements();

    listenClimbButton();

    listenAditionalMovements();

    // log();

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Angle", m_gyro.pidGet());
    SmartDashboard.putNumber("Turning Value", zRotation);
    SmartDashboard.putNumber("SetPoint", setpoint);
    SmartDashboard.putNumber("Distance", m_infrared.getVoltage());
    NetworkTable table = tableInstance.getTable("SmartDashboard");
    System.out.println(table.getEntry("sig").getDouble(0.0));
    SmartDashboard.putBoolean("Power Cell", table.getEntry("sig").getDouble(0.0) == 1.0);
  }

  void log() {
    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%b\n", Robot.m_teleopTimer.get(), this.m_pidTurnController.getP(),
        this.m_pidTurnController.getI(), this.m_pidTurnController.getD(), this.m_gyro.pidGet(), this.zRotation,
        this.m_pidTurnController.getSetpoint(), this.m_pidTurnController.atSetpoint());
  }

}
