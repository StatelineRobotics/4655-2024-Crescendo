package frc.robot.subsystems.Drive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavx implements GyroIO {
  public AHRS gyro;

  public GyroIONavx() {
    gyro = new AHRS(SPI.Port.kMXP);
  }

  public boolean isConnected() {
    return gyro.isConnected();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getYaw() {
    return gyro.getYaw() % 360;
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getPitchVelocity() {
    return gyro.getRawGyroX();
  }

  public double getYawVelocity() {
    return gyro.getRawGyroZ();
  }

  public double getRollVelocity() {
    return gyro.getRawGyroY();
  }

  public double getAccelerationX() {
    return gyro.getRawAccelX();
  }

  public double getAccelerationY() {
    return gyro.getRawAccelY();
  }

  public double getAccelerationZ() {
    return gyro.getRawAccelZ();
  }

  public void reset() {
    // Reset the gyro to 0 degrees
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                gyro.reset();
                Thread.sleep(1000);
                gyro.zeroYaw();
              } catch (Exception e) {
              }
            })
        .start();
  }

  public void updateInputs(GyroIOInputs inputs) {

    //inputs.connected = isConnected();
    inputs.yaw = Rotation2d.fromDegrees(getYaw());
    inputs.yawVelocity = Units.degreesToRadians(getYawVelocity());
  }
  ;
}
