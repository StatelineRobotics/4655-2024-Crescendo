// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.mechanisms.MechanismConstants;


public class ShooterSubsystem  extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean shoot = false;

    public ShooterSubsystem(ShooterIO io){
        System.out.println("[Init] Creating Shooter");
        this.io = io;

    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled()) {
            stop();
        } else if (shoot) {
            io.setShooterMotors(MechanismConstants.kTopShooterRPM, MechanismConstants.KBottomShooterRPM);
       }
    }

    private void stop() {
        shoot = false;
        io.stop();
      }
 
    

 
}
