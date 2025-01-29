// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.*;
import frc.lib.util.LEDs.LightingEffect.Type;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  private AddressableLED leds;
  private int totalLength;
  private LEDZone[] lightingZones;

  /** Creates a new LEDs. */
  public LEDs() {
    leds = new AddressableLED(Constants.ledPort);
    lightingZones = new LEDZone[] {
      new LEDZone("shooter-right", 0, 20),
      new LEDZone("shooter-left", 100, 20),
      //new LEDZone("climb-right", 200, 100),
      //new LEDZone("climb-left", 300, 100)
    };

    {
      int sum = 0;
      for(LEDZone z : lightingZones){
        sum+=z.getLength();
      }
      totalLength = sum;
    }
    leds.setLength(totalLength);
  }

  public void add(String key, LightingEffect... effects){
    search(key).addEffect(effects);
  }
  public void set(String key, LightingEffect... effects){
    search(key).setEffect(effects);
  }
  public void clear(String key){
    search(key).clearEffects();
  }
  public void clear(){
    for(LEDZone z : lightingZones){
      z.clearEffects();
    }
  }
  public void remove(String key, int i){
    search(key).removeEffect(i);
  }
  public void remove(String key, LightingEffect... e){
    search(key).removeEffect(e);
  }

  public Command addEffects(String key, LightingEffect... effects){
    return new InstantCommand(()->add(key, effects));
  }
  public Command setEffects(String key, LightingEffect... effects){
    return new InstantCommand(()->set(key, effects));
  }
  public Command clearEffects(String key){
    return new InstantCommand(()->clear(key));
  }
  public Command clearEffects(){
    return new InstantCommand(()->clear());
  }
  public Command removeEffect(String key, int i){
    return new InstantCommand(()->remove(key, i));
  }
  public Command removeEffect(String key, LightingEffect... e){
    return new InstantCommand(()->remove(key, e));
  }

  public Command temporaryEffect(String key, LightingEffect... es){
    return new FunctionalCommand(
      ()->{add(key, es);}, 
      ()->{}, 
      (interrupted)->{remove(key, es);}, 
      ()->{return false;}
      );
  }

  private LEDZone search(String key){
    for(LEDZone z : lightingZones){
      if(z.getKey().equals(key)){
        return z;
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color[] pixels = new Color[totalLength];
    for(LEDZone z : lightingZones){
      z.update();
      Color[] zonePixels = z.getPixels();
      for(int i = 0; i < z.getLength(); i++){
        pixels[i+z.getLocation()] = zonePixels[i];
      }
    }

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(totalLength);
    for(int i = 0; i < totalLength; i++){
      buffer.setRGB(i, (int)pixels[i].red, (int)pixels[i].green, (int)pixels[i].blue);
    }
    leds.setData(buffer);
  }

  // Preset Effects
  public Command noteStoredConditional(){
    return new ParallelCommandGroup(
      addEffects("shooter-right", new ConditionalColorEffect(0, 5, ()->{return RobotContainer.s_Index.getEye(0);}, new Color(255, 170, 0), new Color(0,0,0), Type.status)),
      addEffects("shooter-left", new ConditionalColorEffect(0, 5, ()->{return RobotContainer.s_Index.getEye(0);}, new Color(255, 170, 0), new Color(0,0,0), Type.status))
    );
  }
  public Command intakeWaiting(){
    return new ParallelCommandGroup(
      temporaryEffect("shooter-right", new SolidColorEffect(0, 20, new Color(255,0,0), Type.cosmetic)),
      temporaryEffect("shooter-left", new SolidColorEffect(0, 20, new Color(255,0,0), Type.cosmetic))
    );
  }
  public Command shooterStatus(){
    return new ParallelCommandGroup(
      temporaryEffect("shooter-right", 
        new VisorEffect(9, 10, new Color(0, 255, 0), 7, 0, 8, ()->{return RobotContainer.s_Shooter.getVelocity()/RobotContainer.s_Shooter.getSetpoint();}, ()->{return RobotContainer.s_Shooter.getVelocity()/RobotContainer.s_Shooter.getSetpoint();}, Type.status),
        new ConditionalColorEffect(9, 10, ()->{return RobotContainer.s_Shooter.readyForNote();}, new Color(0,255,0), new Color(0,0,0), Type.status)),
      temporaryEffect("shooter-right", 
        new VisorEffect(9, 10, new Color(0, 255, 0), 7, 0, 8, ()->{return RobotContainer.s_Shooter.getVelocity()/RobotContainer.s_Shooter.getSetpoint();}, ()->{return RobotContainer.s_Shooter.getVelocity()/RobotContainer.s_Shooter.getSetpoint();}, Type.status),
        new ConditionalColorEffect(9, 10, ()->{return RobotContainer.s_Shooter.readyForNote();}, new Color(0,255,0), new Color(0,0,0), Type.status))
    );
  }
}
