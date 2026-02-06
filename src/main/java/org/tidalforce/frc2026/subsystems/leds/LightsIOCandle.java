// Team: FRC 1721 - Concord Robotics (Tidal Force)
// Year: 2025-2026
// Code: Public codebase for our REBUILT frc robot
// License: MIT License (See LICENSE file for full text)
//
// Copyright (c) 2025-2026 Concord Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN an ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package org.tidalforce.frc2026.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import lombok.Getter;
import org.tidalforce.frc2026.util.CANUpdateThread;
import org.tidalforce.frc2026.util.Device;

/** A lights implementation that uses a CANdle */
public class LightsIOCandle implements LightsIO {
  @Getter private final String name;

  private final CANdle candle;

  private final CANUpdateThread updateThread = new CANUpdateThread();

  /**
   * Constructs a {@link LightsIOCandle} object with the specified name, CAN id, and configuration.
   *
   * @param name A human-readable name for this sensor instance.
   * @param id The Device identifying the bus and device ID for this sensor.
   * @param config The CANrangeConfiguration to apply to the sensor upon initialization.
   */
  public LightsIOCandle(String name, Device.CAN id, CANdleConfiguration config) {
    this.name = name;

    candle = new CANdle(id.id(), new CANBus(id.bus()));

    updateThread.CTRECheckErrorAndRetry(() -> candle.getConfigurator().apply(config));
  }

  @Override
  public void setAnimation(ControlRequest request) {
    candle.setControl(request);
  }
}
