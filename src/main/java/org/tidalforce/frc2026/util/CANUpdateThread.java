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

package org.tidalforce.frc2026.util;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.Supplier;

public class CANUpdateThread implements AutoCloseable {
  // Executor for retrying config operations asynchronously
  private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
  private ThreadPoolExecutor threadPoolExecutor =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

  /**
   * Attempts a CTRE action up to 5 times until it succeeds.
   *
   * @param action The status-returning operation to retry.
   */
  @SuppressWarnings("FutureReturnValueIgnored")
  public void CTRECheckErrorAndRetry(Supplier<StatusCode> action) {
    threadPoolExecutor.submit(
        () -> {
          for (int i = 0; i < 5; i++) {
            StatusCode result = action.get();
            if (result.isOK()) {
              break;
            }
          }
        });
  }

  @Override
  public void close() {
    threadPoolExecutor.shutdownNow();
  }
}
