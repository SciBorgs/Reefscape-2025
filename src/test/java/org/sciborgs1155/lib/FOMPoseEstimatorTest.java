package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class FOMPoseEstimatorTest {
  @Test
  public void newEstimate() {

    assertEquals(
        FOMPoseEstimator.newEstimate(
            new FOMPoseEstimator.Estimate(1, 0), new FOMPoseEstimator.Estimate(2, 3)),
        1);
  }
}
