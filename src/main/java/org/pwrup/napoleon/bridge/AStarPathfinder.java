package org.pwrup.napoleon.bridge;

public class AStarPathfinder implements AutoCloseable {
  static {
    NativeLoader.init();
  }

  private long nativePtr;
  private HybridGrid hybridGrid;

  public AStarPathfinder(
    HybridGrid hybridGrid,
    NodePickStyle nodePickStyle,
    float finderRelativeW,
    float finderRelativeH,
    boolean doAbsoluteDiscard,
    float avgDistanceMinDiscardThreshold,
    float avgDistanceCost
  ) {
    this.hybridGrid = hybridGrid;
    this.nativePtr =
      this.initialize(
          hybridGrid.getStaticObstacles(),
          new int[] { hybridGrid.getWidthX(), hybridGrid.getWidthY() },
          new int[] { hybridGrid.getCenterX(), hybridGrid.getCenterY() },
          hybridGrid.getSqSizeMeters(),
          nodePickStyle.getValue(),
          new float[] { finderRelativeW, finderRelativeH },
          doAbsoluteDiscard ? 1 : 0,
          avgDistanceMinDiscardThreshold,
          avgDistanceCost
        );
  }

  private native long initialize(
    int[] staticObstacles,
    int[] sizeXY,
    int[] centerXY,
    float squareSizeMeters,
    int nodePickStyle,
    float[] finderRelativeWH,
    int doAbsoluteDiscard,
    float avgDistanceMinDiscardThreshold,
    float avgDistanceCost
  );

  public native int[] calculate(int[] start_x_y, int[] end_x_y);

  public native void clearHybridObjects();

  public native void addHybridObjects(float[] objects);

  public native void clearUncertentyFields();

  public native void addUncertentyField(
    float[] center,
    float radius,
    float intensity
  );

  private native void cleanup();

  @Override
  public void close() {
    cleanup();
  }
}
