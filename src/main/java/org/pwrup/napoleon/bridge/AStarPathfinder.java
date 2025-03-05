package org.pwrup.napoleon.bridge;

public class AStarPathfinder {
  static {
    NativeLoader.init();
  }

  private HybridGrid hybridGrid;

  public AStarPathfinder(
      HybridGrid hybridGrid,
      NodePickStyle nodePickStyle,
      int maxNodesInRange,
      float finderRelativeW,
      float finderRelativeH) {
    this.hybridGrid = hybridGrid;
    this.initialize(
        hybridGrid.getStaticObstacles(),
        new int[] { hybridGrid.getWidthX(), hybridGrid.getWidthY() },
        new int[] { hybridGrid.getCenterX(), hybridGrid.getCenterY() },
        hybridGrid.getSqSizeMeters(),
        nodePickStyle.getValue(),
        maxNodesInRange,
        new float[] { finderRelativeW, finderRelativeH });
  }

  private native void initialize(
      int[] static_obstacles,
      int[] size_x_y,
      int[] center_x_y,
      float square_size_meters,
      int node_pick_style,
      int max_nodes_in_range,
      float[] finder_relative_w_h);

  public native int[] calculate(int[] start_x_y, int[] end_x_y);

  public native void clearHybridObjects();

  public native void addHybridObjects(float[] objects);

  public native void clearUncertentyFields();

  public native void addUncertentyField(float[] center, float radius, float intensity);
}
