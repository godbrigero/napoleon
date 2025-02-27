package org.pwrup.napoleon;

public class HybridGrid {

  private final int sizeX, sizeY;
  private final int[][] staticDataPositions;

  public HybridGrid(int sizeX, int sizeY, int[][] staticDataPositions) {
    this.sizeX = sizeX;
    this.sizeY = sizeY;
    this.staticDataPositions = staticDataPositions;
  }

  public HybridGrid(String filePath) {
    this(0, 0, new int[0][0]);
  }
}
