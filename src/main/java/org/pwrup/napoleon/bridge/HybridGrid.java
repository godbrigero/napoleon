package org.pwrup.napoleon.bridge;

import java.io.FileReader;
import lombok.Getter;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

@Getter
public class HybridGrid {

  private final int widthX, widthY;
  private final int centerX, centerY;
  private final float sqSizeMeters;
  private int[] staticObstacles;

  public HybridGrid(
    int widthX,
    int widthY,
    int centerX,
    int centerY,
    float sqSizeMeters,
    int[][] defaultStaticObstacles
  ) {
    this.widthX = widthX;
    this.widthY = widthY;
    this.centerX = centerX;
    this.centerY = centerY;
    this.sqSizeMeters = sqSizeMeters;

    int[] list = new int[defaultStaticObstacles.length * 2];
    int count = 0;
    for (int i = 0; i < defaultStaticObstacles.length; i++) {
      list[count] = defaultStaticObstacles[i][0];
      list[count + 1] = defaultStaticObstacles[i][1];
      count += 2;
    }

    this.staticObstacles = list;
  }

  public HybridGrid(String fileName) {
    try {
      JSONParser parser = new JSONParser();
      JSONObject json = (JSONObject) parser.parse(new FileReader(fileName));

      this.sqSizeMeters =
        ((Number) json.get("square_size_meters")).floatValue();
      int gridSize = ((Number) json.get("grid_size_pixels")).intValue();
      this.widthX = gridSize;
      this.widthY = gridSize;

      this.centerX = this.widthX / 2;
      this.centerY = this.widthY / 2;

      JSONArray obstacles = (JSONArray) json.get("obstacles");
      this.staticObstacles = new int[obstacles.size() * 2];

      for (int i = 0; i < obstacles.size(); i++) {
        JSONObject obstacle = (JSONObject) obstacles.get(i);
        this.staticObstacles[i * 2] = ((Number) obstacle.get("x")).intValue();
        this.staticObstacles[i * 2 + 1] =
          ((Number) obstacle.get("y")).intValue();
      }
    } catch (Exception e) {
      throw new RuntimeException(
        "Failed to load grid configuration from file: " + fileName,
        e
      );
    }
  }
}
