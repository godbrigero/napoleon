package org.pwrup.napoleon.bridge;

import java.io.FileReader;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import lombok.Getter;

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
      int[][] defaultStaticObstacles) {
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

      this.sqSizeMeters = ((Number) json.get("square_size_meters")).floatValue();
      this.widthX = ((Number) json.get("grid_width_squares")).intValue();
      this.widthY = ((Number) json.get("grid_height_squares")).intValue();

      // this.centerXM = (double) ((JSONArray) json.get("center_position_meters")).get(0);
      // this.centerYM = (double) ((JSONArray) json.get("center_position_meters")).get(1);

      this.centerX = this.widthX / 2;
      this.centerY = this.widthY / 2;

      JSONArray walls = (JSONArray) json.get("walls");
      this.staticObstacles = new int[walls.size() * 2];

      for (int i = 0; i < walls.size(); i++) {
        JSONArray wall = (JSONArray) walls.get(i);
        this.staticObstacles[i * 2] = ((Number) wall.get(0)).intValue();
        this.staticObstacles[i * 2 + 1] = ((Number) wall.get(1)).intValue();
      }
    } catch (Exception e) {
      throw new RuntimeException(
          "Failed to load grid configuration from file: " + fileName,
          e);
    }
  }
}
