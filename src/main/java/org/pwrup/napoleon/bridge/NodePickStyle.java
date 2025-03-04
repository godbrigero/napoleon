package org.pwrup.napoleon.bridge;

public enum NodePickStyle {
  ALL,
  SIDES;

  public int getValue() {
    return this == ALL ? 0 : 1;
  }
}
