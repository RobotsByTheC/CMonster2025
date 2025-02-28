package frc.robot.math;

import java.util.ArrayList;

public class MovingAverage {
  private final ArrayList<Double> numbers = new ArrayList<>();
  private final int size;
  public MovingAverage(int size) {
    this.size = size;
  }
  public void addNumber(double num) {
    if (numbers.size() == size) {
      numbers.remove(0);
      numbers.add(num);
    } else {
      numbers.add(num);
    }
  }
  public double getAverage() {
    double sum = 0;
    for (double d : numbers) {
      sum+=d;
    }
    return sum/size;
  }
}