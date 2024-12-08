package com.donothinger.heuristics.impls;

import com.donothinger.dto.Point;
import com.donothinger.heuristics.Heuristic;

public class Euclidian implements Heuristic {
    @Override
    public Double calculate(Point point1, Point point2) {
        return Math.sqrt((point1.x - point2.x) * (point1.x - point2.x) +
                (point1.y - point2.y) * (point1.y - point2.y) +
                (point1.z - point2.z) * (point1.z - point2.z));
    }

}
