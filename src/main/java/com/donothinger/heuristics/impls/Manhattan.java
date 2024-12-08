package com.donothinger.heuristics.impls;

import com.donothinger.dto.Point;
import com.donothinger.heuristics.Heuristic;

public class Manhattan implements Heuristic {

    @Override
    public Double calculate(Point point1, Point point2) {
        return (double) (Math.abs(point1.x - point2.x) + Math.abs(point1.y - point2.y) + Math.abs(point1.z - point2.z));
    }

}
