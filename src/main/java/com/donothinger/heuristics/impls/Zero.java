package com.donothinger.heuristics.impls;

import com.donothinger.dto.Point;
import com.donothinger.heuristics.Heuristic;

public class Zero implements Heuristic {
    @Override
    public Double calculate(Point point1, Point point2) {
        return 0.0;
    }

}
