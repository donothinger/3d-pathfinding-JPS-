package com.donothinger;

import com.donothinger.astar.Astar;
import com.donothinger.astar.searchTreeImpls.SearchTreePQD;
import com.donothinger.dto.Map;
import com.donothinger.dto.Point;
import com.donothinger.heuristics.impls.Euclidian;

public class App 
{
    public static void main( String[] args )
    {
        Map taskMap = new Map("maps/A1.3dmap");
        Point startPoint = new Point(103, 104, 191);
        Point goalPoint = new Point(376, 194, 115);
        Euclidian euclidianHeuristic = new Euclidian();
        SearchTreePQD searchTreePQD = new SearchTreePQD();
        Astar astar = new Astar(taskMap, startPoint, goalPoint, euclidianHeuristic, searchTreePQD);
        System.out.println(astar.run().getG());
    }
}
