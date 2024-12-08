package com.donothinger.astar;

import java.util.List;

import com.donothinger.dto.Map;
import com.donothinger.dto.Node;
import com.donothinger.dto.Point;
import com.donothinger.heuristics.Heuristic;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class Astar {
    public Map taskMap;
    public Point startPoint;
    public Point goalPoint;
    public Node startNode;
    public Node goalNode;
    public Heuristic heuristic;
    public SearchTree searchTree;

    public Node run() {
        Integer steps = 0;
        searchTree.addToOpen(startNode);

        while (!searchTree.openIsEmpty()) {
            steps += 1;
            Node currentNode = searchTree.getBestNodeFromOpen();
            searchTree.addToClosed(currentNode);

            if (currentNode == goalNode) {
                System.out.println("During the search, the following number of OPEN dublicates was encountered: " +
                        String.valueOf(searchTree.getEncOpenDuplicates()));
                return currentNode;
            }

            List<Point> successors = taskMap.getSuccessors(currentNode.getPoint(), goalPoint);

            successors.stream()
                    .forEach(coord -> {
                        Node newNode = new Node(coord,
                                currentNode.getG() + this.taskMap.computeCost(currentNode.getPoint(), coord),
                                heuristic.calculate(coord, this.goalPoint));
                        if (!this.searchTree.wasOpened(newNode)) {
                            this.searchTree.addToOpen(newNode);
                        } else {
                            if (!searchTree.wasExpanded(newNode)) {
                                Node oldNode = this.searchTree.findNode(coord);
                                if (oldNode.getF() > newNode.getF()) {
                                    oldNode.setG(newNode.getG());
                                    oldNode.setF(newNode.getF());
                                    oldNode.setParent(newNode.getParent());
                                }
                            }
                        }
                    });

        }
        return null;
    }

    public Astar(Map taskMap, Point starPoint, Point goalPoint, Heuristic heuristic, SearchTree searchTree) {
        this.taskMap = taskMap;
        this.startPoint = starPoint;
        this.goalPoint = goalPoint;
        this.heuristic = heuristic;
        this.searchTree = searchTree;

        this.startNode = new Node(startPoint, 0.0, heuristic.calculate(starPoint, goalPoint));
        this.goalNode = new Node(goalPoint, 0.0, 0.0);
    }

    public Astar(Map taskMap, Node startNode, Node goalNode, Heuristic heuristic, SearchTree searchTree) {
        this.taskMap = taskMap;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.heuristic = heuristic;
        this.searchTree = searchTree;

        this.startPoint = startNode.getPoint();
        this.goalPoint = goalNode.getPoint();
    }

}
