package com.donothinger.astar.searchTreeImpls;

import java.util.Comparator;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

import com.donothinger.astar.SearchTree;
import com.donothinger.dto.Node;
import com.donothinger.dto.Point;

import lombok.Data;

@Data
public class SearchTreePQD implements SearchTree{
    private PriorityQueue<Node> openList = new PriorityQueue<>(Comparator.comparingDouble(Node::getF));
    private Set<Node> openedSet = new HashSet<Node>();
    private Set<Node> closedSet = new HashSet<Node>();
    private Integer encOpenDuplicates = 0;

    @Override
    public Boolean openIsEmpty() {
        return openList.isEmpty();
    }

    @Override
    public void addToOpen(Node node) {
        for (Node existingNode : openList) {
            if (existingNode.equals(node)) {
                if (existingNode.getG() > node.getG()) {
                    existingNode.setG(node.getG());
                    existingNode.setF(node.getF());
                    existingNode.setParent(node.getParent());
                }
                return;
            }
        }

        openList.add(node);
    }

    @Override
    public Node getBestNodeFromOpen() {
        if (this.openIsEmpty()) {
            return null;
        }
        Node bestNode = openList.remove();
        if (this.wasExpanded(bestNode)) {
            this.encOpenDuplicates += 1;
            return this.getBestNodeFromOpen();
        }

        return bestNode;
    }

    @Override
    public Boolean wasExpanded(Node node) {
        return closedSet.contains(node);
    }

    @Override
    public Boolean wasOpened(Node node) {
        return openedSet.contains(node);
    }

    @Override
    public void addToClosed(Node node) {
        closedSet.add(node);
    }

    @Override
    public Node findNode(Point point) {
        for (Node node : openList) {
            if (point.equals(node.getPoint())) {
                return node;
            }
        }
        throw new IllegalArgumentException();
    }
}
