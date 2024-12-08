package com.donothinger.astar;

import com.donothinger.dto.Node;
import com.donothinger.dto.Point;

public interface SearchTree {
    public void addToOpen(Node node);

    public Boolean openIsEmpty();

    public Node getBestNodeFromOpen();

    public Boolean wasExpanded(Node node);

    public Boolean wasOpened(Node node);

    public void addToClosed(Node node);

    public Node findNode(Point point);

    public Integer getEncOpenDuplicates();

}
