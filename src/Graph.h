#pragma once

#include <vector>
#include <map>

// нумерация идёт с левого нижнего квадрата, если смотреть на полигон, как указано в правилах

struct Node {
	int x;
	int y;
};

bool operator==(const Node& a, const Node& b);
bool operator!=(const Node& a, const Node& b);
bool operator<(const Node& a, const Node& b);

enum NodeType {
	UNKNOWN,
	EMPTY,
	OBSTACLE
};

enum Direction {
	RIGHT = 0, // dx = 1
	DOWN = 1, // dy = -1
	LEFT = 2, // dx = -1
	UP = 3, // dy = 1
};

enum Action {
	FORWARD = 0,
	TURN_LEFT = 1,   // 90 grad
	TURN_RIGHT = 2,  // 90 grad
	TURN_AROUND = 3,  // 180 grad
};

const int COLS = 4;
const int ROWS = 3;
const int costForAction[] = { 2, 3, 3, 4 };
const int dx[] = {1, 0, -1, 0};
const int dy[] = {0, -1, 0, 1};

class Graph final {
public:
	Graph();
	virtual ~Graph() = default;

	NodeType getNodeType(const Node& node) const;
	void setNodeType(const Node& node, NodeType type);

	std::pair<std::vector<Action>, Direction> pathFromNodeToNode(Node fromNode, Direction fromDirection, Node toNode);

private:
	std::map<Node, NodeType> nodeTypes;
};
