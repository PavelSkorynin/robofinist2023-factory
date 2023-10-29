#include "Graph.h"

#include <set>
#include <algorithm>
#include <fstream>
#include <sstream>

inline int toIndex(const Node& node, Direction dir) {
	return dir * COLS * ROWS + node.y * COLS + node.x;
}

inline std::pair<Node, Direction> fromIndex(int index) {
	return std::make_pair<Node, Direction>({index % COLS, (index / COLS) % ROWS}, Direction(index / COLS / ROWS));
}

//
// VisitedNode
//
struct VisitedNode {
	Node node;
	Direction direction;
	int cost;
	Action action;

	inline int index() const {
		return toIndex(node, direction);
	}

	inline std::optional<VisitedNode> forward() const {
		if ((direction == LEFT && node.x <= 0)
				|| (direction == RIGHT && node.x >= COLS - 1)
				|| (direction == DOWN && node.y <= 0)
				|| (direction == UP && node.y >= ROWS - 1)) {
			return std::nullopt;
		}
		VisitedNode v = {{node.x + dx[direction], node.y + dy[direction]}, direction, cost + costForAction[FORWARD], FORWARD};
		return std::make_optional<VisitedNode>(std::move(v));
	}

	inline VisitedNode turnLeft() const {
		return {Node(node), Direction((direction + 3) % 4), cost + costForAction[TURN_LEFT], TURN_LEFT};
	}

	inline VisitedNode turnRight() const {
		return {Node(node), Direction((direction + 1) % 4), cost + costForAction[TURN_RIGHT], TURN_RIGHT};
	}

	inline VisitedNode turnAround() const {
		return {Node(node), Direction((direction + 2) % 4), cost + costForAction[TURN_AROUND], TURN_AROUND};
	}
};

bool operator<(const VisitedNode &a, const VisitedNode &b) {
	if (a.cost < b.cost) {
		return true;
	}
	if (b.cost < a.cost) {
		return false;
	}
	if (a.direction < b.direction) {
		return true;
	}
	if (b.direction < a.direction) {
		return false;
	}
	return a.node < b.node;
}

bool operator==(const Node& a, const Node& b) {
	return a.x == b.x && a.y == b.y;
}

bool operator!=(const Node& a, const Node& b) {
	return a.x != b.x || a.y != b.y;
}

bool operator<(const Node& a, const Node& b) {
	if (a.y < b.y) {
		return true;
	}
	if (b.y < a.y) {
		return false;
	}
	return a.x < b.x;
}

void insertVisitedIfPossible(VisitedNode v, const std::map<Node, NodeType> &nodeTypes, std::set<VisitedNode> &visited, int *costs, Action *actions, const Node &toNode);

//
// Graph
//

Graph::Graph() {
	// по умолчанию все узлы неизвестны
	for (int x = 0; x < COLS; ++x) {
		for (int y = 0; y < ROWS; ++y) {
			nodeTypes[{x, y}] = EMPTY;
		}
	}
}

NodeType Graph::getNodeType(const Node& node) const {
	return nodeTypes.find(node)->second;
}

void Graph::setNodeType(const Node& node, NodeType type) {
	nodeTypes[node] = type;
}

static int pathNumber = 0;

std::pair<std::vector<Action>, Direction> Graph::pathFromNodeToNode(Node fromNode, Direction fromDirection, Node toNode) {
	pathNumber++;
	std::stringstream name;
	name << "/home/root/lms2012/prjs/rro2023/path" << pathNumber << ".txt";
	auto fOut = std::ofstream(name.str());

	fOut << fromNode.x << "," << fromNode.y << std::endl << toNode.x << "," << toNode.y << std::endl << fromDirection << std::endl;

	const int N = ROWS*COLS*4;
	int costs[N];
	Action actions[N];
	const int INF = N*costForAction[TURN_AROUND];
	for (int i = 0; i < N; ++i) {
		costs[i] = INF;
	}
	costs[toIndex(fromNode, fromDirection)] = 0;
	actions[toIndex(fromNode, fromDirection)] = FORWARD;
	std::set<VisitedNode> visited;
	VisitedNode start = {Node(fromNode), Direction(fromDirection), 0};
	visited.emplace(VisitedNode(start));
	while (!visited.empty()) {
		auto v = *(visited.begin());
		visited.erase(visited.begin());
		if (costs[v.index()] != v.cost) {
			continue;
		}
		auto next = v.forward();
		if (next.has_value()) {
			insertVisitedIfPossible(std::move(*next), nodeTypes, visited, costs, actions, toNode);
		}
		insertVisitedIfPossible(v.turnLeft(), nodeTypes, visited, costs, actions, toNode);
		insertVisitedIfPossible(v.turnRight(), nodeTypes, visited, costs, actions, toNode);
		insertVisitedIfPossible(v.turnAround(), nodeTypes, visited, costs, actions, toNode);
	}

	for (int k = 0; k < 4; ++k) { // directions
		fOut << std::endl;
		for (int y = COLS - 1; y >= 0; y--) {
			for (int x = 0; x < ROWS; ++x) {
				fOut << costs[toIndex({x, y}, Direction(k))] << '\t';
			}
			fOut << std::endl;
		}
		fOut << std::endl;
	}

	fOut << std::endl;

	for (int k = 0; k < 4; ++k) {
		fOut << std::endl;
		for (int y = COLS - 1; y >= 0; y--) {
			for (int x = 0; x < ROWS; ++x) {
				fOut << actions[toIndex({x, y}, Direction(k))] << '\t';
			}
			fOut << std::endl;
		}
		fOut << std::endl;
	}

	auto bestCost = INF;
	auto bestDirection = RIGHT;
	for (int i = 0; i < 4; ++i) {
		auto cost = costs[toIndex(toNode, Direction(i))];
		if (cost < bestCost) {
			bestDirection = Direction(i);
			bestCost = cost;
		}
	}

	fOut << std::endl;
	Node bestNode = toNode;
	std::vector<Action> result;
	auto resultDirection = bestDirection;
	while (bestNode != fromNode || bestDirection != fromDirection) {
		int idx = toIndex(bestNode, bestDirection);
		auto action = actions[idx];
		fOut << action << std::endl;
		result.push_back(actions[idx]);

		switch(action) {
		case FORWARD:
			bestNode = {bestNode.x - dx[bestDirection], bestNode.y - dy[bestDirection]};
			break;
		case TURN_LEFT:
			bestDirection = Direction((bestDirection + 1) % 4);
			break;
		case TURN_RIGHT:
			bestDirection = Direction((bestDirection + 3) % 4);
			break;
		case TURN_AROUND:
			bestDirection = Direction((bestDirection + 2) % 4);
			break;
		default:
			// path not found
			bestNode = fromNode;
			bestDirection = fromDirection;
			break;
		}
	}
	std::reverse(result.begin(), result.end());

	return std::make_pair<std::vector<Action>, Direction>(std::move(result), std::move(resultDirection));
}

void insertVisitedIfPossible(VisitedNode v, const std::map<Node, NodeType> &nodeTypes, std::set<VisitedNode> &visited, int* costs, Action *actions, const Node &toNode) {
	int idx = v.index();
	if (v.cost >= costs[idx]) {
		return;
	}
	if (nodeTypes.find(v.node)->second != EMPTY && toNode != v.node) {
		return;
	}

	costs[idx] = v.cost;
	actions[idx] = v.action;
	visited.emplace(std::move(v));
}

