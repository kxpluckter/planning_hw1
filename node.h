#include <iostream>

class Node {
public:
	int x;
	int y;
	double value;
	int parent_x;
	int parent_y;
	Node* parent;
	int g_value;
	int time;
	int visited;

	Node(): x(0), y(0), value(0), parent_x(0), parent_y(0), parent(NULL), g_value(0), time(0), visited(0){}
	Node(int x, int y, double value, int parent_x, int parent_y, int g_value, int time, int visited): x(x), y(y), value(value), parent_x(parent_x), parent_y(parent_y), g_value(g_value), time(time),  visited(visited){}
};