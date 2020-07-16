#include <vector>
#define UNIT_ARRAY_SIZE 32

using namespace std;

class coordinate {
public:
	int x;
	int y;
	coordinate(int _x, int _y) {
		x = _x;
		y = _y;
	}
};

class feature {
public:
	int id;
	feature(int _id) {
		id = _id;
	}
};

class frame {
public:
	int idx;
	int Arr_size = 0;
	vector<feature> feature_set;
	vector<coordinate> coordinate_set;
	void add_feature(feature feature_new, coordinate coordinate_new);
	frame(int _idx) {
		idx = _idx;
	}
};

void frame::add_feature(feature feature_new, coordinate coordinate_new) {
	feature_set.push_back(feature_new);
	coordinate_set.push_back(coordinate_new);
	Arr_size += 1;
}

