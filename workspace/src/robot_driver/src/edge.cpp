#include "edge.hpp"

using namespace std;
// TODO: complete the constructor
edge::edge(float startDistance, float middleDistance, float endDistance, int startIndex, int middleIndex, int endIndex) 
			: startDistance(startDistance), middleDistance(middleDistance), endDistance(endDistance), startIndex(startIndex), middleIndex(middleIndex), endIndex(endIndex) {	
	this->currentX = 0;
	this->currentY = 0;
}

// Write functions here:

float edge::getCurrentX() {
	return currentX;
}

float edge::getCurrentY() {
	return currentY;
}
