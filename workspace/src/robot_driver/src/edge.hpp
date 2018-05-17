class edge {
public:
	enum detectionType { 
		NO_EDGE,
		FALLING_EDGE,
		RISING_EDGE
	};	

    /*
    Edge Constructor
    */
    edge(float startDistance, float middleDistance, float endDistance, int startIndex, int middleIndex, int endIndex);
	
	// Declare public functions here:
	float getCurrentX();
	float getCurrentY();
    
private:
    /*
    Private edge copy constructor - edge cannot be copied, must be passed around via pointers and references.
    */
    edge(const edge&){};

    /*
    Private edge copy assignment operator - edge cannot be copied, must be passed around via pointers and
    references.
    */
    edge& operator=(const edge&) {
        return *this;
    };

	// Declare variables for use here:
	int startIndex;
	int middleIndex;
	int endIndex;
	float startDistance;
	float middleDistance;
	float endDistance;
	float currentX;
	float currentY; 
    
};
