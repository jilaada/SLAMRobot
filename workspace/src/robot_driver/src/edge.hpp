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
    edge();
	
	// Declare public functions here:
	int getIndex();
	float getDistance();
	float getAngle();
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
	int index;
	float distance;
	float angle;
	float currentX;
	float currentY; 
    
};
