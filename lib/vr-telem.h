typedef struct {
        unsigned long sampleIndex;
        unsigned long timestamp;
        long posL;      // Hall angle position
        long posR;
        long composL;   // Commanded Hall angle position
        long composR;
        int dcL;        // PWM duty cycle
        int dcR;
        int gyroX;
        int gyroY;
        int gyroZ;
        int accelX;
        int accelY;
        int accelZ;
        int bemfL;
        int bemfR;
        int Vbatt; // battery voltage
    } vrTelemStruct_t;