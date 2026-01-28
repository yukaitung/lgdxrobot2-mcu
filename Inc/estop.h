enum __emergency_stops {
	software_emergency_stop = 0,
	hardware_emergency_stop = 1,
	bettery_low_emergency_stop = 2,
	emergency_stops_count = 3
};

void ESTOP_Init();
void ESTOP_Enable(int type);
void ESTOP_Disable(int type);
bool ESTOP_Get_Status(int type);