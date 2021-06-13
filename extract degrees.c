double Extract_degrees(char* lon) {
	double dl = atof(lon);
	double dl2 = dl / 100;
	int dint = (int)dl2;
	double dpoint = (dl2 - dint) * 100;
	return dint + (dpoint / 60);
}
