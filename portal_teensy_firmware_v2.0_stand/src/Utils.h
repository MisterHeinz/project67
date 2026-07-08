static double toDouble(String value){
	unsigned int length = value.length() + 1;
	char buffer [length];
	value.toCharArray(buffer, length);
	if (buffer) return atof(buffer);
	return 0.0;
};

static String toString(double value){
	char buffer[40];
	sprintf(buffer, "%.16f", value);
	return String(buffer);
}