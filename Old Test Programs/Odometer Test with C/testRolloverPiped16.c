#include "pigpio.h"

struct timeval {
	int tv_sec;
	int tv_usec;
};

void printBits(size_t const size, void const * const ptr) {
	unsigned char *b = (unsigned char*) ptr;
	unsigned char byte;
	int i, j;

	for (i=size-1;i>=0;i--)
	{
		for (j=7;j>=0;j--)
		{
			byte = (b[i] >> j) & 1;
			printf("%u", byte);
		}
	}
	puts("");
}

int read_raw() {
        int a = 0;
	int reverse = 0;
	int readbit[2];
	uint16_t packets[2];
        static uint16_t data0;
	static uint16_t data1;
	//creates a mask of 16 bit length
        uint16_t mask = (1 << READING_LOW_0_BIT) - 1;
	//Prepare for the read by bringing the CS and clock pins high then low
        gpioWrite(CHIP_SELECT_PIN_1, PIN_HIGH);
	gpioDelay(TICK);
        gpioWrite(CLOCK_PIN_1, PIN_HIGH);
	gpioDelay(TICK);
        gpioWrite(CHIP_SELECT_PIN_1, PIN_LOW);
	gpioDelay(TICK);

	// loop 16 times to get two 16 bit reads from the data pins
	while (a < (READING_BIT_LENGTH + READING_LOW_0_BIT)) {
		gpioWrite(CLOCK_PIN_1, PIN_LOW);
                gpioDelay(TICK);

                gpioWrite(CLOCK_PIN_1, PIN_HIGH);
                gpioDelay(TICK);

                readbit[0] = gpioRead(DATA_PIN_1);
                readbit[1] = gpioRead(DATA_PIN_2);

                packets[0] = ((packets[0] << 1) + readbit[0]);
		packets[1] = ((packets[1] << 1) + readbit[1]);
                a += 1;
        };
	data0 = packets[0];
	data1 = packets[1];
	
	//print the bits of each data
//	printBits(sizeof(data0), &data0);
//	printBits(sizeof(data1), &data1);

	//return as a 32 bit number
	return ((data1 << 16) | data0);
}
//takes the 32 bit number and turns it into a 10 bit number
int bit_slicer (uint32_t from, int lowBit, int count) {
        int range = 1 << count;
        uint32_t mask = range - 1;
        return (from >> lowBit) & mask;
}
//handles the rollovers with the bigJump
int* handle_rollovers( int readings[2] ) {
        static int prevReadings[2];
        static int prevRollovers[2];
        static int values[2];
        int range = 1 << READING_BIT_LENGTH;
	int bigJump = range / 2;
        int i, change;
        for ( i = 0; i < 2; i++) {
//                printf ( "reading %d %d \n", idx, readings[i]);
//                print_bits(sizeof(readings), &readings[i]);
                change = readings[i] - prevReadings[i];
                prevReadings[i] = readings[i];
                prevRollovers[i] += (change > bigJump) ? -1 : (change < -bigJump);
                values[i] = readings[i] + range * prevRollovers[i];
        }
        //printf ( " %d\n" , prevRollovers[i]);
        return values;
}

void gpio_setup() {
        if (gpioInitialise() < 0)
        {
                fprintf(stderr, "pigpio initialisation failed\n");
                return 1;
        }
	gpioSetMode(CHIP_SELECT_PIN_1, PI_OUTPUT);
	gpioSetMode(DATA_PIN_1, PI_INPUT);
        gpioSetMode(DATA_PIN_2, PI_INPUT);
        gpioSetMode(CLOCK_PIN_1, PI_OUTPUT);
}

void writer(readTime, filename, is_first) {
        struct timespec ts;
        struct timespec ts2;
        ts.tv_sec = 1;
        ts.tv_nsec = 0;
	int init_reading_0;
	int init_reading_1;

	int s_fifo = mkfifo(filename, S_IRWXU);
        FILE * wfd = fopen(filename, "w");
        if (wfd < 0)
        {
                printf("open() error: %d\n", wfd);
                return -1;
        }

        while (1) {
		struct timeval tv;
		time_t curtime;
		gettimeofday(&tv, NULL);
		curtime = tv.tv_sec;
                int readings[2];
                int checkValue[2];
		int differingValue[2];
		int data = read_raw();

		readings[0] = bit_slicer ( data, READING_LOW_0_BIT, READING_BIT_LENGTH );
		readings[1] = bit_slicer ( data, READING_LOW_1_BIT, READING_BIT_LENGTH );
		int* values = handle_rollovers( readings );
		if (is_first == 0) {
			init_reading_0 = values[0];
			init_reading_1 = values[1];
			is_first = 1;
		}else {
			values[0] = values[0] - init_reading_0;
			values[1] = values[1] - init_reading_1;
		}		
		differingValue[0] = values[0] - checkValue[0];
		differingValue[1] = values[1] - checkValue[1];
		checkValue[0] = values[0];
		checkValue[1] = values[1];
		int s_write = fprintf(wfd, "%d,%d,%d,%d,%d,%d,%d,%d \n", values[0], -values[1], tv.tv_usec, differingValue[0], differingValue[1], data, readings[0], readings[1]);
                if (s_write < 0)
                {
                        printf("fprintf() error: %d\n", s_write);
                        break;
                }
                fflush(wfd);

//		printf("%d,%d,%d,%d,%d,%d,%d,%d \n", values[0], values[1], tv.tv_usec, differingValue[0], differingValue[1], data, readings[0], readings[1]);
                //usleep(sleepValue);
		gpioSleep(PI_TIME_RELATIVE, 0, readTime);
        }
}

int main(int argc, char *argv[]) {
	int readTime;
	char *filename;
	char *ptr;
	int is_first = 0;
	gpio_setup();
	if (argc < 3)
	{
		readTime = 16000;
		filename = "fifo.tmp";
	}
	else
	{
		readTime = strtol(argv[1], &ptr, 10);
		filename = argv[2];
	}
	writer(readTime, filename, is_first);
}
