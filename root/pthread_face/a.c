#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>

#define GPS_LOG "gps_log"
#define NAV_LOG "nav_log"

static volatile int ctrlC = 1;

void interruptHandler() {
    printf("Terminating \n");
    ctrlC = 0;
}

bool checkTaskStatus(char *fileName){
	FILE *fp;
        long fileSize;
        char *buffer;
	bool status = false;

        fp = fopen(fileName,"r");
        if ( fp == 0){
                printf("Error: %s does not exist",fileName);
                return status;
                }

	fseek(fp, 0, SEEK_END);
        fileSize = ftell(fp);
        buffer = (char*) malloc(fileSize + 1);
        fseek(fp, 0, SEEK_SET);
        fread(buffer, fileSize, 1, fp);
//	printf("%s\n",buffer);
	
       	if ( strstr(buffer, "failed") == NULL) // not failed{
		status = true;
		
        fclose(fp);
	free(buffer);
	return status;
}

void main(){
	/*
	printf("Calling system\n");
	int ret = system("./check");

        if (WIFSIGNALED(ret) &&
           (WTERMSIG(ret) == SIGINT || WTERMSIG(ret) == SIGQUIT))
           printf("got quit\n");
	*/
	unsigned int frameCount1=0;
	struct timespec ts1;
	double deadline = 200;
	double diff1 = 0.0,avgTime1 = 0.0,fc1=0,avgFps1 = 0.0;  // in ms
    	int i;
	//struct timespec ts1;
	double frameTime1, prevFrameTime1;
	
	printf("./gmaps.py starting.. ret val = %d\n",system("./gmaps.py"));
	sleep(5);
	printf("gmaps.py running..\n");
	signal(SIGINT, interruptHandler);
	while(ctrlC){
		clock_gettime(CLOCK_REALTIME, &ts1);
        	frameTime1 =((double)ts1.tv_sec * 1000.0) + ((double)((double)ts1.tv_nsec /1000000.0));
		frameCount1 ++;
        	// we won't have values set in the beginning
        	if(frameCount1 > 2){
        		fc1=(double)frameCount1;   // convert to double
                	avgTime1 =((fc1-1.0)*avgTime1 + diff1)/fc1;    // avg frame time
                	avgFps1 =1000.0/avgTime1; // avg frequency in fps
                }
		if ( checkTaskStatus(GPS_LOG) == false)
			printf("GPS task failed\n");
		//sleep(1);
		if ( checkTaskStatus(NAV_LOG) == false)
                        printf("NAV task failed\n");
		//sleep(1);
		diff1 = frameTime1 - prevFrameTime1;
        	prevFrameTime1 = frameTime1;
        	//if(  ( frameCount1 > 2 ) &&  (diff1 > deadline )){
            	//	printf("Oops! missed a deadline\n");
            	//	}
		}
		//printf(" Avg exeution time  -  %f ms\n",avgTime1);
    		//printf(" fps - %f\n", avgFps1);  
}


