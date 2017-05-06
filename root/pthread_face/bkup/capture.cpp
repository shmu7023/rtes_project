#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sched.h>
#include <syslog.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/param.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include "ultra1.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define NUMBER_OF_FRAMES 100

#define SEMAPHORE_NAME "/semaphore_example"
#define SEMAPHORE_PERMISSION (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)
#define INITIAL_VALUE 0

using namespace cv;
using namespace std;

string fn_haar;
int deviceId;
int im_width;
int im_height;


sem_t semaphore_motor_sensor; 
sem_t semaphore_motor_finder; 

typedef struct
{
    int threadIdx;
} threadParams_t;

VideoCapture capture;

struct timespec frame_time_recognition, frame_time_finder;
double curr_frame_time_recognition, prev_frame_time_recognition;
double curr_frame_time_finder, prev_frame_time_finder;
int frame_cnt_finder = 0, frame_cnt_recognition = 0;



struct timespec frame_time_recognition_sensor, frame_time_sensor;
double curr_frame_time_recognition_sensor, prev_frame_time_recognition_sensor;
double curr_frame_time_sensor, prev_frame_time_sensor;
int frame_cnt_sensor = 0, frame_cnt_recognition_sensor = 0;


double deadline_ms_recognition = 5000;
double deadline_ms_finder = 5000;
double deadline_ms_sensor = 10000;

#define NUM_THREADS (1)
#define NUM_CPUS (1)
pthread_t threads[NUM_THREADS]; //array of pthreads
threadParams_t threadParams[NUM_THREADS]; //array of thread parameters
struct sched_param rt_param[NUM_THREADS];
struct sched_param main_param;
pthread_attr_t main_attr;
pid_t mainpid; //process id

int rt_max_prio; //maximum priority
int rt_min_prio; //minimum priority
int min;

#define HRES 640
#define VRES 480
#define SET_NO 1

Ptr<FaceRecognizer> model;

pthread_mutex_t frame_cap_lock, frame_show_lock;

/***
* Used to print the scheduling policy used in the code
***/
void print_scheduler (void)
{
	int schedType;
	schedType = sched_getscheduler(getpid());
	switch(schedType)
	{
		case SCHED_FIFO:
			printf("Pthread Policy is SCHED_FIFO\n");
			break;
		case SCHED_OTHER:
			printf("Pthread Policy is SCHED_OTHER\n");
			break;
		case SCHED_RR:
			printf("Pthread Policy is SCHED_OTHER\n");
			break;
		default:
			printf("Pthread Policy is UNKNOWN\n");
	}
}

int getkey() 
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

//initial min and max HSV filter values.
//these will be changed using trackbars

int G_MIN = 0;
int G_MAX = 256;

void on_trackbar( int, void* )
{
	//This function gets called whenever a
	// trackbar position is changed
}

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window


volatile int x = 0; 
volatile int y = 0; 


const string trackbarWindowName = "Trackbars";

class findroad
{                              //class that separates out roads from images
	private:
	Mat path;
	public:
	void setpath(Mat& image)
	{
		image.convertTo(path, CV_32S);
	}
	 
	Mat getroad(Mat &image)                  //major working function attribute of the class
	{
		watershed(image, path);                                     //using watershed segmenter
		path.convertTo(path,CV_8U);
		return path;
	}
};
 



string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars()
{
	//create window for trackbars
    namedWindow(trackbarWindowName,0);

	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "G_MIN", G_MIN);
	sprintf( TrackbarName, "G_MAX", G_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    
    createTrackbar( "G_MIN", trackbarWindowName, &G_MIN, G_MAX, on_trackbar );
    createTrackbar( "G_MAX", trackbarWindowName, &G_MAX, G_MAX, on_trackbar );
}

void drawObject(Mat &frame)
{
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
}



void trackFilteredObject(Mat threshold, Mat &cameraFeed)
{
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) 
	{
		int numObjects = hierarchy.size();
#ifdef DEBUG
		printf("\n numObjects: %d \n",numObjects);
#endif
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
			{

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//printf("\n Area: %ld \n",area);

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(1 /*area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea*/)
				{
					x = moment.m10/area;
					y = moment.m01/area;
#ifdef DEBUG
					printf("\n x: %d, y: %d \n",x,y);
#endif
					objectFound = true;
					refArea = area;
				}
				else 
					objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
			{
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(cameraFeed);
			}
		}
		else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}



void* path_finder(void*)
{
	double framedt_finder = 0.0, ave_framedt_finder = 0.0, fc_finder=0, ave_frame_rate_finder = 0.0, ave_framedt_jitter = 0.0, framedt_jitter = 0.0;

	 
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat gray;
	//matrix storage for binary threshold image
	//Mat threshold;
	Mat temp_image;
	//x and y values for the location of the object

	pthread_mutex_lock(&frame_show_lock);
	createTrackbars();
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	pthread_mutex_unlock(&frame_show_lock);

	while(1)
	{
		//sem_wait(&semaphore_motor_finder); 
		//temp_image = imread(argv[1],1);
		//store image to matrix
		clock_gettime(CLOCK_REALTIME, &frame_time_finder);
        curr_frame_time_finder =((double)frame_time_finder.tv_sec * 1000.0) + ((double)((double)frame_time_finder.tv_nsec /1000000.0));
        frame_cnt_finder++;
        if(frame_cnt_finder > 2)
		{
            fc_finder=(double)frame_cnt_finder;
            ave_framedt_finder =((fc_finder-1.0)*ave_framedt_finder + framedt_finder)/fc_finder;
			ave_framedt_jitter = ((fc_finder - 1.0)*ave_framedt_jitter + framedt_jitter)/fc_finder;
            ave_frame_rate_finder =1.0/(ave_framedt_finder/1000.0);
		}

		pthread_mutex_lock(&frame_cap_lock);
		capture.read(temp_image);
		pthread_mutex_unlock(&frame_cap_lock);
		
		resize(temp_image,cameraFeed,Size(500,500),0,0,INTER_LINEAR);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,gray,CV_BGR2GRAY);

		threshold(gray, gray, G_MIN, G_MAX, THRESH_BINARY_INV);          //threasholding the grayscale image


		//filter HSV image between values and store filtered image to
		//threshold matrix

		Mat Erode;
		erode(gray,Erode,Mat(),Point(2,2),7); //eroding image 7 times.
		Mat Dilate;
		dilate(gray,Dilate,Mat(),Point(2,2),7);    //dilating the image
		threshold(Dilate,Dilate,1, 50,THRESH_BINARY_INV);

		Mat path_trace(gray.size(),CV_8U,Scalar(0));
		path_trace= Erode+Dilate; 

		findroad road;                                             //creating an object for our class
		road.setpath(path_trace);                                      //preparing the input
		Mat road_found = road.getroad(cameraFeed);
		road_found.convertTo(road_found,CV_8U);

		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object

		threshold(road_found, road_found, G_MIN, G_MAX, THRESH_BINARY);          //threasholding the grayscale image
		//if(trackObjects)
		trackFilteredObject(road_found,cameraFeed);
		pthread_mutex_lock(&frame_show_lock);
		//imshow("#2---Output",gray);
		imshow("#2---Main",cameraFeed);
		imshow("#2---Roadfound", road_found);
		pthread_mutex_unlock(&frame_show_lock);
		//show frames
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command

		if(x <= 150 || x>= 350)
		{
			sem_post(&semaphore_motor_finder);   
		}

		
		framedt_finder = curr_frame_time_finder - prev_frame_time_finder;
		framedt_jitter = framedt_finder - deadline_ms_finder;
        prev_frame_time_finder = curr_frame_time_finder;

        if( (frame_cnt_finder > 2)  && (framedt_finder > deadline_ms_finder))  
			printf("---#2>>>Deadline Missed- Path Finder Transform [%d] ---- %f\n", frame_cnt_finder, framedt_finder);

		if(frame_cnt_finder % 50 == 0)
		{
			printf("#2---Average execution time (Path Finder Transform) = %fms\n",ave_framedt_finder);
    		printf("#2---Frame rate (Path Finder Transform) = %f\n", ave_frame_rate_finder);
			printf("#2---Frame jitter Path Finder = %f\n", ave_framedt_jitter); 
		}

		if(getkey() == '2')
		{
			printf("Break out of Path Finder\n");
			break;
		}
		char q = cvWaitKey(1);
	}
	if(ave_framedt_finder > deadline_ms_finder)
        printf("#2---Average Deadline Missed- Path Finder Transform\n");
    
    printf("End-#2---Average execution time Path Finder = %fms\n",ave_framedt_finder);
    printf("End-#2---Frame rate Path Finder = %f\n", ave_frame_rate_finder);
	printf("End-#2---Frame jitter Path Finder = %f\n", ave_framedt_jitter);
	capture.release();

	return NULL;
}

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') 
{
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) 
	{
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) 
	{
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) 
		{
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}

void * face_recognition(void *)
{
	double framedt_recognition = 0.0, ave_framedt_recognition = 0.0, fc_recognition=0, ave_frame_rate_recognition = 0.0, ave_framedt_jitter = 0.0, framedt_jitter = 0.0; 
	// Get a handle to the Video device:
    VideoCapture cap(deviceId);
    // Check if we can use this device at all:
    if(!cap.isOpened()) 
	{
        cerr << "Capture Device ID " << deviceId << "cannot be opened." << endl;
        return NULL;
    }

	CascadeClassifier haar_cascade;
    haar_cascade.load(fn_haar);

	// Holds the current frame from the Video device:
    Mat frame;
	Mat original;
	Mat gray;
	string box_test;
    for(;;) 
	{
		pthread_mutex_lock(&frame_cap_lock);
        cap >> frame;
		
        // Clone the current frame:
        original = frame.clone();
		pthread_mutex_unlock(&frame_cap_lock);

		clock_gettime(CLOCK_REALTIME, &frame_time_recognition);
        curr_frame_time_recognition =((double)frame_time_recognition.tv_sec * 1000.0) + ((double)((double)frame_time_recognition.tv_nsec /1000000.0));
        frame_cnt_recognition++;
	
		if(frame_cnt_recognition > 2)
		{
			fc_recognition=(double)frame_cnt_recognition;
            ave_framedt_recognition =((fc_recognition-1.0)*ave_framedt_recognition + framedt_recognition)/fc_recognition;
			ave_framedt_jitter = ((fc_recognition - 1.0)*ave_framedt_jitter + framedt_jitter)/fc_recognition;
            ave_frame_rate_recognition =1.0/(ave_framedt_recognition/1000.0);
		}

        // Convert the current frame to grayscale:
        cvtColor(original, gray, CV_BGR2GRAY);
        // Find the faces in the frame:
        vector< Rect_<int> > faces;
        haar_cascade.detectMultiScale(gray, faces);
        // At this point you have the position of the faces in
        // faces. Now we'll get the faces, make a prediction and
        // annotate it in the video. Cool or what?
        for(int i = 0; i < faces.size(); i++) 
		{
            // Process face by face:
            Rect face_i = faces[i];

            // Crop the face from the image. So simple with OpenCV C++:
            Mat face = gray(face_i);

            // Resizing the face is necessary for Eigenfaces and Fisherfaces. You can easily
            // verify this, by reading through the face recognition tutorial coming with OpenCV.
            // Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
            // input data really depends on the algorithm used.
            // Since I am showing the Fisherfaces algorithm here, I also show how to resize the
            // face you have just found:
            Mat face_resized;
            cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);

            // Now perform the prediction, see how easy that is:
			// Some variables for the predicted label and associated confidence (e.g. distance):
			int predicted_label = -1;
			double predicted_confidence = 0.0;
            //int prediction = model->predict(face_resized);
			model->predict(face_resized, predicted_label, predicted_confidence);			
			//cout << "#1---Prediction = " << predicted_confidence << " " << predicted_label << endl;

            // And finally write all we've found out to the original image!
            // First of all draw a green rectangle around the detected face:
            rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            // Create the text we will annotate the box with:
			if(predicted_label == 3)
			{
            	// Calculate the position for annotated text (make sure we don't
            	// put illegal values in there):
				int pos_x = std::max(face_i.tl().x - 10, 0);
            	int pos_y = std::max(face_i.tl().y - 10, 0);
				string box_text = format("Prediction = %d, Aaresh", predicted_label);
            	// And now put it into the image:
				putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
			}
			else
			{
            	// Calculate the position for annotated text (make sure we don't
            	// put illegal values in there):
		        int pos_x = std::max(face_i.tl().x - 10, 0);
		        int pos_y = std::max(face_i.tl().y - 10, 0);
				string box_text = format("#1---Prediction = %d", predicted_label);
            	// And now put it into the image:
				putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
			}

        }
        // Show the result:
		pthread_mutex_lock(&frame_show_lock);
        imshow("#1---face_recognizer", original);
		pthread_mutex_unlock(&frame_show_lock);
        
		
		framedt_recognition = curr_frame_time_recognition - prev_frame_time_recognition;
		framedt_jitter = framedt_recognition - deadline_ms_recognition;
        prev_frame_time_recognition = curr_frame_time_recognition;			

        if(  ( frame_cnt_recognition > 2 ) &&  (framedt_recognition > deadline_ms_recognition))
		{
            printf("---#1>>>Deadline Missed- Face Recognition [%d] ---- %f\n", frame_cnt_recognition, framedt_recognition);
		}

		if( frame_cnt_recognition % 50 == 0)
		{
  	  		printf("#1---Average execution time Face Recognition = %fms\n",ave_framedt_recognition);
    		printf("#1---Frame rate Face Recognition = %f\n", ave_frame_rate_recognition);		
			printf("#1---Frame jitter Face Recognition = %f\n", ave_framedt_jitter);	
		}	
		
        if(getkey() == '1')
		{
			printf("Break out of Face Recognition\n");
			break;
		}
		char key = (char) waitKey(20);
    }
	if(ave_framedt_recognition > deadline_ms_recognition)
        printf("Average Deadline Missed- Face Recognition\n");
    
    printf("End-#1---Average execution time Face Recognition = %fms\n",ave_framedt_recognition);
    printf("End-#1---Frame rate Face Recognition = %f\n", ave_frame_rate_recognition);
	printf("End-#1---Frame jitter Face Recognition = %f\n", ave_framedt_jitter);
 	cap.release();
	return NULL;
}

jetsonGPIO hcsr04Trigger;
jetsonGPIO hcsr04Echo;
jetsonGPIO hcsr04Trigger1;
jetsonGPIO hcsr04Echo1;
jetsonGPIO triggerGPIO=gpio165; 
jetsonGPIO echoGPIO=gpio166;
jetsonGPIO triggerGPIO1=gpio163; 
jetsonGPIO echoGPIO1=gpio164;

jetsonGPIO motor1_1=gpio160;
jetsonGPIO motor1_2=gpio161;
jetsonGPIO motor2_1=gpio162;
jetsonGPIO motor2_2=gpio57;

void run_motors()
{
	/*gpioSetValue(motor1_1,low);
	gpioSetValue(motor1_2,low);
	gpioSetValue(motor2_1,low);
	gpioSetValue(motor2_2,low);
	usleep(10);*/

	gpioSetValue(motor1_1,low);
	gpioSetValue(motor1_2,high);
	gpioSetValue(motor2_1,low);
	gpioSetValue(motor2_2,high);
#ifdef DEBUG
	printf("Running motors\n");
#endif
}

void stop_motors()
{
       gpioSetValue(motor1_1,low);
       gpioSetValue(motor1_2,low);
       gpioSetValue(motor2_1,low);
       gpioSetValue(motor2_2,low);

}

void reverse()
{
       gpioSetValue(motor1_1,low);
       gpioSetValue(motor1_2,high);
       gpioSetValue(motor2_1,low);
       gpioSetValue(motor2_2,high);
	usleep(3000000);

}

void turn_right()
{
       gpioSetValue(motor1_1,low);
       gpioSetValue(motor1_2,low);
       gpioSetValue(motor2_1,high);
       gpioSetValue(motor2_2,low);

	usleep(3000000);

       gpioSetValue(motor1_1,low);
       gpioSetValue(motor1_2,low);
       gpioSetValue(motor2_1,low);
       gpioSetValue(motor2_2,low);

}

void turn_left()
{
       gpioSetValue(motor1_1,high);
       gpioSetValue(motor1_2,low);
       gpioSetValue(motor2_1,low);
       gpioSetValue(motor2_2,low);

	usleep(3000000);

       gpioSetValue(motor1_1,low);
       gpioSetValue(motor1_2,low);
       gpioSetValue(motor2_1,low);
       gpioSetValue(motor2_2,low);

}

int gpioExport ( jetsonGPIO gpio )
{
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioExport unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio);
    if (write(fileDescriptor, commandBuffer, length) != length) {
        perror("gpioExport");
        return fileDescriptor ;

    }
    close(fileDescriptor);

    return 0;
}

int gpioUnexport ( jetsonGPIO gpio )
{
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioUnexport unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio);
    if (write(fileDescriptor, commandBuffer, length) != length) {
        perror("gpioUnexport") ;
        return fileDescriptor ;
    }
    close(fileDescriptor);
    return 0;
}
int gpioSetDirection ( jetsonGPIO gpio, unsigned int out_flag )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioSetDirection unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (out_flag) {
        if (write(fileDescriptor, "out", 4) != 4) {
            perror("gpioSetDirection") ;
            return fileDescriptor ;
        }
    }
    else {
        if (write(fileDescriptor, "in", 3) != 3) {
            perror("gpioSetDirection") ;
            return fileDescriptor ;
        }
    }
    close(fileDescriptor);
    return 0;
}

//
// gpioSetValue
// Set the value of the GPIO pin to 1 or 0
// Return: Success = 0 ; otherwise open file error
int gpioSetValue ( jetsonGPIO gpio, unsigned int value )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioSetValue unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (value) {
        if (write(fileDescriptor, "1", 2) != 2) {
            perror("gpioSetValue") ;
            return fileDescriptor ;
        }
    }
    else {
        if (write(fileDescriptor, "0", 2) != 2) {
            perror("gpioSetValue") ;
            return fileDescriptor ;
        }
    }
    close(fileDescriptor);
    return 0;
}

//
// gpioGetValue
// Get the value of the requested GPIO pin ; value return is 0 or 1
// Return: Success = 0 ; otherwise open file error
int gpioGetValue ( jetsonGPIO gpio, unsigned int *value)
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];
    char ch;

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fileDescriptor = open(commandBuffer, O_RDONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioGetValue unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (read(fileDescriptor, &ch, 1) != 1) {
        perror("gpioGetValue") ;
        return fileDescriptor ;
     }

    if (ch != '0') {
        *value = 1;
    } else {
        *value = 0;
    }

    close(fileDescriptor);
    return 0;
}

bool triggerPing ( int sensor ) 
{
	if (sensor==0)
	{
	    unsigned int maxEcho = MAX_SENSOR_DISTANCE*ROUNDTRIP_CM + MAX_SENSOR_DELAY ;
	    unsigned int echoValue = low ;
	    // figure out duration of the pulse
	    struct timeval tval_before, tval_after, tval_result;
	    // Set trigger low to get ready to start
	    gpioSetValue(hcsr04Trigger,low) ;
	    usleep(4) ;
	    // Pull trigger high for 10 microseconds to initiate echo location
	    gpioSetValue(hcsr04Trigger,high) ;
	    usleep(10) ;
	    gpioSetValue(hcsr04Trigger,low) ;
	    gettimeofday(&tval_before, NULL);

	    gpioGetValue(hcsr04Echo,&echoValue) ;
	    if (echoValue) 
		{
		return false ;  // Previous echo not finished
    		}
    while (echoValue != high) 
	{
        gpioGetValue(hcsr04Echo,&echoValue) ;
        gettimeofday(&tval_after, NULL);
        timersub(&tval_after, &tval_before, &tval_result);
		if (tval_result.tv_usec > maxEcho) 
		{
		    return false ;
		}
    	}
   	 return true ;
	}
else if (sensor==1)
	{
	    unsigned int maxEcho = MAX_SENSOR_DISTANCE*ROUNDTRIP_CM + MAX_SENSOR_DELAY ;
	    unsigned int echoValue = low;
	    // figure out duration of the pulse
	    struct timeval tval_before, tval_after, tval_result;
	    // Set trigger low to get ready to start
	    gpioSetValue(hcsr04Trigger1,low) ;
	    usleep(4) ;
	    // Pull trigger high for 10 microseconds to initiate echo location
	    gpioSetValue(hcsr04Trigger1,high) ;
	    usleep(10) ;
	    gpioSetValue(hcsr04Trigger1,low) ;
	    gettimeofday(&tval_before, NULL);

	    gpioGetValue(hcsr04Echo1,&echoValue) ;
	   	 if (echoValue) 
		{
			return false ;  // Previous echo not finished
	    	}
	    while (echoValue != high) 
		{
		gpioGetValue(hcsr04Echo1,&echoValue) ;
		gettimeofday(&tval_after, NULL);
		timersub(&tval_after, &tval_before, &tval_result);
	       		if (tval_result.tv_usec > maxEcho) 
			{
		    	return false ;
			}
	    	}
	    	return true ;
	}
}

unsigned int ping ( int sensor ) 
{
	if (sensor==0)
	{
		if (!triggerPing(0)) 
			{
			// Timed out ; No objects or objects too close to sensor to measure
			return NO_ECHO ;
	
		    // trigger a ping, then wait for the echo
		   
		    	}
		else
		{
			unsigned int maxEcho = MAX_SENSOR_DISTANCE*ROUNDTRIP_CM + MAX_SENSOR_DELAY ;
			unsigned int echoValue = high ;
			// figure out duration of the pulse
			struct timeval tval_before, tval_after, tval_result;
			gettimeofday(&tval_before, NULL);
			while (echoValue != low) 	
				{
				    gpioGetValue(hcsr04Echo,&echoValue) ;
				    if (echoValue==low) 
					{
						break;
				   	 }

				    gettimeofday(&tval_after, NULL);
				    timersub(&tval_after, &tval_before, &tval_result);
				    if (tval_result.tv_usec > maxEcho) 
					{
					return NO_ECHO ;
				    	}
				}
			gettimeofday(&tval_after, NULL);
			timersub(&tval_after, &tval_before, &tval_result);
			// convert to microseconds
			// second conversion not necessary
			return tval_result.tv_sec*1000000+tval_result.tv_usec ;
		    }
	}

	else if (sensor==1)
	{
		if (!triggerPing(1)) 
		{
			// Timed out ; No objects or objects too close to sensor to measure
			return NO_ECHO ;
	
		    // trigger a ping, then wait for the echo
		   
		}
		else
		{
			unsigned int maxEcho = MAX_SENSOR_DISTANCE*ROUNDTRIP_CM + MAX_SENSOR_DELAY ;
			unsigned int echoValue = high ;
			// figure out duration of the pulse
			struct timeval tval_before, tval_after, tval_result;
			gettimeofday(&tval_before, NULL);
			while (echoValue != low) 
			{
			    gpioGetValue(hcsr04Echo1,&echoValue) ;
			    if (echoValue==low) 
				{
					break;
			    	}

			    gettimeofday(&tval_after, NULL);
			    timersub(&tval_after, &tval_before, &tval_result);
			    if (tval_result.tv_usec > maxEcho) 
				{
					return NO_ECHO ;
			    	}
			}
			gettimeofday(&tval_after, NULL);
			timersub(&tval_after, &tval_before, &tval_result);
			// convert to microseconds
			// second conversion not necessary
			return tval_result.tv_sec*1000000+tval_result.tv_usec ;
		    }
	}
}

unsigned int calculateMedian (int count, unsigned int sampleArray[]) 
{
	unsigned int tempValue=0 ;
	int i,j ;   // loop counters
	for (i=0 ; i < count-1; i++) 
	{
		for (j=0; j<count; j++) 
		{
			if (sampleArray[j] < sampleArray[i]) 
			{
				// swap elements
				tempValue = sampleArray[i] ;
				sampleArray[i] = sampleArray[j] ;
				sampleArray[j] = tempValue ;
			}
		}
	}
	if (count%2==0) 
	{
		// if there are an even number of elements, return mean of the two elements in the middle
		return ((sampleArray[count/2] + sampleArray[count/2-1]) / 2.0) ;
	} 
	else 
	{
		// return the element in the middle
		return (sampleArray[count/2]) ;
	}
}


unsigned int pingMedian (int iterations, int sensor) 
{

	if (sensor==0)
	{
	    unsigned int pings[iterations] ;
	    unsigned int lastPing=0 ;
	    int index = 0 ;
	    int samples = iterations ;
	    int cursor = 0 ;
	    pings[0] = NO_ECHO ;
	    while (index < iterations) {
		lastPing = ping(0) ;
		if (lastPing != NO_ECHO) {
		    // Add this to the samples array
		    pings[cursor] = lastPing ;
		    cursor ++ ;
		} else {
		    // Nothing detected, don't add to samples.
		    samples -- ;
		}
		index ++ ;
		usleep(1000) ; // delay a millisecond between pings
	    }
	    // Figure out the median of the samples
	    if (samples == 0) return NO_ECHO ;
	    return calculateMedian(samples,pings) ;
	}
	else if (sensor==1)
	{
	    unsigned int pings[iterations] ;
	    unsigned int lastPing=0 ;
	    int index = 0 ;
	    int samples = iterations ;
	    int cursor = 0 ;
	    pings[0] = NO_ECHO ;
	    while (index < iterations) {
		lastPing = ping(1) ;
		if (lastPing != NO_ECHO) {
		    // Add this to the samples array
		    pings[cursor] = lastPing ;
		    cursor ++ ;
		} else {
		    // Nothing detected, don't add to samples.
		    samples -- ;
		}
		index ++ ;
		usleep(1000) ; // delay a millisecond between pings
	    }
	    // Figure out the median of the samples
	    if (samples == 0) return NO_ECHO ;
	    return calculateMedian(samples,pings) ;
	}
}


void* ultrasonic_read(void*)
{
	hcsr04Trigger = gpio165;
	hcsr04Echo = gpio166;
	hcsr04Trigger1 = gpio163;
	hcsr04Echo1 = gpio164;

	unsigned int duration;
	unsigned int duration1;
	unsigned int total_duration;


	double framedt_sensor = 0.0, ave_framedt_sensor = 0.0, fc_sensor=0, ave_frame_rate_sensor = 0.0, ave_framedt_jitter = 0.0, framedt_jitter = 0.0;


	while(1)
	{
		clock_gettime(CLOCK_REALTIME, &frame_time_sensor);
		curr_frame_time_sensor =((double)frame_time_sensor.tv_sec * 1000.0) + ((double)((double)frame_time_sensor.tv_nsec /1000000.0));
		frame_cnt_sensor++;
		if(frame_cnt_sensor > 2)
		{
			fc_sensor=(double)frame_cnt_sensor;
			ave_framedt_sensor =((fc_sensor-1.0)*ave_framedt_sensor + framedt_sensor)/fc_sensor;
			ave_framedt_jitter = ((fc_sensor - 1.0)*ave_framedt_jitter + framedt_jitter)/fc_sensor;
			ave_frame_rate_sensor =1.0/(ave_framedt_sensor/1000.0);
		}

		if(getkey() == '3')
		{
			printf("Break out of ultrasonic read\n");
			break;
		}
		#ifdef DEBUG
		printf("Exported\n");
		#endif
		gpioExport(hcsr04Trigger1);
		gpioExport(hcsr04Echo1) ;
		gpioExport(hcsr04Trigger);
		gpioExport(hcsr04Echo) ;
		gpioExport(motor1_1);
		gpioExport(motor1_2) ;
		gpioExport(motor2_1);
		gpioExport(motor2_2) ;
		gpioSetDirection(hcsr04Trigger,outputPin) ;
		gpioSetDirection(hcsr04Echo,inputPin) ;
		gpioSetDirection(hcsr04Trigger1,outputPin) ;
		gpioSetDirection(hcsr04Echo1,inputPin) ;
		gpioSetDirection(motor1_1,outputPin) ;
		gpioSetDirection(motor1_2,outputPin) ;
		gpioSetDirection(motor2_1,outputPin) ;
		gpioSetDirection(motor2_2,outputPin) ;
		duration = pingMedian(2,0);
		run_motors();
		if (duration == NO_ECHO) 
		{
			//printf("No echo for Sensor 0\n");
		} 
		else 
		{
	   		// print out distance in inches and centimeters
			//printf("Duration: %d, Distance (in):%d, Distance (cm):%d \n",duration, duration/148, duration/58);
		}
		usleep(1000);
		/*duration1 = pingMedian(2,1) ;
		if (duration1 == NO_ECHO) 
		{
			printf("No echo for Sensor 1\n");
		} 
		else 
		{
			// print out distance in inches and centimeters
			printf("Duration1: %d, Distance1 (in):%d, Distance1 (cm):%d \n",duration1, duration1/148, duration1/58);
		}
		usleep(1000);
		total_duration=((duration/148)+(duration1/148))/2;
		if (total_duration<10)
		{ 
			//mutexlock
			stop_motors();
			reverse();
			turn_right();
			//mutexunlock
		}*/
		gpioUnexport(hcsr04Trigger) ;
		gpioUnexport(hcsr04Echo) ;
		gpioUnexport(hcsr04Trigger1) ;
		gpioUnexport(hcsr04Echo1) ;
		gpioUnexport(motor1_1) ;
		gpioUnexport(motor1_2) ;
		gpioUnexport(motor2_1) ;
		gpioUnexport(motor2_2) ;
		duration=0;
		duration1=0;
		total_duration=0;
		#ifdef DEBUG
		printf("Unexported\n");
		#endif

		framedt_sensor = curr_frame_time_sensor - prev_frame_time_sensor;
		framedt_jitter = framedt_sensor - deadline_ms_sensor;
		prev_frame_time_sensor = curr_frame_time_sensor;

		if( (frame_cnt_sensor > 2)  && (framedt_sensor > deadline_ms_sensor))  
			printf("---#3>>>Deadline Missed- Ultrasonic Transform [%d] ---- %f\n", frame_cnt_sensor, framedt_sensor);

		if(frame_cnt_sensor % 50 == 0)
		{
			printf("#3---Average execution time (Ultrasonic Transform) = %fms\n",ave_framedt_sensor);
			printf("#3---Frame rate (Ultrasonic Sensor Transform) = %f\n", ave_frame_rate_sensor);
			printf("#3---Frame jitter Ultrasonic = %f\n", ave_framedt_jitter);
		}
	}	


	if(ave_framedt_sensor > deadline_ms_sensor)
		printf("#3---Average Deadline Missed- Ultrasonix Transform\n");

	printf("End-#3---Average execution time Path Finder = %fms\n",ave_framedt_sensor);
	printf("End-#3---Frame rate Ultrasonic = %f\n", ave_frame_rate_sensor);
	printf("End-#3---Frame jitter Ultrasonic = %f\n", ave_framedt_jitter);
	return NULL;
}

void* motor_control(void*)
{
	while(1)
	{
        if(sem_trywait(&semaphore_motor_finder) == 0)
	{
		 printf("#2---Adjusting motors\n");
		 if(x <= 100 && y <= 100)
		 {
			// half left	
		 }
		 else if(x <= 100 && y >= 350)
		 {
			// full  left	
		 }
		 else if(x >= 350 && y <= 100)
		 {
			// half right	
		 }
		 else if(x>=350 && y >=350)
		 {
			// full right	
      	         }		
		//sem_post(&semaphore_motor_finder);

	//sem_wait(&semaphore_motor_finder);
	}

	if(sem_trywait(&semaphore_motor_sensor) == 0)
	{
		 printf("#2---Adjusting sensors\n");
		//sem_post(&semaphore_motor_finder);

	//sem_wait(&semaphore_motor_sensor);
	}
	
	if(getkey() == '4')
	{
		break; 
	}
    }
	
}

int main( int argc, char** argv )
{

	if (argc != 4) 
	{
        cout << "usage: " << argv[0] << " </path/to/haar_cascade> </path/to/csv.ext> </path/to/device id>" << endl;
        cout << "\t Path to the Haar Cascade for face detection." << endl;
        cout << "\t Path to the CSV file with the face database." << endl;
        cout << "\t The webcam device id to grab frames from." << endl;
        exit(1);
    }
    // Get the path to your CSV:
    fn_haar = string(argv[1]);
    string fn_csv = string(argv[2]);
    deviceId = atoi(argv[3]);

    //sem_init(&semaphore_motor_sensor, 0, 1);
    //sem_init(&semaphore_motor_finder, 0, 1);

	//sem_t *m = sem_open(SEMAPHORE_NAME, O_CREAT | O_EXCL, SEMAPHORE_PERMISSION, INITIAL_VALUE);
	//sem_t *m = sem_open(SEMAPHORE_NAME, O_CREAT | O_EXCL, SEMAPHORE_PERMISSION, INITIAL_VALUE);
	

    // These vectors hold the images and corresponding labels:
    vector<Mat> images;
    vector<int> labels;
    // Read in the data (fails if no valid input filename is given, but you'll get an error message):
    try 
	{
        read_csv(fn_csv, images, labels);
    } 
	catch (cv::Exception& e) 
	{
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        // nothing more we can do
        exit(1);
    }
    // Get the height from the first image. We'll need this
    // later in code to reshape the images to their original
    // size AND we need to reshape incoming faces to this size:
    im_width = images[0].cols;
    im_height = images[0].rows;
	
	cout << "Width = " << im_width << " Height = " << im_height << endl;
    // Create a FaceRecognizer and train it on the given images:
    model = createFisherFaceRecognizer();
    model->train(images, labels);
 

	int rc;
	int i, scope;
	cpu_set_t cpuset;

	CPU_ZERO(&cpuset); //clear all CPUs for the set
	for(i=0; i < NUM_CPUS; i++)
	   CPU_SET(i, &cpuset); //Assign a CPU to the thread

	mainpid=getpid();

	rt_max_prio = sched_get_priority_max(SCHED_FIFO); //get maximum/highest priority level
	rt_min_prio = sched_get_priority_min(SCHED_FIFO); //get minimum/lowest priority level

	print_scheduler();
	rc=sched_getparam(mainpid, &main_param);
	main_param.sched_priority=rt_max_prio; //setting highest priority for main
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param); //setting main scheduling policy
	if(rc < 0) perror("main_param");
	print_scheduler();


	pthread_attr_getscope(&main_attr, &scope); //getting the scope of the system

	if(scope == PTHREAD_SCOPE_SYSTEM)
	 printf("PTHREAD SCOPE SYSTEM\n");
	else if (scope == PTHREAD_SCOPE_PROCESS)
	 printf("PTHREAD SCOPE PROCESS\n");
	else
	 printf("PTHREAD SCOPE UNKNOWN\n");

	printf("rt_max_prio=%d\n", rt_max_prio);
	printf("rt_min_prio=%d\n", rt_min_prio);

    pthread_attr_t rt_sched_attr;
    pthread_attr_t *attributeP;
    
    struct sched_param parameter;
    attributeP = &rt_sched_attr;
    
    parameter.sched_priority = 1;

    pthread_attr_init(&rt_sched_attr);
    pthread_attr_setinheritsched(&rt_sched_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&rt_sched_attr, SCHED_FIFO); //using SCHED_FIFO
    pthread_attr_setschedparam(&rt_sched_attr, &parameter);

	//pthread creation
	pthread_create(&threads[0], &rt_sched_attr, face_recognition, NULL);
	pthread_create(&threads[1], &rt_sched_attr, path_finder, NULL);
	pthread_create(&threads[2], &rt_sched_attr, ultrasonic_read, NULL);
	pthread_create(&threads[3], &rt_sched_attr, motor_control, NULL);
	
	
	//Joining of pthreads
    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);
    pthread_join(threads[2], NULL);
    pthread_join(threads[3], NULL);

	return 0;
};




