#include <iostream>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <raspicam/raspicam_cv.h>
#include <tesseract/baseapi.h>

#define IMAGE_WIDTH (640)
#define IMAGE_HEIGHT (480)

using namespace std;

int main(int argc, char **argv)
{
	raspicam::RaspiCam_Cv Camera;
	cv::Mat image, gray, crop, blur, thresh, hsv;
	tesseract::TessBaseAPI tess;
	int start, stop, time;
	
	
	// Set camera params
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);

	// Init tesseract stuffs
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
	tess.SetVariable("tessedit_char_whitelist", "ABCDEFGHIJKLMNOPQRSTUVWZYZ0123456789!@#$%^&*()");
	// ABCDEFGHIJKLMNOPQRSTUVWZYZ0123456789!@#$%^&*()
	// Open camera
	if (!Camera.open())	{
		cerr << "Error opening the camera" << endl;
		return -1;
	}

	// Start capture
	Camera.grab();
	usleep(1000 * 300);
	
	while(1) {
		Camera.grab();
		Camera.retrieve(image);
		cv::imshow("Raw", image);

		
		
		

		//Cropping Region of interest
		crop = image(cv::Rect(0.25*IMAGE_WIDTH, 0.2*IMAGE_HEIGHT,0.5*IMAGE_WIDTH, 0.4*IMAGE_HEIGHT));
		cv::imshow("Crop", crop);
		
		// BGR to HSV
		cv::cvtColor(crop, hsv, CV_BGR2HSV);
		cv::imshow("HSV", hsv);
		
		
		//Grayscale
		cv::cvtColor(hsv, gray, CV_BGR2GRAY);
		cv::imshow("Gray", gray);
		
		// Gaussian Blurring
		cv::GaussianBlur(gray, blur, cv::Size(5,5), 0, 0); 
		cv::imshow("Blur", blur);
		
		// Direct Thresholding
		cv::threshold(blur, thresh, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
		
		// Adaptive Thresholding
		// cv::adaptiveThreshold(gray, thresh, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 13, 3);
		cv::imshow("Thresh", thresh);

		// Tesseract
		start = clock();
		tess.SetImage((uchar*)thresh.data, thresh.cols, thresh.rows, 1, thresh.cols);
		// tess.SetImage((uchar*)draw.data, draw.cols, draw.rows, 1, draw.cols);
		char* out = tess.GetUTF8Text();
		
		// Calculate Tesseract Time
		stop = clock();
		time = (stop-start) /double(CLOCKS_PER_SEC)*1000;
		cout << "Time (Tesseract) : " << time << endl;
		
		// Display result
		std::cout << out << std::endl;
		// usleep(1000 * 1000 * 2);
		// cv::waitKey(0);
		
		// Save image
		// cv::imshow("Sobel", draw);
		// cv::imshow("Crop", crop);
		// cv::imshow("Adaptive_Threshold", thresh);
		cv::waitKey(30);
		// cv::waitKey(0);
		// cv::imwrite("thresh.jpg", thresh);

	}
	Camera.release();

	return 0;
}
