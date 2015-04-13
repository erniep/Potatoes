#include <iostream>
#include <ctime>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <raspicam/raspicam_cv.h>
#include <tesseract/baseapi.h>

#define IMAGE_WIDTH (640)
#define IMAGE_HEIGHT (480)

using namespace std;

char getWallColor(cv::Mat myHSV);
char testForCaret(cv::Mat myThresh);
static tesseract::TessBaseAPI tess;

int main(int argc, char **argv)
{
	raspicam::RaspiCam_Cv Camera;
	cv::Mat image, gray, crop, blur, thresh, hsv;
	int start, stop, time;
	char wallColor = '-';
	
	
	// Set camera params
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3); 				// Selects color channel
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);		// Sets Image width with defined macro
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);		// Sets Image height with defined macro
	Camera.set(CV_CAP_PROP_GAIN, 100);		// Sets Image height with defined macro
	// Camera.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);		// Sets Image height with defined macro
	// Camera.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);		// Sets Image height with defined macro

	// Init tesseract stuffs
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);		// Use PSM_SINGLE_BLOCK or PSM_SINGLE_CHAR
	
	// Open camera
	if (!Camera.open())	{
		cerr << "Error opening the camera" << endl;
		return -1;
	}
	Camera.grab();												// Open Camera
	usleep(1000* 300);										// Allow Camera to warm up (300 ms)
	
	while(1) {
		Camera.grab();
		Camera.retrieve(image);

		/* Cropping Region of interest */
		crop = image(cv::Rect(0*IMAGE_WIDTH, 0.3*IMAGE_HEIGHT,IMAGE_WIDTH, 0.6*IMAGE_HEIGHT));
		cv::imshow("Crop", crop);
		cv::imwrite("Crop.jpg", crop);
	
		/* BGR to HSV */
		cv::cvtColor(crop, hsv, CV_BGR2HSV);
		wallColor = getWallColor(hsv);
		
		if(wallColor == 'G') continue;
		else if(wallColor == 'B') tess.SetVariable("tessedit_char_whitelist", "0123456789!@#$%^&*()");
		else if(wallColor == 'R') tess.SetVariable("tessedit_char_whitelist", "ABCDEFGHIJKLMNOPQRSTUVWZYZ!@#$%^&*()");
		
		cout << "Wall Color: " << wallColor << endl;
		
		/* Grayscale */
		cv::cvtColor(hsv, gray, CV_BGR2GRAY);
		
		/* Gaussian Blurring */
		cv::GaussianBlur(gray, blur, cv::Size(5,5), 0, 0); 
		
		/* Direct Thresholding */
		cv::threshold(blur, thresh, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);


		/* Tesseract */
		// start = clock();
		tess.SetImage((uchar*)thresh.data, thresh.cols, thresh.rows, 1, thresh.cols);
		char out = (tess.GetUTF8Text())[0];
		
		
		if (out == 'A') out = testForCaret(thresh);
		
		/* Display result */
		cout << "Character: " << out << endl;
		// usleep(1000 * 1000 * 5);
		cv::imshow("HSV", hsv);
		cv::imshow("Thresh", thresh);
		cv::waitKey(0);
		
		/* Save image */
		/* Calculate Tesseract Time */
		// stop = clock();
		// time = (stop-start) /double(CLOCKS_PER_SEC)*1000;
		// cout << "Time (Tesseract) : " << time << endl;
		// cv::imshow("Raw", image);
		// cv::imshow("Gray", gray);
		// cv::imshow("Blur", blur);
		// cv::imwrite("hsv.jpg", hsv);

	}
	Camera.release();

	return 0;
}

char getWallColor(cv::Mat myHSV){
		cv::vector<cv::Mat> hsv_planes;
		cv::split(myHSV, hsv_planes);
		int histSize = 360; 
		float range[] = {0, 360};
		const float* histRange = {range}; 
		bool uniform = true, accumulate = false;
		cv::Mat h_hist;
		cv::calcHist(&hsv_planes[0], 1, 0, cv::Mat(), h_hist, 1, &histSize, &histRange, uniform, accumulate);
		int hist_w = 720; int hist_h = 1000;
		int bin_w = cvRound( (double) hist_w/histSize);
		cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));
		normalize(h_hist, h_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
		int colorIndex = 0;
		
		/* Drawing a Histogram */
		/* for(int i=1; i<=histSize; i++){
			// cout << "Index: " << i << "\t\t" << "Value: " << cvRound(h_hist.at<float>(i-1)) << endl;
			line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(h_hist.at<float>(i-1)) ), 
							cv::Point(bin_w*(i), hist_h - cvRound(h_hist.at<float>(i)) ),
							cv::Scalar( 255, 0, 0), 2, 8, 0  );
		} */
		
		for(int i=histSize; i>0; i--){
			// cout << "Index: " << i << "\t\t" << "Value: " << cvRound(h_hist.at<float>(i-1)) << endl;
			if(cvRound(h_hist.at<float>(i-1)) > 500){	
				colorIndex = i-1;
				break;
			}
		}		
		cout << "Color Index: " << colorIndex << endl;
		if((colorIndex > 75) && (colorIndex < 175)) return 'B';
		else if(colorIndex > 50) return 'G';
		else return 'R';
	
		// cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);
		// cv::imshow("calcHist Demo", histImage);
		// cv::waitKey(0);
	
}

char testForCaret(cv::Mat myThresh){
	cv::flip(myThresh, myThresh, 0);
	tess.SetImage((uchar*)myThresh.data, myThresh.cols, myThresh.rows, 1, myThresh.cols);
	char out = (tess.GetUTF8Text())[0];
	// cout << "CaretTest: " << out << endl;
	if (out == 'V') return '^';
	else return 'A';
	
	
}










