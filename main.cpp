#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "CaptureStereo.h"
#include <iostream>

using namespace std;

// Function Protoptypes
void captureFrame(IplImage* left, vector<CvPoint2D32f> lCorners, int lCornerCount, IplImage* right, vector<CvPoint2D32f> rCorners, int rCornerCount);
void calibrateStereo(CvSize imgSize);
void checkError(int totalPointCount, CvMat mLeftPoints, CvMat mRightPoints, CvMat mLeftParams, CvMat mLeftDistort, CvMat mRightParams, CvMat mRightDistort, CvMat mFundamental);
void computeRectification(CvMat mLeftParams, CvMat mRightParams, CvMat mLeftDistort, CvMat mRightDistort, CvMat mRotation, CvMat mTranslation, CvSize imageSize, CvFileStorage *leftStorage, CvFileStorage *rightStorage);
void cvShowStereoImage(char *windowTitle, IplImage *frameL, IplImage *frameR);

// The number of frames we've captured, as well as a storage for all the corners we've found
int numberFrames = 0;
vector<CvPoint2D32f> allLeftCorners;
vector<CvPoint2D32f> allRightCorners;

// Disparity Rendering Functions
CvStereoBMState *BMState=0;
void renderDisparity(IplImage* new_frameL, IplImage* new_frameR, CvMat* reprojection);

bool running = true;
bool isCalibrating = true;

//User modifiable settings
int cam1Index = 0, cam2Index = 1;
CvSize camSize = cvSize(320, 240);
char *outputFilename = "StereoCamera-320.yml";
/* Parameters for the size of the board (in number of internal corners),
   As well as actual distance between corners in centimetres */
const int chessboard_width = 8;
const int chessboard_height = 6;
const float squareSize = 2.9f;


void main() {
	//Initialize the cameras and turn on white balancing
	CameraStereo *cameraStereo = new CameraStereo(cam1Index, cam2Index, camSize); 

	//Initialize the storage for the current frame
	int lCornerCount, rCornerCount;
	vector<CvPoint2D32f> lCorners; lCorners.resize(chessboard_width* chessboard_height);
	vector<CvPoint2D32f> rCorners; rCorners.resize(chessboard_width* chessboard_height);

	//Loop while we're still running
	while (running) {
		//Get frames from the cameras
		IplImage *new_frameL = cameraStereo->getLeftFrame();
		IplImage *new_frameR = cameraStereo->getRightFrame();

		if (isCalibrating) {
			//Find the chessboard corners in both frames
			int lResult = cvFindChessboardCorners(new_frameL, cvSize(chessboard_width, chessboard_height), &lCorners[0], &lCornerCount);
			int rResult = cvFindChessboardCorners(new_frameR, cvSize(chessboard_width, chessboard_height), &rCorners[0], &rCornerCount);

			//Clone the images, render the chessboard corners, then clean up
			IplImage *leftCorners = cvCloneImage(new_frameL); IplImage *rightCorners = cvCloneImage(new_frameR);
			cvDrawChessboardCorners(leftCorners, cvSize(chessboard_width, chessboard_height), &lCorners[0], lCornerCount, lResult);
			cvDrawChessboardCorners(rightCorners, cvSize(chessboard_width, chessboard_height), &rCorners[0], rCornerCount, rResult);
			cvShowStereoImage("Input Images", leftCorners, rightCorners);
			cvReleaseImage(&leftCorners); cvReleaseImage(&rightCorners);

			//Check for key events
			switch (cvWaitKey(1)) {
				case 27:
					//If escape was pressed, stop running
					running = false; break;
				case ' ':
					//If space was pushed, and we successfully found all the chessboard corners, capture the frame
					if (lResult && rResult) {
						cout << "Capturing Image" << endl;
						captureFrame(new_frameL, lCorners, lCornerCount, new_frameR, rCorners, rCornerCount);
					}
					break;
				case 13:
					//If enter was pushed, run the calibration
					calibrateStereo(camSize);
					delete cameraStereo;
					cameraStereo = new CameraStereo(cam1Index, cam2Index, outputFilename);

					BMState = cvCreateStereoBMState();
					isCalibrating = false;
			}

		} else {
			renderDisparity(new_frameL, new_frameR, cameraStereo->getReprojection());
			cvShowStereoImage("Input Images", new_frameL, new_frameR);

			switch (cvWaitKey(1)) {
				case 27:
					//If escape was pressed, stop running
					running = false; break;
			}
		}

		//Clean up
		cvReleaseImage(&new_frameL); cvReleaseImage(&new_frameR);
	}

	delete cameraStereo;

}

/*Capture a good frame and store all the chessboard corners*/
void captureFrame(IplImage* left, vector<CvPoint2D32f> lCorners, int lCornerCount, IplImage* right, vector<CvPoint2D32f> rCorners, int rCornerCount) {

	//Convert the left image to black and white, and find the chessboard corners with sub pixel accuracy
	IplImage *lBW = cvCreateImage(cvGetSize(left), left->depth, 1); cvConvertImage(left, lBW);
	cvFindCornerSubPix(lBW, &lCorners[0], lCornerCount, cvSize(11,11), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01) );
	cvReleaseImage(&lBW);

	//Convert the right image to black and white, and find the chessboard corners with sub pixel accuracy
	IplImage *rBW = cvCreateImage(cvGetSize(right), right->depth, 1); cvConvertImage(right, rBW);
	cvFindCornerSubPix(rBW, &rCorners[0], rCornerCount, cvSize(11,11), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01) );
	cvReleaseImage(&rBW);

	//Insert the sub pixel accurate corners into the list of all corners and increment the number of frames
	allLeftCorners.insert(allLeftCorners.end(), lCorners.begin(), lCorners.end());
	allRightCorners.insert(allRightCorners.end(), rCorners.begin(), rCorners.end());
	numberFrames++;
}

/*Stereo Calibration using the captured points*/
void calibrateStereo(CvSize imgSize) {
	//Initialise the chessboard point array
	vector<CvPoint3D32f> chessboardPoints; chessboardPoints.resize(chessboard_width * chessboard_height);
	for (int i=0; i<chessboard_height; i++) 
		for (int j=0; j<chessboard_width; j++) 
			chessboardPoints[i*chessboard_width+j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);

	//Clone the points for each frame
	vector<CvPoint3D32f> allChessboardPoints;
	for (int i=0; i<numberFrames; i++) 
		allChessboardPoints.insert(allChessboardPoints.end(), chessboardPoints.begin(), chessboardPoints.end());

	//Find out the total point count
	int totalPointCount = chessboard_width * chessboard_height * numberFrames;

	//Initialize some matrices with the left, right and chessboard points
	CvMat mChessboardPoints = cvMat(1, totalPointCount, CV_32FC3, &allChessboardPoints[0] );
	CvMat mLeftPoints = cvMat(1, totalPointCount, CV_32FC2, &allLeftCorners[0] );
	CvMat mRightPoints = cvMat(1, totalPointCount, CV_32FC2, &allRightCorners[0] );
	
	vector<int> pointCount; pointCount.resize(numberFrames, chessboard_width * chessboard_height);
	CvMat mpointCount = cvMat(1, pointCount.size(), CV_32S, &pointCount[0] );
	
	// Initialise the camera intrinsic parameters
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];

	CvMat mLeftParams = cvMat(3, 3, CV_64F, M1 );
    CvMat mRightParams = cvMat(3, 3, CV_64F, M2 );
	cvSetIdentity(&mLeftParams);
	cvSetIdentity(&mRightParams);

	CvMat mLeftDistort = cvMat(1, 5, CV_64F, D1 );
    CvMat mRightDistort = cvMat(1, 5, CV_64F, D2 );
	cvZero(&mLeftDistort);
	cvZero(&mRightDistort);

    CvMat mRotation = cvMat(3, 3, CV_64F, R );
    CvMat mTranslation = cvMat(3, 1, CV_64F, T );
    CvMat mEssential = cvMat(3, 3, CV_64F, E );
    CvMat mFundamental = cvMat(3, 3, CV_64F, F );

	//Run calibration
    cout << "Running stereo calibration ..." << endl;
    cvStereoCalibrate(&mChessboardPoints, &mLeftPoints, &mRightPoints, &mpointCount,
        &mLeftParams, &mLeftDistort, &mRightParams, &mRightDistort,
        imgSize, &mRotation, &mTranslation, &mEssential, &mFundamental,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        /*CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH +*/
        CV_CALIB_FIX_K3);
    cout << "Done." << endl;

	//Check the error
	checkError(totalPointCount, mLeftPoints, mRightPoints, mLeftParams, mLeftDistort, mRightParams, mRightDistort, mFundamental);

	//Initialise the stereo parameters
    double R1[3][3], R2[3][3], P1[3][4], P2[3][4], Q[4][4];
    CvMat mLeftRectify = cvMat(3, 3, CV_64F, R1); CvMat mRightRectify = cvMat(3, 3, CV_64F, R2);
	CvMat mLeftProject = cvMat(3, 4, CV_64F, P1); CvMat mRightProject = cvMat(3, 4, CV_64F, P2);
    CvMat mReprojection = cvMat(4, 4, CV_64FC1, Q);

	cvStereoRectify( &mLeftParams, &mRightParams, &mLeftDistort, &mRightDistort, imgSize,	&mRotation, &mTranslation,
		&mLeftRectify, &mRightRectify, &mLeftProject, &mRightProject, &mReprojection, CV_CALIB_ZERO_DISPARITY);

    //Save stereo camera parameters
    CvFileStorage* fstorage = cvOpenFileStorage(outputFilename, NULL, CV_STORAGE_WRITE);
    cvWriteInt(fstorage, "image_width", imgSize.width ); cvWriteInt(fstorage, "image_height", imgSize.height);
	cvWrite(fstorage, "reprojection", &mReprojection);

	//Save left camera parameters
	cvWrite(fstorage, "left_camera_intrinsics", &mLeftParams);
    cvWrite(fstorage, "left_camera_distortion", &mLeftDistort);
	cvWrite(fstorage, "left_camera_rectify", &mLeftRectify);
	cvWrite(fstorage, "left_camera_projection", &mLeftProject);    

	//Save right camera parameters
	cvWrite(fstorage, "right_camera_intrinsics", &mRightParams);
    cvWrite(fstorage, "right_camera_distortion", &mRightDistort);
	cvWrite(fstorage, "right_camera_rectify", &mRightRectify);
	cvWrite(fstorage, "right_camera_projection", &mRightProject);    
	
    cvReleaseFileStorage(&fstorage);

}

/*Compute the error in the calibration*/
void checkError(int totalPointCount, CvMat mLeftPoints, CvMat mRightPoints, CvMat mLeftParams, CvMat mLeftDistort, CvMat mRightParams, CvMat mRightDistort, CvMat mFundamental) {
	//Initialise some space for the epipolar lines
	vector<CvPoint3D32f> lLines; vector<CvPoint3D32f> rLines;
    lLines.resize(totalPointCount); rLines.resize(totalPointCount);
    CvMat mLeftLines = cvMat(1, totalPointCount, CV_32FC3, &lLines[0]);
    CvMat mRightLines = cvMat(1, totalPointCount, CV_32FC3, &rLines[0]);

	//Always work in undistorted space
    cvUndistortPoints( &mLeftPoints, &mLeftPoints, &mLeftParams, &mLeftDistort, 0, &mLeftParams );
    cvUndistortPoints( &mRightPoints, &mRightPoints, &mRightParams, &mRightDistort, 0, &mRightParams );
    cvComputeCorrespondEpilines( &mLeftPoints, 1, &mFundamental, &mLeftLines );
    cvComputeCorrespondEpilines( &mRightPoints, 2, &mFundamental, &mRightLines );
    
	//Compute the error for each point
	double avgErr = 0;
    for( int i = 0; i < totalPointCount; i++ )
    {
        double err = fabs(allLeftCorners[i].x*rLines[i].x +
            allLeftCorners[i].y*rLines[i].y +rLines[i].z)
            + fabs(allRightCorners[i].x*lLines[i].x +
            allRightCorners[i].y*lLines[i].y + lLines[i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(totalPointCount) );
}

//Render two images side by side
void cvShowStereoImage(char *windowTitle, IplImage *frameL, IplImage *frameR) {
	//Create a new image, copy the images side by side, then render
	IplImage *dualImage = cvCreateImage(cvSize(frameL->width + frameR->width, frameL->height<frameR->height?frameR->height:frameL->height), frameL->depth, frameL->nChannels);
	cvSetImageROI(dualImage, cvRect(0,0,frameL->width, frameL->height)); cvCopy(frameL, dualImage);
	cvSetImageROI(dualImage, cvRect(frameL->width,0,frameR->width, frameR->height)); cvCopy(frameR, dualImage);
	cvResetImageROI(dualImage);
	cvNamedWindow(windowTitle); cvShowImage(windowTitle, dualImage);
	cvReleaseImage(&dualImage);
}

/*Calculate the disparity image given the two input images, background images, and reprojection matrix*/
void  renderDisparity(IplImage* new_frameL, IplImage* new_frameR, CvMat* reprojection) {

	//Convert images to black and white
	IplImage *bwFrameL = cvCreateImage(cvGetSize(new_frameL), new_frameL->depth, 1);
	IplImage *bwFrameR = cvCreateImage(cvGetSize(new_frameR), new_frameR->depth, 1);
	cvConvertImage(new_frameL, bwFrameL); cvConvertImage(new_frameR, bwFrameR);
	
	//Calculate Disparity Map
	CvMat* mDisparity = cvCreateMat( new_frameL->height, new_frameL->width, CV_32F );
    cvFindStereoCorrespondenceBM( bwFrameL, bwFrameR, mDisparity, BMState);
	double min, max; cvMinMaxLoc(mDisparity, &min, &max);
	double scale = 255.0/(max-min), shift = min*scale;
	
	//Render Disparity Map to Image
	IplImage* disparityImg = cvCreateImage(cvGetSize(bwFrameL), bwFrameL->depth, bwFrameL->nChannels);
	cvConvertScale(mDisparity, disparityImg, scale, -shift);

	cvNamedWindow("Disparity"); cvShowImage("Disparity", disparityImg);

	//Clean up
	cvReleaseMat(&mDisparity); cvReleaseImage(&disparityImg);
	cvReleaseImage(&bwFrameL); cvReleaseImage(&bwFrameR);
}