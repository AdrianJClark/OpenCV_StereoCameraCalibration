#include "videoInput.h"
#include <string>
#include <iostream>

using namespace std;

/*The camera class wrapper for the Stereo Camera*/
class CameraStereo {
public:
	CameraStereo() {
		lCamera = new Camera(0); rCamera = new Camera(1);
	}

	CameraStereo(char* parameterFile) {
		captureSize = loadCaptureSize(parameterFile);
		lCamera = new Camera(0, captureSize); rCamera = new Camera(1, captureSize);
		loadCaptureParams(parameterFile);
	}

	CameraStereo(int cam1Index, int cam2Index) {
		lCamera = new Camera(cam1Index); rCamera = new Camera(cam2Index);
	}

	CameraStereo(int cam1Index, int cam2Index, char* parameterFile) {
		captureSize = loadCaptureSize(parameterFile);
		lCamera = new Camera(cam1Index, captureSize); rCamera = new Camera(cam2Index, captureSize);
		loadCaptureParams(parameterFile);
	}

	CameraStereo(int cam1Index, int cam2Index, CvSize capSize) {
		captureSize = capSize;
		lCamera = new Camera(cam1Index, captureSize); rCamera = new Camera(cam2Index, captureSize);
	}

	/*Clean everything up*/
	~CameraStereo() {
		delete lCamera; delete rCamera;
	}

	/*Return the reprojection matrix*/
	CvMat* getReprojection() {
		return camReprojection;
	}

	IplImage* getLeftFrame() {
		IplImage* newFrame = cvCreateImage(captureSize, IPL_DEPTH_8U, 3);
		if (lCamera->vi) newFrame->imageData = (char*)lCamera->vi->getPixels(lCamera->camIndex, false, true);
		if (lCamera->distortX && lCamera->distortY) {
			IplImage *unDistortedFrame = cvCreateImage(captureSize, newFrame->depth, newFrame->nChannels);
			cvRemap( newFrame, unDistortedFrame, lCamera->distortX, lCamera->distortY);
			cvReleaseImage(&newFrame); newFrame = unDistortedFrame;
		}
		return newFrame;
	}

	IplImage* getRightFrame() {
		IplImage* newFrame = cvCreateImage(captureSize, IPL_DEPTH_8U, 3);
		if (rCamera->vi) newFrame->imageData = (char*)rCamera->vi->getPixels(rCamera->camIndex, false, true);
		if (rCamera->distortX && rCamera->distortY) {
			IplImage *unDistortedFrame = cvCreateImage(captureSize, newFrame->depth, newFrame->nChannels);
			cvRemap( newFrame, unDistortedFrame, rCamera->distortX, rCamera->distortY);
			cvReleaseImage(&newFrame); newFrame = unDistortedFrame;
		}
		return newFrame;
	}


protected:
	/*Load the calibration file*/
	void loadCaptureParams(string stereo_camera_params) {
		//Create some storage and open the file
		CvMemStorage* storage = cvCreateMemStorage();
		CvFileStorage* fstorage = cvOpenFileStorage(stereo_camera_params.c_str(), storage, CV_STORAGE_READ);
		
		//If something went wrong, print an error message
		if(!fstorage) {
			cerr << "Failed to open calibration file " << stereo_camera_params.c_str() << endl;
			return;
		}
		
		//Load the information out of the file
		captureSize.width = cvReadIntByName(fstorage, NULL, "image_width");
		captureSize.height = cvReadIntByName(fstorage, NULL, "image_height");
		camReprojection = (CvMat*)cvReadByName(fstorage, NULL, "reprojection");

		//Load the left camera's parameters
		lCamera->captureParams = (CvMat*)cvReadByName(fstorage, NULL, "left_camera_intrinsics");
		lCamera->captureDistortion  = (CvMat*)cvReadByName(fstorage, NULL, "left_camera_distortion");
		lCamera->camRectify  = (CvMat*)cvReadByName(fstorage, NULL, "left_camera_rectify");
		lCamera->camProjection  = (CvMat*)cvReadByName(fstorage, NULL, "left_camera_projection");

		//Load the right camera's parameters
		rCamera->captureParams = (CvMat*)cvReadByName(fstorage, NULL, "right_camera_intrinsics");
		rCamera->captureDistortion = (CvMat*)cvReadByName(fstorage, NULL, "right_camera_distortion");
		rCamera->camRectify = (CvMat*)cvReadByName(fstorage, NULL, "right_camera_rectify");
		rCamera->camProjection = (CvMat*)cvReadByName(fstorage, NULL, "right_camera_projection");

		//If any of the matrices failed to load
		if(!lCamera->captureParams || !lCamera->captureDistortion || !lCamera->camRectify || !lCamera->camProjection) {
			// print an error message
			cerr << "Failed to read intrinsic parameters from " << stereo_camera_params.c_str() << endl;
			return;
		} else {
			//Otherwise initialise the undistortion maps
			lCamera->distortX = cvCreateMat(captureSize.height, captureSize.width, CV_32F); lCamera->distortY = cvCreateMat(captureSize.height, captureSize.width, CV_32F);
			cvInitUndistortRectifyMap(lCamera->captureParams, lCamera->captureDistortion, lCamera->camRectify, lCamera->camProjection, lCamera->distortX, lCamera->distortY);
		}

		//If any of the matrices failed to load
		if(!rCamera->captureParams || !rCamera->captureDistortion || !rCamera->camRectify || !rCamera->camProjection) {
			// print an error message
			cerr << "Failed to read intrinsic parameters from " << stereo_camera_params.c_str() << endl;
			return;
		} else {
			//Otherwise initialise the undistortion maps
			rCamera->distortX = cvCreateMat(captureSize.height, captureSize.width, CV_32F); rCamera->distortY = cvCreateMat(captureSize.height, captureSize.width, CV_32F);
			cvInitUndistortRectifyMap(rCamera->captureParams, rCamera->captureDistortion, rCamera->camRectify, rCamera->camProjection, rCamera->distortX, rCamera->distortY);
		}

		//Clean up
		cvReleaseFileStorage(&fstorage);
		cvReleaseMemStorage(&storage);
	} 

	CvSize loadCaptureSize(string stereo_camera_params) {
		//Create some storage and open the file
		CvMemStorage* storage = cvCreateMemStorage();
		CvFileStorage* fstorage = cvOpenFileStorage(stereo_camera_params.c_str(), storage, CV_STORAGE_READ);
		
		//If something went wrong, print an error message
		if(!fstorage) {
			cerr << "Failed to open calibration file " << stereo_camera_params.c_str() << endl;
			return cvSize(-1,-1);
		}
		
		//Load the information out of the file
		int width = cvReadIntByName(fstorage, NULL, "image_width");
		int height = cvReadIntByName(fstorage, NULL, "image_height");

		//Clean up
		cvReleaseFileStorage(&fstorage);
		cvReleaseMemStorage(&storage);

		return cvSize(width, height);
	}

private:
	CvSize captureSize;

	struct Camera {
		Camera(int index) { init(index); }
		Camera(int index, CvSize size) { init(index, size); }

		~Camera() { 
			delete vi;
			if (captureParams) cvReleaseMat(&captureParams); captureParams = 0;
			if (captureDistortion) cvReleaseMat(&captureDistortion); captureDistortion = 0;
			if (camRectify) cvReleaseMat(&camRectify); camRectify = 0;
			if (camProjection) cvReleaseMat(&camProjection); camProjection = 0;
			if (distortX) cvReleaseMat(&distortX); distortX = 0;
			if (distortY) cvReleaseMat(&distortY); distortY = 0;
		}

		void init(int cameraIndex, CvSize imgSize=cvSize(-1,-1)) {
			//Set the camera index
			camIndex = cameraIndex;

			vi = new videoInput(); vi->setVerbose(false); vi->setUseCallback(true);
			if (imgSize.width>0 && imgSize.height >0) {
				vi->setupDevice(cameraIndex, imgSize.width, imgSize.height);
			} else {
				vi->setupDevice(cameraIndex);
			}

			captureParams = captureDistortion = camRectify = camProjection = distortX = distortY = 0;
		}

		videoInput *vi; int camIndex;
		CvMat *captureParams, *captureDistortion, *camRectify, *camProjection, *distortX, *distortY;
		
	};

	Camera *lCamera, *rCamera;

	CvMat *camReprojection;

		


};