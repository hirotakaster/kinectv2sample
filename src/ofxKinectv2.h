#pragma once

#include <Kinect.h>
#include <Windows.h>
#include "ofImage.h"
#include "ofThread.h"

template<typename T>
class ComPtr {
private:
	T* ptr;

public:
	ComPtr() {
		ptr = nullptr;
	}

	~ComPtr() {
		if (ptr != nullptr) {
			ptr->Release();
			ptr = nullptr;
		}
	}

	T** operator &() {
		return &ptr;
	}

	T* operator -> () {
		return ptr;
	};

	operator T* () {
		return ptr;
	}
};

// for tracking user info
class JointUser {
public:
	int idx; // UserIdx
	Joint joint[JointType::JointType_Count];
	HandState leftHandState;
	HandState rightHandState;
	JointUser(int idx) { this->idx = idx; };
};

// for scope lock
class ofxKinectv2Lock {
public:
	ofxKinectv2Lock (ofMutex & _mutex) : mutex(_mutex) {
		mutex.lock();
	};

	~ofxKinectv2Lock() {
		mutex.unlock();
	};

	ofMutex & mutex;
};


class ofxKinectv2 : public ofThread {
public:
	// Kinect Classes
	IKinectSensor* sensor;
	IDepthFrameSource* depthSource;
	IColorFrameSource* colorSource;
	IBodyFrameSource* bodySource;
	IBodyIndexFrameSource* bodyIndexSource;


	IColorFrameReader* colorReader;
	IBodyFrameReader* bodyReader;
	IBodyIndexFrameReader* bodyIndexReader;
	IDepthFrameReader* depthReader;

	IFrameDescription* depthDescription;
	IFrameDescription* colorDescription;
	IFrameDescription* bodyDescription;

	ICoordinateMapper* coordinateMapper;
	ColorSpacePoint*   colorCoordinates;

	// ImageData
	ofImage bodyscaleImage;
	ofImage colorImage;
	ofImage depthImage;

	// Joint Data
	std::vector<JointUser> jointCountList;

	// width, height
	int depthWidth, depthHeight;

	int colorWidth, colorHeight;
	int bodyWidth, bodyHeight;
	unsigned int colorBufferSize;
	unsigned int colorBytesPerPixels;
	unsigned int depthBytesPerPixels;

	// Buffer
	std::vector<UINT16> depthBuffer;
	std::vector<BYTE> bodyIndexBuffer;

	// Get Images
	ofImage getBodyScaleImage();
	ofImage getColorImage();
	ofImage getDepthImage();

	std::vector<UINT16> getDepthBuffer();
	std::vector<JointUser> getJointList();

protected:
	void threadedFunction();
	void kinectThread();

public:
	ofxKinectv2();
	~ofxKinectv2();
	bool setup();
	void start();
	void stop();
};
