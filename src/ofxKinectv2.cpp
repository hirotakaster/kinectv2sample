#include "ofxKinectv2.h"

ofxKinectv2::ofxKinectv2(void)
{
}


ofxKinectv2::~ofxKinectv2(void) {
	this->stop();
}


bool ofxKinectv2::setup() {
	HRESULT hResult = S_OK;

	// Open Kinect
	hResult = GetDefaultKinectSensor(&sensor);
	if(FAILED(hResult)){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return false;
	}

	hResult = sensor->Open();
	if(FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return false;
	}

	// Open Source
	hResult = sensor->get_ColorFrameSource( &colorSource );
	if(FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return false;
	}

	hResult = sensor->get_BodyFrameSource( &bodySource );
	if(FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return false;
	}

	hResult = sensor->get_DepthFrameSource( &depthSource );
	if(FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return false;
	}

	hResult = sensor->get_BodyIndexFrameSource( &bodyIndexSource );
	if(FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_BodyIndexFrameSource()" << std::endl;
		return false;
	}

	// Open Reader
	hResult = colorSource->OpenReader( &colorReader );
	if(FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return false;
	}

	hResult = bodySource->OpenReader( &bodyReader );
	if(FAILED(hResult)){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return false;
	}

	hResult = depthSource->OpenReader( &depthReader );
	if(FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return false;
	}

	hResult = bodyIndexSource->OpenReader( &bodyIndexReader );
	if(FAILED(hResult)){
		std::cerr << "Error : IBodyIndexFrameSource::OpenReader()" << std::endl;
		return false;
	}


	// get descriptions
	hResult = depthSource->get_FrameDescription( &depthDescription );
	if(FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return false;
	}

	hResult = colorSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Rgba, &colorDescription );
	if(FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return false;
	}

	hResult = bodyIndexSource->get_FrameDescription( &bodyDescription );
	if(FAILED(hResult)){
		std::cerr << "Error : IBodyIndexFrameSource::get_FrameDescription()" << std::endl;
		return false;
	}

	// get coordinate mapper
	hResult = sensor->get_CoordinateMapper( &coordinateMapper );
	if(FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return false;
	}

	depthDescription->get_Width(&depthWidth);
	depthDescription->get_Height(&depthHeight);
	depthDescription->get_BytesPerPixel(&depthBytesPerPixels);

	colorDescription->get_Width( &colorWidth );
	colorDescription->get_Height( &colorHeight );
	colorDescription->get_BytesPerPixel( &colorBytesPerPixels);

	bodyDescription->get_Width( &bodyWidth );
	bodyDescription->get_Height( &bodyHeight );

	colorCoordinates = new ColorSpacePoint[depthWidth * depthHeight];
	depthBuffer.resize(depthWidth * depthHeight);
	bodyIndexBuffer.resize(bodyWidth * bodyHeight);

	bodyscaleImage.allocate(depthWidth, depthHeight, OF_IMAGE_COLOR_ALPHA);
	colorImage.allocate(colorWidth, colorHeight, OF_IMAGE_COLOR_ALPHA);
	depthImage.allocate(depthWidth, depthHeight, OF_IMAGE_GRAYSCALE);

	return true;
}

void ofxKinectv2::start() {
	startThread();
}

void ofxKinectv2::stop() {
	if (isThreadRunning()) 
		stopThread();
}


void ofxKinectv2::threadedFunction() {
	while (isThreadRunning()) {
		kinectThread();
	}
}

void ofxKinectv2::kinectThread () {

	// get depth info
	ComPtr<IDepthFrame> depthFrame;
	HRESULT result = depthReader->AcquireLatestFrame(&depthFrame);
	if (SUCCEEDED( result )) {
		ofxKinectv2Lock scopedLock(mutex);
		result = depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
		if (SUCCEEDED(result)) {
			for (int i = 0; i < depthBuffer.size(); i++) {
				depthImage.getPixels()[i] = ((depthBuffer[i]*255)/8000);
			}

			// create coordinate mapper
			coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], depthWidth * depthHeight, colorCoordinates);
		}
	}


	// get rgb data
	ComPtr<IColorFrame> colorFrame;
	result = colorReader->AcquireLatestFrame( &colorFrame );
	if (SUCCEEDED(result)) {
		ofxKinectv2Lock scopedLock(mutex);
		result = colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * colorBytesPerPixels, colorImage.getPixels(), ColorImageFormat_Rgba);
	}

	// get body info
	ComPtr<IBodyIndexFrame> bodyIndexFrame;
	result = bodyIndexReader->AcquireLatestFrame(&bodyIndexFrame);
	if (SUCCEEDED(result)) {
		ofxKinectv2Lock scopedLock(mutex);
		result = bodyIndexFrame->CopyFrameDataToArray(bodyIndexBuffer.size(), &bodyIndexBuffer[0]);
		if (SUCCEEDED(result)){
			for (int depthIndex = 0; depthIndex < (bodyHeight * bodyWidth); ++depthIndex) {
				if (bodyIndexBuffer[depthIndex] != 0xff) {
					UINT16 depth = depthBuffer[depthIndex];
					ColorSpacePoint colorPoint = colorCoordinates[depthIndex];
					int colorX = (int)(floor(colorPoint.X));
					int colorY = (int)(floor(colorPoint.Y));
					int colorIndex = colorX + (colorY * colorWidth);
					if ((colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight)) {
						bodyscaleImage.getPixels()[depthIndex * 4 + 0] = colorImage.getPixels()[colorIndex * colorBytesPerPixels + 0];
						bodyscaleImage.getPixels()[depthIndex * 4 + 1] = colorImage.getPixels()[colorIndex * colorBytesPerPixels + 1];
						bodyscaleImage.getPixels()[depthIndex * 4 + 2] = colorImage.getPixels()[colorIndex * colorBytesPerPixels + 2];
						bodyscaleImage.getPixels()[depthIndex * 4 + 3] = 255;
					} else {
						bodyscaleImage.getPixels()[depthIndex * 4 + 0] = 0;
						bodyscaleImage.getPixels()[depthIndex * 4 + 1] = 0;
						bodyscaleImage.getPixels()[depthIndex * 4 + 2] = 0;
						bodyscaleImage.getPixels()[depthIndex * 4 + 3] = 255;
					}
				} else {
					bodyscaleImage.getPixels()[depthIndex * 4 + 0] = 0;
					bodyscaleImage.getPixels()[depthIndex * 4 + 1] = 0;
					bodyscaleImage.getPixels()[depthIndex * 4 + 2] = 0;
					bodyscaleImage.getPixels()[depthIndex * 4 + 3] = 255;
				}
			}
		}
	}

	// get body frame
	ComPtr<IBodyFrame> bodyFrame;
	result = bodyReader->AcquireLatestFrame( &bodyFrame );
	if (SUCCEEDED(result)) {
		IBody* pBody[BODY_COUNT] = { 0 };
		result = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
		if (SUCCEEDED(result)) {
			ofxKinectv2Lock scopedLock(mutex);
			jointCountList.clear();
			for( int count = 0; count < BODY_COUNT; count++ ){
				BOOLEAN bTracked = false;
				result = pBody[count]->get_IsTracked( &bTracked );
				if (SUCCEEDED(result) && bTracked) {
					Joint joint[JointType::JointType_Count];
					result = pBody[count]->GetJoints( JointType::JointType_Count, joint );
					if (SUCCEEDED(result)) {
						jointCountList.push_back(JointUser(count));

						// Joint
						for( int type = 0; type < JointType::JointType_Count; type++ ){
							jointCountList.back().joint[type] = joint[type];
						}

						// Hand State
						pBody[count]->get_HandLeftState(&jointCountList.back().leftHandState);
						pBody[count]->get_HandRightState(&jointCountList.back().rightHandState);
					}
				}
			}
		}
	}
}


ofImage ofxKinectv2::getBodyScaleImage() {
	ofxKinectv2Lock scopedLock(mutex);
	return bodyscaleImage;
};

std::vector<UINT16> ofxKinectv2::getDepthBuffer() {
	ofxKinectv2Lock scopedLock(mutex);
	return depthBuffer;
}

std::vector<JointUser> ofxKinectv2::getJointList() {
	ofxKinectv2Lock scopedLock(mutex);
	return jointCountList;
}

ofImage ofxKinectv2::getColorImage() {
	ofxKinectv2Lock scopedLock(mutex);
	return this->colorImage;
}

ofImage ofxKinectv2::getDepthImage() {
	ofxKinectv2Lock scopedLock(mutex);
	return this->depthImage;
}

