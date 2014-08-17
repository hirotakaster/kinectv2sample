#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	ofBackground(0,0,0);

	if (!this->kinect.setup())
		exit();
	this->kinect.start();
}

//--------------------------------------------------------------
void ofApp::update(){
	depth = this->kinect.getDepthImage();
	body = this->kinect.getBodyScaleImage();
	color = this->kinect.getColorImage();

	// get joint points
	points.clear();
	std::vector<JointUser> usr = this->kinect.getJointList();
	for (int i = 0; i < usr.size(); i++) {
		for( int type = 0; type < JointType::JointType_Count; type++ ){
			ColorSpacePoint colorSpacePoint = { 0 };
			this->kinect.coordinateMapper->MapCameraPointToColorSpace(usr[i].joint[type].Position, &colorSpacePoint);
			int x = static_cast<int>(colorSpacePoint.X);
			int y = static_cast<int>(colorSpacePoint.Y);
			if((x >= 0) && (x < kinect.colorWidth) && (y >= 0) && (y < kinect.colorHeight)){
				points.push_back(ofPoint(x, y));
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	color.draw(0, 0, color.getWidth(), color.getHeight());
	depth.draw(0, 0, depth.getWidth(), depth.getHeight());
	body.draw(0, depth.getHeight(), body.getWidth(), body.getHeight());
	
	// draw joint points
	for (int i = 0; i < points.size(); i++)
		ofCircle(points[i], 2);
}

//--------------------------------------------------------------
void ofApp::exit() {
	this->kinect.stop();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
