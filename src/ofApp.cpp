// This example shows how to work with the BodyIndex image in order to create
// a green screen effect. Note that this isn't super fast, but is helpful
// in understanding how the different image types & coordinate spaces work
// together. If you need performance, you will probably want to do this with shaders!

#include "ofApp.h"

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define DEPTH_SIZE DEPTH_WIDTH * DEPTH_HEIGHT


//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowShape(DEPTH_WIDTH * 2, DEPTH_HEIGHT);
#ifdef OF_TARGET_WINVS
	kinect.open();
	kinect.initDepthSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();
#else
	kinect.init(true, false, false);
	kinect.open();		// opens first available kinect
#endif
	
	
	numBodiesTracked = 0;
	bHaveAllStreams = false;
	
	bodyIndexImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
	medianFilter.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
	colorCoords.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	
	
	gui.setup();
	gui.setPosition(DEPTH_WIDTH+10, 10);
	
	cvParameters.setName("Contour Finder");
	cvParameters.add(threshold.set("Contour Finder Threshold", 128, 0, 255));
	cvParameters.add(holes.set("Send Holes", false));
	cvParameters.add(cvDiffThreshold.set("cvDiffThreshold",2, 1, 255));
	cvParameters.add(invert.set("invert Contour Finder", false));
	cvParameters.add(maxArea.set("maxArea", 150, 0, 2*DEPTH_SIZE/3));
	cvParameters.add(minArea.set("minArea", 10, 0, DEPTH_SIZE/16));
	
	cvParameters.add(farThreshold.set("farThreshold", 0, 0, 10000));
	cvParameters.add(nearThreshold.set("nearThreshold", 0, 0, 10000));
	cvParameters.add(blurAmount.set("blurAmount", 2, 0, 10));
	
	cvParameters.add(resetBackground.set("Reset Background", true));
	cvParameters.add(learningTime.set("Learning Time", 10, 0, 30));
	cvParameters.add(thresholdValue.set("Background Threshold Value", 10, 0, 255));
	
	cvParameters.add(blurAmount1.set("blurAmount1", 2, 0, 10));
	cvParameters.add(nErode0.set("nErode0",   1, 0,5));
	cvParameters.add(nDilate0.set("nDilate0", 1, 0,5));
	cvParameters.add(nErode1.set("nErode1",   1, 0,5));
	cvParameters.add(nDilate1.set("nDilate1", 1, 0,5));
	
	
	gui.add(cvParameters);
	
	nearThreshold = 0;
	farThreshold = 255;
	bMirror = false;
	bFreeze = false;
	
	resetBackground = true;
	
	
	oscParameters.setName("OSC Settings");
	oscParameters.add(sendPort.set("Send Port", 6667, 6666, 7777));
	oscParameters.add(ipAddress.set("ipAddress", "192.168.1.255"));
	gui.add(oscParameters);
	
	gui.setSize(300, 400);
	gui.setWidthElements(300);
	gui.loadFromFile(ofToDataPath("settings.xml"));
	
	sender.setup(ipAddress.get(), sendPort.get());
	
	grayImage.allocate(kinect.width, kinect.height);
	grayBg.allocate(kinect.width, kinect.height);
	grayDiff.allocate(kinect.width, kinect.height);
	grayTemp.allocate(kinect.width, kinect.height);
	
	thresholded.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.setDepthClipping(nearThreshold, farThreshold);
	kinect.update();
	
#ifdef OF_TARGET_WINVS
	// WINDOWS USING KINECT V2:
	auto& bodyIndexPix = kinect.getBodyIndexSource()->getPixels();
	
	// Make sure there's some data here, otherwise the cam probably isn't ready yet
	if (!bodyIndexPix.size()) {
		bHaveAllStreams = false;
		return;
	}
	else {
		bHaveAllStreams = true;
	}
	
	// Count number of tracked bodies
	numBodiesTracked = 0;
	auto& bodies = kinect.getBodySource()->getBodies();
	for (auto& body : bodies) {
		if (body.tracked) {
			numBodiesTracked++;
		}
	}
	
	// Loop through the depth image
	for (int y = 0; y < DEPTH_HEIGHT; y++) {
		for (int x = 0; x < DEPTH_WIDTH; x++) {
			int index = (y * DEPTH_WIDTH) + x;
			bodyIndexImg.setColor(x, y, ofColor::white);
			
			// This is the check to see if a given pixel is inside a tracked
			// body or part of the background. If it's part of a body,
			// the value will be that body's id (0-5), or will > 5 if it's
			// part of the background
			// More info here:
			// https://msdn.microsoft.com/en-us/library/windowspreview.kinect.bodyindexframe.aspx
			float val = bodyIndexPix[index];
			if (val >= bodies.size()) {
				continue;
			}
			
			// Give each tracked body a color value so we can tell
			// them apart on screen
			//	ofColor c = ofColor::fromHsb(val * 255 / bodies.size(), 200, 255);
			bodyIndexImg.setColor(x, y, ofColor::black);
		}
	}
	
	// Update the images since we manipulated the pixels manually. This uploads to the
	// pixel data to the texture on the GPU so it can get drawn to screen
	bodyIndexImg.update();
	
	if (numBodiesTracked > 0) {
		//run through ofxCV thresholding
		contourFinder.setInvert(invert);
		contourFinder.setThreshold(threshold);
		contourFinder.setFindHoles(holes);
		contourFinder.setMinArea(minArea);
		contourFinder.setMaxArea(maxArea);
		contourFinder.findContours(bodyIndexImg);
	}
	
#else
	// MACOS USING KINECTV1
	
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
	
		
		if (bFreeze){
			grayImage = grayCopy;
		} else {
			grayImage.setFromPixels(kinect.getDepthPixels());
		}
		if (bMirror){
			grayImage.mirror(false, true);
		}
		
		// Quick and dirty noise filter on the depth map. Needs work
		grayImage.erode();
		grayImage.dilate();
		grayImage.blur(blurAmount);
		
		// If the user pressed spacebar, capture the depth image, and save for later
		if (resetBackground){
			resetTime = ofGetElapsedTimef();
			resetBackground = false;
		}
		
		if((ofGetElapsedTimef() - resetTime) < learningTime){
			grayTemp = grayImage;
			ofImage foo;
			
			foo.setFromPixels(grayTemp.getPixels());
			background.setLearningTime(learningTime);
			background.setThresholdValue(thresholdValue);
			background.update(grayTemp, thresholded);
			ofxCv::toOf(background.getBackground(), foo);
			grayBg.setFromPixels(foo.getPixels());
			
			grayBg.erode();
			grayBg.dilate();
		}
		
		// Copy the current depth image (grayImage) into grayDiff
		grayDiff = grayImage;
		
		// Subtract the bg, producing a depth-difference image in graydiff.
		grayDiff -= grayBg;
		
		// anything that is > 1 has changed, so keep it
		grayDiff.blur(blurAmount1);
		grayDiff.threshold(cvDiffThreshold);
		for (int i=0; i<nErode0; i++){
			grayDiff.erode();
		}
		for (int i=0; i<nDilate0; i++){
			grayDiff.dilate();
		}
		for (int i=0; i<nErode1; i++){
			grayDiff.erode();
		}
		for (int i=0; i<nDilate1; i++){
			grayDiff.dilate();
		}
		
		
		
		// multiply in the current depth values, to mask it
		grayDiff *= grayImage;
		
		contourFinder.setInvert(invert);
		contourFinder.setThreshold(threshold);
		contourFinder.setFindHoles(holes);
		contourFinder.setMinArea(minArea);
		contourFinder.setMaxArea(maxArea);
		contourFinder.findContours(grayDiff.getCvImage());

	}
	
#endif
	
	
	vector<vector<cv::Point>> contours = contourFinder.getContours();
	int blobCount = 0;
	for (vector<cv::Point> p : contours) {
		if (p.size() > 2) {
			ofxOscMessage b;
			b.setAddress("/blob");
			ofBuffer buffer;
			buffer.allocate(p.size() * sizeof(cv::Point));
			memcpy(buffer.getData(), &p[0], buffer.size());
			
			points.clear();
			int size = buffer.size()/sizeof(cv::Point);
			char *tempData = buffer.getData();
			points.resize(size);
			memcpy(&points[0], tempData, buffer.size());
			
			b.addBlobArg(buffer);
			b.addIntArg(contourFinder.getHole(blobCount)?1:0);
			
			sender.sendMessage(b);
		}
		blobCount++;
	}
	
	ofxOscMessage b;
	b.setAddress("/metadata/kinect");
	b.addIntArg((int)kinect.getWidth());
	b.addIntArg((int)kinect.getHeight());
	sender.sendMessage(b);
	
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255, 255, 255);
	grayDiff.draw(0, 0);
	
	grayBg.draw(DEPTH_WIDTH, 0);
	ofPushMatrix();
	ofSetColor(255, 0, 255);
	contourFinder.draw();
	ofPopMatrix();
	
	ofPushMatrix();
	ofSetColor(255, 255, 0);
	ofPolyline line;
	for (int i = 0; i < points.size(); i++) {
		line.addVertex(ofxCv::toOf(points[i]));
	}
	line.draw();
	ofPopMatrix();
	
	ofPushMatrix();
	ofTranslate(DEPTH_WIDTH, DEPTH_HEIGHT);
	
	for (ofPolyline p:blobs) {
		ofSetColor(255, 255, 0);
		p.draw();
	}
	
	ofPopMatrix();
	stringstream ss;
	ss << "fps : " << ofGetFrameRate() << endl;
	
#ifdef OF_TARGET_WINVS
	ss << "Tracked bodies: " << numBodiesTracked;
	if (!bHaveAllStreams) ss << endl << "Not all streams detected!";
#endif
	
	ofDrawBitmapStringHighlight(ss.str(), 20, 20);
	
	gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key){
		case 'm':
			bMirror = !bMirror;
			break;
		case ' ':
			resetBackground = true;
			break;
		case 'f':
			bFreeze = !bFreeze;
			if (bFreeze){
				grayCopy.setFromPixels(kinect.getDepthPixels());
			}
			break;
			
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
	
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {
	
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {
	
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
	
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {
	
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {
	
}
