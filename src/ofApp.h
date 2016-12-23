#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"

#ifdef OF_TARGET_WINVS
#include "ofxKinectForWindows2.h"
#else
#include "ofxKinect.h"
#endif

class ofApp : public ofBaseApp {
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
#ifdef OF_TARGET_WINVS
    ofxKFW2::Device kinect;
#else
    ofxKinect kinect;
#endif
    
    ofImage bodyIndexImg, medianFilter;
    // Used to store each depth frame
    ofxCvGrayscaleImage grayTemp;
	ofxCvGrayscaleImage	grayCopy;
    ofxCvGrayscaleImage grayImage;
    // Used to store captured depth bg
    ofxCvGrayscaleImage grayBg;
    // Used to store the processed depth image
    ofxCvGrayscaleImage grayDiff;
    
    // Used to find blobs in the filtered depthmap
//    ofxCvContourFinder 	contourFinder;
    
    
    ofxCv::RunningBackground background;
    ofImage thresholded;
    
    ofxPanel gui;
    ofParameter<bool> resetBackground;
    ofParameter<float> learningTime, thresholdValue;
    
    vector<ofVec2f> colorCoords;
    int numBodiesTracked;
    bool bHaveAllStreams;
    
    float resetTime;
	bool	bMirror;
	bool	bFreeze; 
	
    
    vector<ofPolyline> blobs;
    ofxCv::ContourFinder contourFinder;
    ofColor targetColor;
    vector<cv::Point>	points;
    ofParameterGroup	cvParameters;
    ofParameter<int>	threshold;
	ofParameter<int>	cvDiffThreshold;
    ofParameter<bool>	holes;
    ofParameter<bool>	invert;
    ofParameter<int>	resampleMode;
    ofParameter<int>	minArea;
    ofParameter<int>	maxArea;
    ofParameter<float>	mSpacing;
    ofParameter<int>	maxCount;
    ofParameter<int>	blurAmount;
	ofParameter<int>	blurAmount1;
	ofParameter<int>	nDilate0;
	ofParameter<int>	nErode0;
	ofParameter<int>	nDilate1;
	ofParameter<int>	nErode1;

    
    ofParameter<bool> bThreshWithOpenCV;
    ofParameter<int> nearThreshold;
    ofParameter<int> farThreshold;
    
    ofParameterGroup oscParameters;
    ofParameter<int> sendPort;
    ofParameter<string> ipAddress;
    
    vector<float> distance;
    
    ofxOscSender sender;
    
    
    
};
