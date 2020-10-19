#pragma once

#include "ofMain.h"
#include "ofxDelaunay.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
  
  void exit();
  
  void createContour(ofPoint point);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
  
  ofxDelaunay delaunay;
  
  ofxKinect kinect;
  
  ofEasyCam cam;
  
  ofVideoGrabber webcam;
  
  ofxCvColorImage color;
  ofxCvGrayscaleImage grayscale;
  ofxCvGrayscaleImage background;
  ofxCvGrayscaleImage difference;
  ofxCvContourFinder contour;
  
  ofMesh mshConverted;
  ofMesh mshWireframe;
  
  ofPolyline * lines;
  
  ofImage imgBlob;
  
  ofxPanel gui;
  ofParameter <int> colorAlpha;
  ofParameter <float> noiseAmount;
  ofxToggle useRealColors;
  ofParameter <int> pointSkip;
		
};
