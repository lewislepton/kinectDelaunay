#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
  kinect.init();
  kinect.open();
  
  color.allocate(kinect.getWidth(), kinect.getHeight());
  grayscale.allocate(kinect.getWidth(), kinect.getHeight());
  background.allocate(kinect.getWidth(), kinect.getHeight());
  difference.allocate(kinect.getWidth(), kinect.getHeight());
  
  imgBlob.allocate(kinect.getWidth(), kinect.getHeight(),OF_IMAGE_GRAYSCALE);
  
  gui.setup();
  gui.setPosition(ofPoint(10,10));
  gui.add(noiseAmount.set("Noise Amount", 0.0, 0.0,20.0));
  gui.add(pointSkip.set("Point Skip", 3, 3,20));
  gui.add(useRealColors.setup("Real Colors", false));
  gui.add(colorAlpha.set("Color Alpha", 255,0,255));
}

//--------------------------------------------------------------
void ofApp::update(){
  kinect.update();
  
//  if (kinect.isFrameNew()){
//    delaunay.reset();
//
//    color.setFromPixels(kinect.getPixels());
//    grayscale = color;
//    difference.absDiff(background, grayscale);
//    difference.threshold(60);
//
//    int minimum = difference.getWidth() * difference.getHeight() * 0.05;
//    int maximum = difference.getWidth() * difference.getHeight() * 0.8;
//    contour.findContours(difference, minimum, maximum, 1, false);
//  }
  
  if(kinect.isFrameNew()) {
    delaunay.reset();
    
    
    unsigned char* pix = new unsigned char[640*480];
    
    unsigned char* gpix = new unsigned char[640*480];
    
    for(int x=0;x<640;x+=1) {
      for(int y=0;y<480;y+=1) {
        float distance = kinect.getDistanceAt(x, y);
        
        int pIndex = x + y * 640;
        pix[pIndex] = 0;
        
        if(distance > 100 && distance < 1100) {
          pix[pIndex] = 255;
        }
        
      }
    }
    
    imgBlob.setFromPixels(pix, 640, 480, OF_IMAGE_GRAYSCALE);
    
    int numPoints = 0;
    
    for(int x=0;x<640;x+=pointSkip*2) {
      for(int y=0;y<480;y+=pointSkip*2) {
        int pIndex = x + 640 * y;
        
        if(imgBlob.getPixels()[pIndex]> 0) {
          ofVec3f wc = kinect.getWorldCoordinateAt(x, y);
          
          wc.x = x - 320.0;
          wc.y = y - 240.0;
          
          if(abs(wc.z) > 100 && abs(wc.z ) < 2000) {
            
            wc.z = -wc.z;
            
            wc.x += ofSignedNoise(wc.x,wc.z)*noiseAmount;
            wc.y += ofSignedNoise(wc.y,wc.z)*noiseAmount;
            
            wc.x = ofClamp(wc.x, -320,320);
            wc.y = ofClamp(wc.y, -240,240);
            
            delaunay.addPoint(wc);
          }
          numPoints++;
        }
        
      }
    }
    
    
    if(numPoints >0)
      delaunay.triangulate();
    
    for(int i=0;i<delaunay.triangleMesh.getNumVertices();i++) {
      delaunay.triangleMesh.addColor(ofColor(0,0,0));
    }
    
//    for(int i=0;i<delaunay.triangleMesh.getNumIndices()/3;i+=1) {
//      ofVec3f v = delaunay.triangleMesh.getVertex(delaunay.triangleMesh.getIndex(i*3));
//
//      v.x = ofClamp(v.x, -319,319);
//      v.y = ofClamp(v.y, -239, 239);
//
//      ofColor c = kinect.getColorAt(v.x+320.0, v.y+240.0);
//
//      if(!useRealColors)
//        c = ofColor(255,0,0);
//
//      c.a = colorAlpha;
//
//      delaunay.triangleMesh.setColor(delaunay.triangleMesh.getIndex(i*3),c);
//      delaunay.triangleMesh.setColor(delaunay.triangleMesh.getIndex(i*3+1),c);
//      delaunay.triangleMesh.setColor(delaunay.triangleMesh.getIndex(i*3+2),c);
//    }
    
    
    
    
    mshConverted.clear();
    mshWireframe.clear();
    mshWireframe.setMode(OF_PRIMITIVE_TRIANGLES);
    for(int i=0;i<delaunay.triangleMesh.getNumIndices()/3;i+=1) {
      
      ofVec3f v = delaunay.triangleMesh.getVertex(delaunay.triangleMesh.getIndex(i*3));
      
      v.x = ofClamp(v.x, -319,319);
      v.y = ofClamp(v.y, -239, 239);
      
      ofColor c = kinect.getColorAt(v.x+320.0, v.y+240.0);
      
      if(!useRealColors){
        c = ofColor(255,0,0);
      }
      
      c.a = colorAlpha;
      
      delaunay.triangleMesh.setColor(delaunay.triangleMesh.getIndex(i*3),c);
      delaunay.triangleMesh.setColor(delaunay.triangleMesh.getIndex(i*3+1),c);
      delaunay.triangleMesh.setColor(delaunay.triangleMesh.getIndex(i*3+2),c);

      int indx1 = delaunay.triangleMesh.getIndex(i*3);
      ofVec3f p1 = delaunay.triangleMesh.getVertex(indx1);
      int indx2 = delaunay.triangleMesh.getIndex(i*3+1);
      ofVec3f p2 = delaunay.triangleMesh.getVertex(indx2);
      int indx3 = delaunay.triangleMesh.getIndex(i*3+2);
      ofVec3f p3 = delaunay.triangleMesh.getVertex(indx3);

      ofVec3f triangleCenter = (p1+p2+p3)/3.0;
      triangleCenter.x += 320;
      triangleCenter.y += 240;

      triangleCenter.x = floor(ofClamp(triangleCenter.x, 0,640));
      triangleCenter.y = floor(ofClamp(triangleCenter.y, 0, 480));

      int pixIndex = triangleCenter.x + triangleCenter.y * 640;
      if(pix[pixIndex] > 0) {

        mshConverted.addVertex(p1);
        mshConverted.addColor(delaunay.triangleMesh.getColor(indx1));

        mshConverted.addVertex(p2);
        mshConverted.addColor(delaunay.triangleMesh.getColor(indx2));

        mshConverted.addVertex(p3);
        mshConverted.addColor(delaunay.triangleMesh.getColor(indx3));

        mshWireframe.addVertex(p1);
        mshWireframe.addVertex(p2);
        mshWireframe.addVertex(p3);
      }
    }
    
//    delete pix;
//    delete gpix;
    
  }
}

//--------------------------------------------------------------
void ofApp::draw(){
  ofBackground(219, 214, 217);
  //glEnable(GL_DEPTH_TEST);
  
  ofPushMatrix();
  
  cam.begin();
  cam.setScale(1,-1,1);
  
  ofSetColor(255,255,255);
  ofTranslate(0, -80,1100);
  ofFill();
  
//  postFx.begin();
  
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glShadeModel(GL_FLAT);
  glProvokingVertex(GL_FIRST_VERTEX_CONVENTION);
  mshConverted.drawFaces();
  glShadeModel(GL_SMOOTH);
  glPopAttrib();
  
  if(useRealColors) {
    ofSetColor(30,30,30, 255);
  } else
    ofSetColor(124,136,128,255);
  
  ofPushMatrix();
  ofTranslate(0, 0,0.5);
  mshWireframe.drawWireframe();
  ofPopMatrix();
  cam.end();
  ofPopMatrix();
  
//  postFx.end();
  

    
    ofPushStyle();
    ofSetColor(255,255,255,255);
    gui.draw();
    ofPopStyle();
  ofSetColor(255, 255, 255);
}

//--------------------------------------------------------------
void ofApp::exit(){
  kinect.close();
  kinect.setCameraTiltAngle(0);
}

//--------------------------------------------------------------
void ofApp::createContour(ofPoint point){
  delaunay.addPoint(point);
  delaunay.triangulate();
  
//  delete contourEdge;
//  contourEdge = new ofxBox2dEdge();
//  contourEdge->addVertexes(polyline);
//  contourEdge->create(box2d.getWorld());
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
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

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
