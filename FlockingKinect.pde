import org.openkinect.processing.*;
import blobDetection.*;

Kinect kinect;
BlobDetection theBlobDetection;

float minThresh = 430;
float maxThresh = 730;
PImage img;
PImage videoImage;

// Corrections for camera resolutions
float offsetX;
float offsetY;

int BLOBMAX = 10; // Change this # to fix speed rendering
int COUNTER;

Flock flock;

void setup() {
  //size(1280, 360);
  size(640,480);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 250; i++) {
    Boid b = new Boid(width/2,height/2);
    flock.addBoid(b);
  }
  
  // Init the kinect
  kinect = new Kinect(this);
  kinect.setTilt(15);
  kinect.initDepth();
  kinect.initVideo();
  img = createImage(kinect.width, kinect.height, RGB);
  //img = createImage(960, 480, RGB);
  
  // Init the blob detection
  theBlobDetection = new BlobDetection(width, height);
  BlobDetection.setConstants(BLOBMAX, 4000, 500);
  theBlobDetection.setPosDiscrimination(true);
  theBlobDetection.setThreshold(0.70f); // threshold is between 0.0f and 1.0f
  //theBlobDetection.computeBlobs(img.pixels);
}

void draw() {
  background(0);
  
  drawKinect();
  flock.run(theBlobDetection);
  
  // Instructions
  fill(0);
  text("Drag the mouse to generate new boids.",10,height-16);
}

// Add a new boid into the System
void mouseDragged() {
  //flock.addBoid(new Boid(mouseX,mouseY));
}

void drawKinect() {
  img.loadPixels();
  
  int[] depth = kinect.getRawDepth();
  //int position2= 0;
  for (int x = 0; x < kinect.width; x++) {
    for (int y = 0; y < kinect.height; y++) {

      int position = int(x + y * kinect.width);
      int d = depth[position];

      
      offsetX = ((int(x - 15 - width * 0.5) * 241) >> 8) + width * 0.5;
      offsetY = ((int(y + 25 - height * 0.5) * 240) >> 8) + height * 0.5;
      
      position = int(offsetX + offsetY * kinect.width);

      // Threshold checking
      if (position >= 0 && position < img.pixels.length) {
        if (d > minThresh && d < maxThresh) {
          img.pixels[position] = color(255);
        } else {
          img.pixels[position] = color(0);
        }
      }
      
    }
  }

  //img.filter(BLUR, 3);
  img.updatePixels();

  // Get the masked RGB image that's in the threshold range
  videoImage = kinect.getVideoImage().copy();
  videoImage.mask(img);
  
  // Comment/uncomment to either display whole video or masked video
  image(img, 0, 0);
  //image(videoImage, 0, 0);
  
  // Blob - only computes pixels above a certain brightness
  theBlobDetection.computeBlobs(img.pixels);
  drawBlobsAndEdges(false, false);
}

void drawBlobsAndEdges(boolean drawBlobs, boolean drawEdges) {
  noFill();
  Blob b;
  EdgeVertex eA, eB;
  for (int n=0 ; n<theBlobDetection.getBlobNb() ; n++)
  {
    b=theBlobDetection.getBlob(n);
    if (b!=null)
    {
      // Edges
      if (drawEdges)
      {
        strokeWeight(2);
        stroke(0, 255, 0);
        for (int m=0;m<b.getEdgeNb();m++)
        {
          eA = b.getEdgeVertexA(m);
          eB = b.getEdgeVertexB(m);
          if (eA !=null && eB !=null)
            line(
            eA.x*width, eA.y*height, 
            eB.x*width, eB.y*height
              );
        }
      }

      // Blobs
      if (drawBlobs)
      {
        strokeWeight(1);
        stroke(255, 0, 0);
        rect(
        b.xMin*width, b.yMin*height, 
        b.w*width, b.h*height
          );
      }
    }
  }
}