#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
 
using namespace cv;
using namespace std;

int main() {

  Mat image;
  char c = ' ';
  string name = "";
  int image_count = 0;

  VideoCapture cap (0);
  if (!cap.isOpened()) { 
      cout << "Error \n";
      return 0;
    }

  while (true) {
      cap >> image;
      imshow("Video", image);
      c = waitKey(10);
      if (c == ' ') 
      {
            name = "./images/snapped_images/snapped_image_" + std::to_string(image_count) + ".jpg"; // Full name: "snapped_image_1.jpg"
            image_count++; // Image count increases
            cout << name << "\n";
            imwrite(name, image);
      } 
      else if (c == 'q') 
      {              
            cout << "Ecit Successful \n";
            break;  
      } 
    }
  return 0;
}

