#include "utils/enumCvType.hpp"

std::string enumCvType(cv::Mat& img)
{	
    std::string imgTypeString;
	switch ( img.type() % 8 )
    {
	    case 0:
	    	imgTypeString = "8U";
	    	break;
	    case 1:
	    	imgTypeString = "8S";
	    	break;
	    case 2:
	    	imgTypeString = "16U";
	    	break;
	    case 3:
	    	imgTypeString = "16S";
	    	break;
	    case 4:
	    	imgTypeString = "32S";
	    	break;
	    case 5:
	    	imgTypeString = "32F";
	    	break;
	    case 6:
	    	imgTypeString = "64F";
	    	break;
	    default:
	    	break;
    }

	int channel = (img.type() / 8) + 1; // Find channel
	std::stringstream ss;
	ss << channel;
	std::string str = ss.str();
	return "CV_" + imgTypeString + "C" + str;
}

