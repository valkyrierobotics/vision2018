
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <chrono>

int main()
{
	cv::VideoCapture camera (0);

	cv::Mat img;
	double fps = 0;

	const std::string FPS_FILE = "logs/fps.log";
	std::ofstream fpsFile;
	fpsFile.open(FPS_FILE.c_str(), std::ios::out | std::ios::app);

	std::chrono::high_resolution_clock::time_point start, end;
	while (true)
	{
        	start = std::chrono::high_resolution_clock::now();

		camera >> img;
		cv::imshow("test", img);
		cv::waitKey(10);

		end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> timeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
		fps = 1.0 / timeElapsed.count();
		std::string gnuplotBuf = std::to_string(fps);
		fpsFile << gnuplotBuf.c_str() << std::endl;
	}
	return 0;
}

