#include "getAngles.hpp"
#include "vector_ordering.hpp"
#include <iostream>
#include <vector>

void test_angles()
{
  assert (atan_wrt_x_axis(10, 10) == 45);
  assert (atan_wrt_x_axis(10, -10) == 135);
  assert (atan_wrt_x_axis(-10, -10) == 225);
  assert (atan_wrt_x_axis(-10, 10) == 315);

  assert (angle_wrt_x_axis(cv::Point2f(10, 10), cv::Point2f(0, 0)) == 45);
  cv::Point2f origin (100, 100);
  assert (angle_wrt_x_axis(cv::Point2f(origin.x + 10, origin.y + 10), origin) == 45);
  assert (angle_wrt_x_axis(cv::Point2f(origin.x - 10, origin.y + 10), origin) == 135);
  assert (angle_wrt_x_axis(cv::Point2f(origin.x - 10, origin.y - 10), origin) == 225);
  assert (angle_wrt_x_axis(cv::Point2f(origin.x + 10, origin.y - 10), origin) == 315);

  std::vector<size_t> test_idx = {2, 1, 3, 5, 4, 0};

  std::vector<cv::Point> test_value;
  for (size_t i = 0; i < 6; ++i)
    test_value.push_back(cv::Point2f(i, i));

  std::vector<cv::Point> test_answer;
  test_answer.push_back(cv::Point2f(5, 5));
  test_answer.push_back(cv::Point2f(1, 1));
  test_answer.push_back(cv::Point2f(0, 0));
  test_answer.push_back(cv::Point2f(2, 2));
  test_answer.push_back(cv::Point2f(4, 4));
  test_answer.push_back(cv::Point2f(3, 3));

  reorder_destructive(test_idx.begin(), test_idx.end(), test_value.begin());
  assert (test_value == test_answer);

  std::vector<double> test_nums = { 234.2, 214.5, 123.2, 264.2 };
  std::vector<size_t> answer_idx = { 2, 1, 0, 3};
  std::vector<size_t> least_to_greatest_idx = sort_indexes(test_nums, comp_double);
  assert (least_to_greatest_idx == answer_idx);

}
