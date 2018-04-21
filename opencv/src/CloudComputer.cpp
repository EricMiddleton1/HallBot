#include "CloudComputer.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <numeric>

CloudComputer::CloudComputer(std::vector<IConfigurable::Param> &&params)
    : IConfigurable{{"regression_type"}, std::move(params)}, regression_type{std::stof(getParam("regression_type"))}, how_recent{std::stof(getParam("how_recent"))}, theta{std::stof(getParam("theta"))}, auto_adjust_angle{std::stof(getParam("auto_adjust_angle"))}
{
  theta = (theta * PI) / 180.0;
  w = 500;
  // Create black empty images
  hallway_image = cv::Mat::zeros(w, w, CV_8UC3);
  bw_hallway_image = cv::Mat::zeros(w, w, CV_8UC1);
  wall_alert = false;
  enough_pts_already = false;
  adjusted_3d = false;
}

void CloudComputer::addCircle(cv::Mat img, cv::Point center, int color)
{
  int thickness = -1;
  int lineType = 8;

  circle(img,
         center,
         0.1,
         cv::Scalar(color, 255, 255),
         thickness,
         lineType);
}

void CloudComputer::drawLine(cv::Mat img, cv::Vec4f line, int thickness, cv::Scalar color)
{
  // calculate start point
  cv::Point startPoint;
  startPoint.x = line[2] - w * line[0]; // x0
  startPoint.y = line[3] - w * line[1]; // y0
  // calculate end point
  cv::Point endPoint;
  endPoint.x = line[2] + w * line[0]; //x[1]
  endPoint.y = line[3] + w * line[1]; //y[1]
  // draw overlay of bottom lines on image
  cv::clipLine(cv::Size(w, w), startPoint, endPoint);
  cv::line(img, startPoint, endPoint, color, thickness, 8, 0);
}

cv::Vec4f CloudComputer::convertLine2D(cv::Vec6f a3dline)
{
  cv::Vec4f a2dline = cv::Vec4f(a3dline[0], a3dline[2], a3dline[3], a3dline[5]);
  return a2dline;
}

cv::Point CloudComputer::convertPoint2D(cv::Point3f a3dpoint)
{
  // convention is to keep X and Z dimensions
  cv::Point a2dpoint = cv::Point(a3dpoint.x, a3dpoint.z);
  return a2dpoint;
}

void CloudComputer::displayPoints()
{
  float green_theta = getGreenTheta();
  // rot matrix based on green theta
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << cos(green_theta), 0, sin(green_theta),
                        0, 1, 0,
                        -sin(green_theta), 0, cos(green_theta));
  for (int i = 0; i < raw_mat_vector.size(); i++)
  {
    cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_mat_vector.at(i).at<float>(0), raw_mat_vector.at(i).at<float>(1), raw_mat_vector.at(i).at<float>(2));
    pt_i = rot_matrix * pt_i;
    cv::Point p = cv::Point(static_cast<int>(w / 2 + (pt_i.at<float>(0) * 50)),
                            static_cast<int>((3 * w / 4) - (pt_i.at<float>(2) * 50)));
    addCircle(hallway_image, p, 255);
  }
}

void CloudComputer::updatePointVectors(cv::Mat pos)
{
  //All point vecotrs are stored RAW
  raw_mat_vector.push_back(pos.clone());
  cv::Mat clone_pos = pos.clone();
  //2d point vector
  cv::Point p = cv::Point(static_cast<int>(clone_pos.at<float>(0)), static_cast<int>(clone_pos.at<float>(2)));
  pts_vector.push_back(p);
  //3d point vector
  cv::Point3f p3 = cv::Point3f(static_cast<int>(clone_pos.at<float>(0)), static_cast<int>(clone_pos.at<float>(1)), static_cast<int>(clone_pos.at<float>(2)));
  pts_vector_3d.push_back(p3);
}

void CloudComputer::clearPointVectors()
{
  raw_mat_vector.clear();
  pts_vector.clear();
  pts_vector_3d.clear();
}

void CloudComputer::makeGreenLine()
{
  //HACK need to add toggle for how many pts to consider...
  //vector<cv::Point> longterm_pts_vector(pts_vector.begin(), pts_vector.end());
  // RAW GREEN LINE
  //cv::fitLine(longterm_pts_vector, long_term_line, regression_type, 0, 0.01, 0.01);
  cv::fitLine(pts_vector, long_term_line, regression_type, 0, 0.01, 0.01);
  //then convert green line to 2D diplayable coordinate system
  // cv::Vec4f displayable_greenline = cv::Vec4f(long_term_line[0], long_term_line[1], (w / 2 + (long_term_line[2] * 20)), (w / 2 + (long_term_line[3] * 20)));
  // drawLine(hallway_image, displayable_greenline, 1, cv::Scalar(0, 255, 0));
}

void CloudComputer::steerPoints()
{
  float green_theta = getGreenTheta();
  // rot matrix based on green theta
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << cos(green_theta), 0, sin(green_theta),
                        0, 1, 0,
                        -sin(green_theta), 0, cos(green_theta));
  vector<cv::Mat> raw_clone(raw_mat_vector);
  clearPointVectors();
  // std::cout << "[BEGIN STEER]" << std::endl;
  for (int i = 0; i < raw_clone.size(); i++)
  {
    cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_clone.at(i).at<float>(0), raw_clone.at(i).at<float>(1), raw_clone.at(i).at<float>(2));
    pt_i = rot_matrix * pt_i;
    updatePointVectors(pt_i);
  }
  // std::cout << "[FINISHED STEER]" << std::endl;
}

void CloudComputer::display2D(ORB_SLAM2::Map *total_map)
{
  // diplay window
  char hallway_window[] = "Hallway Projection";

  // set background to white if short term and long term angle is above a certain threshhold
  if (wall_alert)
  {
    hallway_image = cv::Mat::ones(w, w, CV_8UC3);
    hallway_image = cv::Scalar(255, 255, 255);
  }
  else
  {
    hallway_image = cv::Mat::zeros(w, w, CV_8UC3);
  }

  //TODO
  clearPointVectors();

  // get map points and make iterator for reference points.
  const vector<ORB_SLAM2::MapPoint *> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint *> &ref_pts = total_map->GetReferenceMapPoints();
  set<ORB_SLAM2::MapPoint *> set_ref_pts(ref_pts.begin(), ref_pts.end());
  // Map points loop
  for (size_t i = 0, iend = map_pts.size(); i < iend; i++)
  {
    if (map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
      continue;
    cv::Mat pos;
    if (auto_adjust_angle)
    {
      pos = autoRotate(map_pts[i]->GetWorldPos());
    }
    else
    {
      pos = rotateWithTheta(map_pts[i]->GetWorldPos());
    }
    updatePointVectors(pos);
  }
  // Refernece points loop
  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos;
    if (auto_adjust_angle)
    {
      pos = autoRotate((*sit)->GetWorldPos());
    }
    else
    {
      pos = rotateWithTheta((*sit)->GetWorldPos());
    }
    updatePointVectors(pos);
  }
  //regression line
  makeGreenLine();
  // display all 2D pts
  displayPoints();
  // new style green reference line
  cv::Vec4f greenline = cv::Vec4f(0, 1, w / 2, w / 2);
  drawLine(hallway_image, greenline, 1, cv::Scalar(0, 255, 0));
  // print green theta
  // std::cout << getGreenTheta() << std::endl;
  // std::cout << raw_mat_vector.sipts_ << " " << pts_vector.size() << " " << vector_3d.size() << std::endl;
  // std::cout << distToFacingWall() << std::endl;
  // Display
  imshow(hallway_window, hallway_image);
}

void CloudComputer::calcHistogram()
{
  // number of bins k
  int k = w / 10;
  // int k = 10;
  // init hist
  vector<int> buckets(k);
  // histogram of X axis
  // find max and min values
  int min = pts_vector.at(0).x;
  int max = pts_vector.at(0).x;
  for (int i = 0; i < pts_vector.size(); i++)
  {
    if (pts_vector.at(i).x < min)
    {
      min = pts_vector.at(i).x;
    }
    if (pts_vector.at(i).x > max)
    {
      max = pts_vector.at(i).x;
    }
  }
  int range = max - min;
  int b_width = range / k;
  // start and end of current bucket being populated
  int start, end;
  for (int j = 0; j < buckets.size(); j++)
  {
    start = min + (b_width * j);
    end = start + b_width;
    buckets[j] = std::count_if(pts_vector.begin(), pts_vector.end(), [start, end](const auto &cur_pt) {
      if (cur_pt.x >= start && cur_pt.x < end)
      {
        return true;
      }
      else
      {
        return false;
      }
    });
  }
  // for (int m = 0; m < buckets.size(); m++)
  // {
  //   std::cout << "[" << m << "]:" << buckets.at(m) << " ";
  // }
  // std::cout << endl;

  vector<int> diffs(k);
  std::adjacent_difference(buckets.begin(), buckets.end(), diffs.begin());
  int max_diff = std::max_element(diffs.begin(), diffs.end()) - diffs.begin();
  int left_hall = min + (b_width * (max_diff));
  int min_diff = std::min_element(diffs.begin(), diffs.end()) - diffs.begin();
  int right_hall = min + (b_width * (min_diff));
  // for (int i = 0; i < diffs.size(); i++)
  // {
  //   std::cout << diffs[i] << std::endl;
  // }
  // std::cout << "Left: " << left_hall << " Right: " << right_hall << " max diff: " << max_diff << " min diff: " << min_diff << std::endl;

  // print lines for hall boundaries
  cv::Point startPoint;
  startPoint.x = left_hall; // x0
  startPoint.y = 1;         // y0
  cv::Point endPoint;
  endPoint.x = left_hall; // x0
  endPoint.y = w - 5;     // y0
  // cv::clipLine(cv::Size(w, w), startPoint, endPoint);
  cv::line(hallway_image, startPoint, endPoint, cv::Scalar(0, 255, 255), 1, 8, 0);
  startPoint.x = right_hall; // x0
  startPoint.y = 1;          // y0
  endPoint.x = right_hall;   // x0
  endPoint.y = w - 5;        // y0
  // cv::clipLine(cv::Size(w, w), startPoint, endPoint);
  cv::line(hallway_image, startPoint, endPoint, cv::Scalar(0, 255, 255), 1, 8, 0);
}

// ONLY RUN AFTER how_recent^2 pts have been found

std::vector<int> CloudComputer::getProjection(const cv::Mat &image)
{

  std::vector<int> projection(image.rows);

  for (unsigned int i = 0; i < projection.size(); ++i)
  {
    projection[i] = cv::countNonZero(image(cv::Rect(0, i, image.cols, 1)));
  }

  return projection;
}

float CloudComputer::distToFacingWall()
{
  // reset image
  bw_hallway_image = cv::Mat::zeros(w, w, CV_8UC1);
  float green_theta = getGreenTheta();
  // rot matrix based on green theta
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << cos(green_theta), 0, sin(green_theta),
                        0, 1, 0,
                        -sin(green_theta), 0, cos(green_theta));
  for (int i = 0; i < raw_mat_vector.size(); i++)
  {
    cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_mat_vector.at(i).at<float>(0), raw_mat_vector.at(i).at<float>(1), raw_mat_vector.at(i).at<float>(2));
    pt_i = rot_matrix * pt_i;
    cv::Point p = cv::Point(static_cast<int>(w / 2 + (pt_i.at<float>(0) * 50)),
                            static_cast<int>((3 * w / 4) - (pt_i.at<float>(2) * 50)));
    bw_hallway_image.at<cv::Vec3b>(p) = cv::Vec3b(255, 255, 255);
  }
  // get average
  std::vector<int> proj = getProjection(bw_hallway_image);
  int sum = std::accumulate(proj.begin(), proj.end(), 0);
  int total = std::count_if(proj.begin(), proj.end(), [](int value) {
    if (value)
    {
      return true;
    }
    else
    {
      return false;
    }
  });
  int average = sum / total;

  // loop from the back ;)
  bool found_wall = false;
  int k = 0;
  while (!found_wall && k < proj.size())
  {
    if (proj[k] > average)
    {
      // convert to raw coord
      cv::Vec4f line = cv::Vec4f(1, 0, 5, k);
      drawLine(hallway_image, line, 1, cv::Vec3b(255, 0, 0));
      float raw_y = (k - (3.f * w / 4.f)) / 50.f;
      // float raw_y = ((3.f * w / 4.f) - (k / 50.f));
      // rotate camera
      float green_theta = getGreenTheta();
      // rot matrix based on green theta
      cv::Mat rot_matrix = (cv::Mat_<float>(2, 2) << cos(green_theta), -sin(green_theta),
                            sin(green_theta), cos(green_theta));
      cv::Mat cam = (cv::Mat_<float>(2, 1) << cam_pos[0], cam_pos[1]);
      cv::Mat rot_cam = rot_matrix * cam;
      float cam_y = cam.at<float>(1);
      //TODO check for negative output??
      // std::cout << "wall: " << raw_y << " cam: " << cam_pos << endl;
      return -raw_y - cam_y;
    }
    k++;
  }
  return -1;
}

cv::Vec4f CloudComputer::getGreenLine()
{
  //REVIEW if this works
  return long_term_line;
}

float CloudComputer::getGreenTheta()
{
  // angle between green line and 2D y axis
  // float green_theta = atan2(-long_term_line[0], long_term_line[1]);
  // if (green_theta > PI / 2)
  // {
  //   green_theta -= PI;
  // }
  // else if (green_theta < PI / 2)
  // {
  //   green_theta += PI;
  // }

  // return green_theta;
  float green_theta = (atan2(long_term_line[1], long_term_line[0]) * 180 / PI);
  if (green_theta < 0)
  {
    green_theta += 90;
  }
  else if (green_theta > 0)
  {
    green_theta = 90 - green_theta;
    if (green_theta != 0)
    {
      green_theta *= -1;
    }
  }
  else
  {
    green_theta = 0;
  }
  return green_theta * PI / 180;
}

void CloudComputer::setCameraPos(cv::Vec2f pos)
{
  cam_pos = pos;
}

cv::Mat CloudComputer::rotateWithTheta(cv::Mat pos)
{
  cv::Mat pos_clone = pos.clone();
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                        0, cos(theta), -sin(theta),
                        0, sin(theta), cos(theta));
  return rot_matrix * pos_clone;
}

cv::Mat CloudComputer::autoRotate(cv::Mat pos)
{
  // if not enough pts yet, do nothing
  if (pts_vector_3d.size() < how_recent)
  {
    return pos;
  }
  // if enough pts found, check if line has been calculated
  // if line not calculated, do that and update old pts...
  if (!adjusted_3d)
  {
    // regresion line on 3d pts found before enough_pts_already flag thrown
    cv::fitLine(pts_vector_3d, compass_line, regression_type, 0, 0.01, 0.01);
    // create vector on X Z plane
    cv::Mat xz_vector = (cv::Mat_<float>(1, 3) << compass_line[0], 0, compass_line[2]);
    // theta between lines
    auto_theta = acos(((xz_vector.at<float>(0) * compass_line[0]) + (xz_vector.at<float>(1) * compass_line[1]) + (xz_vector.at<float>(2) * compass_line[2])) / (sqrt(pow(compass_line[0], 2) + pow(compass_line[1], 2) + pow(compass_line[2], 2)) * sqrt(pow(xz_vector.at<float>(0), 2) + pow(xz_vector.at<float>(1), 2) + pow(xz_vector.at<float>(2), 2))));
    // rot matrix based on auto_theta
    cv::Mat rot_matrix_init = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                               0, cos(auto_theta), -sin(auto_theta),
                               0, sin(auto_theta), cos(auto_theta));
    std::cout << "[3d THETA]: " << (auto_theta * 180 / PI) << " degrees" << std::endl;
    // clone raw pts and clear all vectors
    vector<cv::Mat> raw_clone(raw_mat_vector);
    clearPointVectors();
    std::cout << "[BEGIN UPDATE OF " << raw_clone.size() << " OLD PTS]" << std::endl;
    for (int i = 0; i < raw_clone.size(); i++)
    {
      cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_clone.at(i).at<float>(0), raw_clone.at(i).at<float>(1), raw_clone.at(i).at<float>(2));
      pt_i = rot_matrix_init * pt_i;
      updatePointVectors(pt_i);
    }
    std::cout << "[FINISHED UPDATE OF OLD PTS]" << std::endl;
    adjusted_3d = true;
  }
  //return current point after rotation made
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                        0, cos(auto_theta), -sin(auto_theta),
                        0, sin(auto_theta), cos(auto_theta));
  return rot_matrix * pos;
}

void CloudComputer::getPointRanges(ORB_SLAM2::Map *total_map)
{

  const vector<ORB_SLAM2::MapPoint *> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint *> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint *> set_ref_pts(ref_pts.begin(), ref_pts.end());

  pt max_pt_vals = {0, 0, 0};
  pt min_pt_vals = {0, 0, 0};

  for (size_t i = 0, iend = map_pts.size(); i < iend; i++)
  {
    if (map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
      continue;
    cv::Mat pos = map_pts[i]->GetWorldPos();
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    if (new_pt.x > max_pt_vals.x)
      max_pt_vals.x = new_pt.x;
    if (new_pt.y > max_pt_vals.y)
      max_pt_vals.y = new_pt.y;
    if (new_pt.z > max_pt_vals.z)
      max_pt_vals.z = new_pt.z;
    if (new_pt.x < min_pt_vals.x)
      min_pt_vals.x = new_pt.x;
    if (new_pt.y < min_pt_vals.y)
      min_pt_vals.y = new_pt.y;
    if (new_pt.z < min_pt_vals.z)
      min_pt_vals.z = new_pt.z;
  }

  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos = (*sit)->GetWorldPos();
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    if (new_pt.x > max_pt_vals.x)
      max_pt_vals.x = new_pt.x;
    if (new_pt.y > max_pt_vals.y)
      max_pt_vals.y = new_pt.y;
    if (new_pt.z > max_pt_vals.z)
      max_pt_vals.z = new_pt.z;
    if (new_pt.x < min_pt_vals.x)
      min_pt_vals.x = new_pt.x;
    if (new_pt.y < min_pt_vals.y)
      min_pt_vals.y = new_pt.y;
    if (new_pt.z < min_pt_vals.z)
      min_pt_vals.z = new_pt.z;
  }

  std::cout << "min pt" << min_pt_vals.x << ", " << min_pt_vals.y << ", " << min_pt_vals.z << "\n"
            << std::endl;
  std::cout << "max pt" << max_pt_vals.x << ", " << max_pt_vals.y << ", " << max_pt_vals.z << "\n"
            << std::endl;
}
