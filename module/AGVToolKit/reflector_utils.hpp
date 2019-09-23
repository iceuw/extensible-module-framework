#ifndef REFLECTOR_UTILS_
#define REFLECTOR_UTILS_

#include "stdmsg.pb.h"
#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

struct Pose2D 
{
	float x;
	float y;
	Pose2D(float a, float b) {
		x = a; y = b;
	}
	float SqureDist(Pose2D p) {
		return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y);
	}
	float Dist(Pose2D p) {
		return sqrt(this->SqureDist(p));
	}
};

class ReflectorUtils
{
private:
  float self_dist_thre_ = 0.4 * 0.4;
  float other_dist_thre = 3 * 3;

  float rssi_min_thre_ = 60;
  float rssi_max_thre_ = 180;

  vector<Pose2D> reflectors_;

  vector<vector<Pose2D>> observes_;
  vector<Pose2D> observes_center_;

  void AddOneObserve(Pose2D pose, int index = -1)
  {
    if (index > -1)
    {
      assert(observes_.size() > index);
      observes_[index].push_back(pose);

      //重新计算中心点位置
      float old_x = observes_center_[index].x;
      float old_y = observes_center_[index].y;
      int nu = observes_[index].size();
      float update_x = old_x * (nu - 1) / nu + pose.x / nu;
      float update_y = old_y * (nu - 1) / nu + pose.y / nu;
      observes_center_[index].x = update_x;
      observes_center_[index].y = update_y;
    }
    else
    {
      vector<Pose2D> new_observe;
      new_observe.push_back(pose);
      observes_center_.push_back(pose);
      observes_.push_back(new_observe);
    }
    assert(observes_.size() == observes_center_.size());
  }

public:
  ReflectorUtils()
  {
  }

  vector<Pose2D> GetReflectors()
  {
    return reflectors_;
  }
  void SaveRefToFile()
  {
    ComputeRefByObserves();
    fstream file("ReflectorPos.dat", ios::out | ios::ate);
    for (auto reflector : reflectors_)
    {
      file << reflector.x;
      file << ",";
      file << reflector.y;
      file << endl;
    }
    file.close();
  }

  void ReadRefFromFile()
  {
    ifstream file("ReflectorPos.dat", ios::in);
    float x, y;
    string line;
    stringstream ss;
    while (!file.eof())
    {
      getline(file, line);
      char c;
      ss.clear();
      ss << line;
      if (ss >> x >> c >> y)
      {
        Pose2D ref(x, y);
        reflectors_.push_back(ref);
      }
    }
  }

  //Pose GetBias(stdmsg::Laser_Scan scan)
  //{
  //}

  void TryToAddOneObserve(Pose2D pose)
  {
    int handle_status = 0;	// 用于表明这个点的状态，-1为异常点；
                            // 0为将为该点创建一个新的区域；1为添加到了现存的反光板，
    for (int i = 0; i < observes_center_.size(); ++i)
    {
      Pose2D centroid = observes_center_[i];
      float dist = pose.SqureDist(centroid);
      if (dist < self_dist_thre_)
      {
        cout << "obserced reflector, start add (" << pose.x
          << ", " << pose.y << ") to observes" << endl;
        AddOneObserve(pose, i);
        handle_status = 1;
        if (observes_[i].size() > 100)
        {
          SaveRefToFile();
        }
        break;
      }
      else if (dist < other_dist_thre && observes_[i].size() > 20)
      {
        handle_status = -1;
        break;
      }
      else if (dist < other_dist_thre && observes_[i].size() < 20)
      {
        if (observes_[i].size() > 0)
        {
          observes_[i].pop_back();
        }
        if (observes_[i].size() == 0)
        {
          observes_center_.erase(observes_center_.begin() + i);
          observes_.erase(observes_.begin() + i);
        }
      }
    }

    if (handle_status == 0)
    {
      AddOneObserve(pose);
    }
  }

  void GetOneScan(const stdmsg::Laser_Scan* scan)
  {
    for (int i = 0; i < scan->ranges_size(); ++i)
    {
      float rssi = scan->ranges_rssi(i);
      float range, theta, x, y;
      if (rssi > rssi_min_thre_ && rssi < rssi_max_thre_)
      {
        range = scan->ranges(i);
        theta = scan->pose().orentation().yaw()
          + scan->config().angle_min() + scan->config().angle_increment() * i;
        x = scan->pose().position().x() + range * cos(theta);
        y = scan->pose().position().y() + range * sin(theta);
        Pose2D p(x, y);
        TryToAddOneObserve(p);
      }
      else
        continue;
    }
  }

  void GetScans(stdmsg::LaserList scans)
  {
    for (int i = 0; i < scans.scans_size(); ++i)
    {
      GetOneScan(&(scans.scans(i)));
    }
  }

  void ComputeRefByObserves()
  {
    for (int i = 0; i < observes_.size(); ++i)
    {
      bool valid = true;
      if (observes_[i].size() > 20)
      {
        for (auto reflector : reflectors_)
        {
          if (reflector.SqureDist(observes_center_[i]) < self_dist_thre_)
          {
            valid = false;
            break;
          }
        }
        if (valid)
          reflectors_.push_back(observes_center_[i]);
      }
    }
  }


};

#endif