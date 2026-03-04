#include "planner_common/graph_base.h"
#include <tf/transform_datatypes.h>

class Timer
{
private:
	std::chrono::_V2::system_clock::time_point t_start, t_end;

public:
	Timer()
	{
		t_start = std::chrono::high_resolution_clock::now();
	}
	void reset()
	{
		t_start = std::chrono::high_resolution_clock::now();
	}
	double endTimer(bool print = false)
	{
		t_end = std::chrono::high_resolution_clock::now();
		double dt = std::chrono::duration<double, std::milli>(t_end - t_start).count();
		if (print)
		{
			std::cout << "Time duration: " << dt << "ms" << std::endl;
		}
		return dt;
	}
};

inline void convert(const Eigen::Matrix<double, 5, 1> &st, geometry_msgs::Pose &p)
{
	Eigen::Quaterniond q =
	Eigen::AngleAxisd(st[3], Eigen::Vector3d::UnitZ()) *
	Eigen::AngleAxisd(st[4], Eigen::Vector3d::UnitY());
	q.normalize();

	p.position.x = st[0];
	p.position.y = st[1];
	p.position.z = st[2];

	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();

}

inline void convert(const geometry_msgs::Pose &p, Eigen::Matrix<double, 5, 1> &st)
{
	st[0] = p.position.x;
	st[1] = p.position.y;
	st[2] = p.position.z;
	// st[3] = tf::getYaw(p.orientation);
	tf::Quaternion quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	tfScalar yaw, pitch, roll;
	tf::Matrix3x3 mat(quat);
	mat.getEulerYPR(yaw, pitch, roll);
	st[3] = yaw;
	st[4] = pitch;
}

inline void convert(const std::vector<geometry_msgs::Pose> &pose_path, std::vector<Eigen::Matrix<double, 5, 1>> &vec_path)
{
	vec_path.clear();
	for(auto &p : pose_path)
	{
		Eigen::Matrix<double, 5, 1> vp;
		convert(p, vp);
		vec_path.push_back(vp);
	}
}

inline void convert(const std::vector<Eigen::Matrix<double, 5, 1>> &vec_path, std::vector<geometry_msgs::Pose> &pose_path)
{
	pose_path.clear();
	for(auto &vp : vec_path)
	{
		geometry_msgs::Pose p;
		convert(vp, p);
		pose_path.push_back(p);
	}
}

inline void truncateAngle(double &angle)
{
	if (angle < -M_PI)
		angle += 2 * M_PI;
	else if (angle > M_PI)
		angle -= 2*M_PI;
}

inline double getDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
	Eigen::Matrix<double, 5, 1> vec1, vec2;
	convert(pose1, vec1);
	convert(pose2, vec2);
	return (vec1.head(3) - vec2.head(3)).norm();
}

inline double pathLength(std::vector<geometry_msgs::Pose> &path)
{
	double path_length = 0;
	for(int i=1; i<path.size(); ++i)
	{
		Eigen::Matrix<double, 5, 1> vec1, vec2;
		convert(path[i], vec1);
		convert(path[i-1], vec2);
		path_length += (vec1.head(3) - vec2.head(3)).norm();
	}
	return path_length;
}

inline void linearlyInterpolateYaw(std::vector<geometry_msgs::Pose> &path)  // Keeps the yaw same for first and last pose, and interpolates the rest
{
	if(path.size() <= 2)
		return;

	Eigen::Matrix<double, 5, 1> first_point, last_point;
	convert(path.front(), first_point);
	convert(path.back(), last_point);

	int sign_factor = 1;
	if(last_point[3] > first_point[3])
	{
		if(std::abs(last_point[3] - first_point[3]) >= M_PI)
		{
			sign_factor = -1;
		}
		else
		{
			sign_factor = 1;
		}
	}
	else
	{
		if(std::abs(last_point[3] - first_point[3]) >= M_PI)
		{
			sign_factor = 1;
		}
		else
		{
			sign_factor = -1;
		}
	}

	double delta_theta = last_point[3] - first_point[3];
	truncateAngle(delta_theta);
	delta_theta = std::abs(delta_theta);
	double path_length = pathLength(path);
	double prev_yaw = first_point[3];
	for(int i=1; i<path.size()-1; ++i)
	{
		double del_theta = delta_theta * getDistance(path[i], path[i-1]) / path_length;
		double new_yaw = prev_yaw + sign_factor * del_theta;
		truncateAngle(new_yaw);
		Eigen::Matrix<double, 5, 1> new_point;
		convert(path[i], new_point);
		new_point[3] = new_yaw;
		convert(new_point, path[i]);
		prev_yaw = new_yaw;
	}
}

inline double getDistance(const Eigen::Matrix<double, 5, 1>& a, const Eigen::Matrix<double, 5, 1>& b)
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

inline double pathLength(const std::vector<Eigen::Matrix<double, 5, 1>>& path)
{
  if (path.size() < 2) return 0.0;

  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    len += getDistance(path[i], path[i-1]);
  return len;
}

// Keeps yaw same for first and last state, interpolates the rest (linearly by traveled distance)
inline void linearlyInterpolateYaw(std::vector<Eigen::Matrix<double, 5, 1>>& path)
{
  if (path.size() <= 2)
    return;

  const Eigen::Matrix<double, 5, 1>& first_point = path.front();
  const Eigen::Matrix<double, 5, 1>& last_point  = path.back();

  int sign_factor = 1;
  if (last_point[3] > first_point[3])
  {
    sign_factor = (std::abs(last_point[3] - first_point[3]) >= M_PI) ? -1 : 1;
  }
  else
  {
    sign_factor = (std::abs(last_point[3] - first_point[3]) >= M_PI) ?  1 : -1;
  }

  double delta_theta = last_point[3] - first_point[3];
  truncateAngle(delta_theta);              // should wrap to e.g. [-pi, pi]
  delta_theta = std::abs(delta_theta);

  const double total_len = pathLength(path);
  if (total_len <= 1e-12)                  // avoid divide-by-zero if all points coincide
    return;

  double prev_yaw = first_point[3];

  for (size_t i = 1; i + 1 < path.size(); ++i)
  {
    const double seg_len   = getDistance(path[i], path[i-1]);
    const double del_theta = delta_theta * (seg_len / total_len);

    double new_yaw = prev_yaw + sign_factor * del_theta;
    truncateAngle(new_yaw);

    path[i][3] = new_yaw;                  // update yaw only
    prev_yaw = new_yaw;
  }
}
