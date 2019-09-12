#ifndef REFLETORSLAM
#define REFLETORSLAM

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <windows.h>
#include "stdmsg.pb.h"
#include "cfg_utils.hpp"
#include "function_block.hpp"
#include <graphics.h>
#include <conio.h>
#include <ceres/ceres.h>

using namespace std;


namespace agv_robot{
    struct Pose {
        double x,y,theta;
    };

    struct Mark {
        double x, y;
    };

	struct Circle {
		double a, b, R;
	};

	struct LocalizationCost {
		LocalizationCost(const double ref_match_angle, const Mark ref_match_curr, const Mark ref_match_in_map)
			: ref_match_angle(ref_match_angle), ref_match_curr(ref_match_curr), ref_match_in_map(ref_match_in_map) {}

		template <typename T>
		bool operator()(const T* const x, const T* const y, const T* const theta, T* residual) const {
			T predict_theta = theta[0] + T(ref_match_angle);
			T delta_y = T(ref_match_in_map.y) - y[0];
			T delta_x = T(ref_match_in_map.x) - x[0];
			//定位角度与测量角度的误差

			residual[0] = delta_y-delta_x*tan(predict_theta);
			residual[1] = ref_match_in_map.x - (x[0] + ref_match_curr.x*cos(theta[0]) - ref_match_curr.y*sin(theta[0]));
			residual[2] = ref_match_in_map.y - (y[0] + ref_match_curr.x*sin(theta[0]) + ref_match_curr.y*cos(theta[0]));
			return true;
		}

		Mark ref_match_in_map, ref_match_curr;
		double ref_match_angle;
	};

    class __declspec(dllexport) ReflectorSLAM : public FunctionBlock{
        private:            
			Pose	install_position_,              //激光雷达在车体上的安装位置
					pose_now_,                      //AGV当前在固定坐标系下的位姿
					pose_in_graph_,                //AGV在图像中位姿
					relative_pose_,					//反光板定位坐标系相对于粒子滤波定位与坐标系的位姿
					pose_vehicle_;					//在初始车体坐标系下的位姿

			bool   set_frame_permissed_,            //是否可以设置坐标系
				   add_ref_permissed_,              //是否可以添加反光板
				   is_update_,                      //是否完成扫描更新
				   is_set_frame_,                   //是否完成设置坐标系
				   is_adding_ref_,                  //是否正在添加反光板
				   is_locationing_, 			    //是否正在定位
				   is_initialized_,			        //是否初始化
				   is_operating_,                   //是否正在运算
				   transformed_;					//是否已将反光板定位坐标系转化到粒子滤波定位坐标系

            int    num_ref_set_,                   //在地图中预期放置的反光板数量
                   num_scan_set_ ;                 //设置扫描次数

            double min_rssi_thre_,max_rssi_thre_,  //反光板反射率阈值
                   ref_match_thre_;                //反光板匹配阈值

			fstream file_ref_write_,               //地图中反光板坐标文件
				    file_ref_read_,
					file_blog_;

			vector<Mark> ref_in_map_,              //地图中的反光板位置
						ref_scan_,                //扫描到反光板在激光雷达坐标系下的位置
						ref_curr_,                //扫描到的反光板在车体坐标系下的位置
						ref_world_,               //扫描到的反光板在世界坐标系下的位置
						ref_in_graph_,			  //观测到的反光板在图像中的位置
						ref_in_map_in_graph_;    //地图中的反光板在图像中的位置

			vector<double> ref_angle_;			   //扫描到的反光板角度


			int graph_size_,                       //图像尺寸
				max_dist_;                          //图像所描绘的最大距离

			char file_name_[100];

            stdmsg::Pose* pose_now_stdmsg_ptr_;    //AGV当前位姿（用于输出）
			IMAGE img_,img_temp_;
			
			double time_;
			LARGE_INTEGER nFreq_;//计时频率
			LARGE_INTEGER nBeginTime_;
			LARGE_INTEGER nEndTime_;
 
        public:
            ReflectorSLAM(ConfigFile &cfg);//构造函数
			void MatchRef( vector<double>& ref_match_angle,
				             vector<Mark>& ref_match_curr,
				           vector<Mark>& ref_match_in_map); //将当前观测到的反光板与地图中的反光板匹配
            void GetRefMark(const stdmsg::Laser_Scan& scan);  //计算当前坐标系下各反光板位置
            vector<Mark> MeanForRef(const vector<vector<Mark> > &ref_all); //计算反光板位置均值
            void ComputePoseWithAngle(const vector<double>& ref_match_angle,
				             const vector<Mark>& ref_match_in_map); //根据匹配反光板角度计算AGV位姿
			void ComputePoseWithAngleAndDistance(const vector<Mark>& ref_match_curr,
				const vector<Mark>& ref_match_in_map);//根据匹配反光板角度和距离计算AGV位姿
            vector<Mark> transform(const vector<Mark> &mark1,const Pose &pose);  //坐标转换
			Circle ComputeCircle(const double& ref_match_angle1,
				                 const Mark& ref_match_in_map1,
				                 const double& ref_match_angle2,
				                 const Mark& ref_match_in_map2);//计算外接圆
			Pose ComputeRelativePose(const Pose &pose1, const Pose &pose2);//计算坐标系2相对于坐标系1的位姿
			Mark ComputeXY(const Mark &ref_same, const Circle &circle1, const Circle &circle2);
			Mark ComputeXY(const vector<Mark>& ref_match_curr, const vector<Mark>& ref_match_in_map);
			void ComputeTheta(const vector<double> &ref_match_angle,const vector<Mark> &ref_match_in_map);
			vector<double> GetAngleForRef(const vector<Mark> &ref);
			double Distance(const Circle &circle1, const Circle &circle2);
			void ComputePoseWithCeres(const vector<double> &ref_match_angle, const vector<Mark> &ref_match_curr, const vector<Mark> &ref_match_in_map);
			void MarkTran2Graph();//将反光板坐标转换到图像坐标系
			void PoseTran2Graph();//将AGV位姿转换到图像坐标系
			void DrawPose();//在图像中画出AGV位姿
			void CancelDrawPose();//在图像中将箭头去掉
			void MarkInMapTran2Graph();//将地图中的反光板坐标转换到图像坐标系
			void DrawMarkInMap();//在图像中画出地图中的反光板
			void DrawMark();//在图像中画出观测到的反光板
			void CancelDrawMark();//在图像中将观测到的反光板取消
			stdmsg::String Trans2Current(const stdmsg::String &a);//将地图转化到当前车体坐标系下
			void CancelDrawMarkInMap();//在图像中将地图中的反光板去掉
            void Location();   //定位
			Pose TransformPose2Vehicle(const Pose &reference, const Pose &object);
			void GetTime(string item);//获得运行用时
			stdmsg::String Initialize(const stdmsg::String& a);
            void Update(vector<Message*> input, vector<Message*> output);

    };
    EXPORT_INSTANCE(ReflectorSLAM);
}
#endif