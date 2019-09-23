#include "ReflectorSLAM.h"
namespace agv_robot {
	ReflectorSLAM::ReflectorSLAM(const ConfigFile &cfg) {
		is_update_ = false;
		is_initialized_ = false;
		is_operating_ = false;
		transformed_ = false;
		pf_transformed_ = false;
		is_set_frame_ =true;
		pose_now_ = { 0,0,0 };
		ref_radius_ = cfg.value("ReflectorSLAM", "ref_radius");
		//num_scan_set_ = cfg.value("ReflectorSLAM", "num_scan_set");
		install_position_.x = cfg.value("ReflectorSlAM", "lx");
		install_position_.y = cfg.value("ReflectorSlAM", "ly");
		install_position_.theta = cfg.value("ReflectorSlAM", "ltheta");
		//min_rssi_thre_ = cfg.value("ReflectorSLAM", "min_rssi_thre");
		min_rssi_thre_ = 65;
		max_rssi_thre_ = cfg.value("ReflectorSLAM", "max_rssi_thre");
		//ref_match_thre_ = cfg.value("ReflectorSLAM", "ref_match_thre");
		ref_match_thre_ =1.5;

		time_t timeTick = time(NULL);
		strftime(file_name_, 100, "./out_log/Outlog_%Y%m%d%H%M%S.log", localtime(&timeTick));
		ref_mark_file_ = cfg.value("ReflectorSLAM", "file_ref_mark");
		graph_size_ = 600;
		max_dist_ = 40;
		file_pose_.open("pose_vehicle.txt", ios::out);
		file_blog_.open(file_name_, ios::out);
		file_ref_read_.open(ref_mark_file_, ios::in);
		Set("Initialize", &ReflectorSLAM::Initialize, this);
		Set("Trans2Current", &ReflectorSLAM::Trans2Current, this);
		Set("Record", &ReflectorSLAM::Record, this);
		Set("SetFrame", &ReflectorSLAM::SetFrame, this);
		Set("AddRef", &ReflectorSLAM::AddRef, this);
		Set("AddFinish", &ReflectorSLAM::AddFinish, this);
		Set("ReadConfigureFile", &ReflectorSLAM::ReadConfigureFile, this);
		stringstream  ss;
		string s;
		char punctuation;
		double x, y;
		while (file_ref_read_.peek() != EOF) {
			getline(file_ref_read_, s);
			ss.str(s);
			ss >> punctuation >> x >> punctuation >> y;
			ref_in_map_.push_back({ x,y });
		}
		file_ref_read_.close();
		MarkInMapTran2Graph();
		initgraph(graph_size_, graph_size_, SHOWCONSOLE);//创建图形界面
		setlinestyle(PS_DASH | PS_ENDCAP_FLAT, 5);
		loadimage(&img_, _T("./agv.jpg"));// 从文件加载图像
		double counts = 0;
		QueryPerformanceFrequency(&nFreq_);
		QueryPerformanceCounter(&nBeginTime_);//开始计时 		
	}

	stdmsg::String ReflectorSLAM::Record(const stdmsg::String& a) {
		is_operating_ = true;
		stdmsg::String str;
		while (1) {
			if (is_update_) {
				file_pose_ << "粒子滤波  " << pose_pf_new_.x << "  " << pose_pf_new_.y << "  " << pose_pf_new_.theta << endl;
				file_pose_ << "反光板定位" << pose_vehicle_.x << "  " << pose_vehicle_.y << "  " << pose_vehicle_.theta << endl << endl;;
				break;
			}
		}
		str.set_str("Record successfully!");
		is_operating_ = false;
		return str;
	}

	stdmsg::String ReflectorSLAM::ReadConfigureFile(const stdmsg::String& a) {
		is_operating_ = true;
		stdmsg::String str;
		while (1) {
			if (is_update_) {
				string cfg_name = "localization.ini";
				ConfigFile cfg(cfg_name);
				install_position_.x = cfg.value("ReflectorSlAM", "lx");
				install_position_.y = cfg.value("ReflectorSlAM", "ly");
				install_position_.theta = cfg.value("ReflectorSlAM", "ltheta");
				break;
			}
		}
		str.set_str("OK");
		is_operating_ = false;
		return str;
	}

	stdmsg::String ReflectorSLAM::Initialize(const stdmsg::String& a) {
		stdmsg::String str;
		while (1) {
			if (is_update_) {
				is_operating_ = true;
				if (ref_curr_.size() >= 3) {
					vector<Ray> ref_ray_match_curr;
					vector<Mark> ref_match_in_map, ref_new_curr;
					MatchRef(ref_ray_match_curr, ref_match_in_map, ref_new_curr); //匹配反光板
					if (ref_ray_match_curr.size() >= 3) {
						file_blog_ << "匹配超过3个" << endl;
						ComputePoseWithCeres(ref_ray_match_curr, ref_match_in_map);
						//ComputePoseWithAngleAndDistance(ref_match_curr, ref_match_in_map);
						//ComputePoseWithAngle(ref_match_angle, ref_match_in_map);
						is_initialized_ = true;
						str.set_str("初始化成功！");
						is_operating_ = false;
						break;
					}
				}
				is_operating_ = false;
			}
		}
		return str;
	}

	vector<Mark> ReflectorSLAM::MeanForRef(const vector<vector<Mark> > &ref_all) {
		vector<Mark> ref;
		for (int j = 0; j < ref_all[0].size(); j++) {
			double x = 0, y = 0;
			for (int i = 0; i < ref_all.size(); i++) {
				x += ref_all[i][j].x;
				y += ref_all[i][j].y;
			}
			x /= ref_all.size();
			y /= ref_all.size();
			ref.push_back({ x,y });
		}
		return ref;
	}

	void ReflectorSLAM::PoseTran2Graph() {
		int ratio = graph_size_ / max_dist_;
		pose_in_graph_.x = (pose_now_.x +max_dist_/2) * ratio;
		pose_in_graph_.y= (max_dist_ / 2 -pose_now_.y) * ratio;
		pose_in_graph_.theta = pose_now_.theta;
	}
	
	void ReflectorSLAM::DrawPose() {
		setlinecolor(BLACK);
		setfillcolor(RED);
		fillcircle((int)pose_in_graph_.x, (int)pose_in_graph_.y, 5);
		rotateimage(&img_temp_, &img_, pose_in_graph_.theta);
		putimage((int)pose_in_graph_.x, (int)pose_in_graph_.y, &img_temp_);
	}
	void ReflectorSLAM::CancelDrawPose() {
		setlinecolor(BLACK);
		setfillcolor(BLACK);
		fillrectangle((int)pose_in_graph_.x, (int)pose_in_graph_.y, (int)pose_in_graph_.x + img_temp_.getwidth(), (int)pose_in_graph_.y + img_temp_.getheight());
		return;
	}

	void ReflectorSLAM::MarkInMapTran2Graph() {
		ref_in_map_in_graph_.clear();
		double x, y;
		int ratio = graph_size_ / max_dist_;
		for (int i = 0; i < ref_in_map_.size(); i++) {
			x = (ref_in_map_[i].x + max_dist_ / 2) * ratio;
			y = (max_dist_ / 2 - ref_in_map_[i].y) * ratio;
			ref_in_map_in_graph_.push_back({ x,y });
		}
		return;
	}

	double ReflectorSLAM::ThresholdAngle(const double &theta) {
		if (theta >= 3.1415926)    return theta - 2 * 3.1415926;
		else if (theta < -3.1415926) return theta + 2 * 3.1515926;
		else return theta;
	}
	Pose ReflectorSLAM::ComputeRelativePose(const Pose &pose1, const Pose &pose2) {
		double x, y, theta;
		theta = ThresholdAngle(pose1.theta - pose2.theta);
		x = pose1.x - (pose2.x*cos(theta) - pose2.y * sin(theta));
		y = pose1.y - (pose2.x*sin(theta) + pose2.y * cos(theta));
		return { x,y,theta };
	}
	void ReflectorSLAM::DrawMarkInMap() {
		for (int i = 0; i < ref_in_map_in_graph_.size(); i++) {
			setlinecolor(WHITE);
			line(ref_in_map_in_graph_[i].x - 5,
				ref_in_map_in_graph_[i].y + 5,
				ref_in_map_in_graph_[i].x + 5,
				ref_in_map_in_graph_[i].y - 5
			);
		}
		return;
	}
	void ReflectorSLAM::DrawMark() {
		for (int i = 0; i < ref_in_graph_.size(); i++) {
			setlinecolor(BLUE);
			line(ref_in_graph_[i].x - 5,
				ref_in_graph_[i].y - 5,
				ref_in_graph_[i].x + 5,
				ref_in_graph_[i].y + 5
			);
		}
		return;
	}
	void ReflectorSLAM::CancelDrawMark() {
		for (int i = 0; i < ref_in_graph_.size(); i++) {
			setlinecolor(BLACK);
			line(ref_in_graph_[i].x - 5,
				 ref_in_graph_[i].y - 5,
				 ref_in_graph_[i].x + 5,
				 ref_in_graph_[i].y + 5
			);
		}
		return;
	}

	Ray ReflectorSLAM::ComputeCircleCenter(const vector<Ray> &ref_ray) {
		double theta = 0,range;

		for (int i = 0; i < ref_ray.size(); i++) {
			theta += ref_ray[i].theta;
		}
		theta /= ref_ray.size();
		if (ref_ray.size() == 1) range = ref_ray[0].range;
		else {
			for (int i = 0; i < ref_ray.size(); i++) {
				if (theta < ref_ray[i].theta) {
					range = (ref_ray[i].range - ref_ray[i - 1].range) / (ref_ray[i].theta - ref_ray[i - 1].theta)*(theta - ref_ray[i - 1].theta) + ref_ray[i - 1].range;
					break;
				}
			}
		}
		return { range+ref_radius_,theta };
	}
	
	void ReflectorSLAM::Update(vector<Message*> input, vector<Message*> output) {	
		is_update_ = false;
		
		stdmsg::Laser_Scan scan = *(stdmsg::Laser_Scan*)input[0];
		pose_now_stdmsg_ptr_ = (stdmsg::Pose*)output[0];
		Pose pose_ref2pf;
		if (!is_operating_) {
			pose_pf_.x = scan.robot().position().x();
			pose_pf_.y = scan.robot().position().y();
			pose_pf_.theta = scan.robot().orentation().yaw();

			CancelDrawPose();
			CancelDrawMark();

			GetRefMark(scan);
			MarkTran2Graph();
			PoseTran2Graph();

			DrawMark();
			DrawMarkInMap();
			DrawPose();
			
			if (ref_curr_.size() >= 3) {
				if (is_initialized_) {
					Location();
					if (!transformed_) {
						relative_pose_ = ComputeRelativePose(pose_pf_, pose_vehicle_);
						transformed_ = true;
					}
					else {
						cout << "可设置坐标系！" << endl;
						pose_ref2pf = TransformPose(relative_pose_, pose_vehicle_);
						cout << "反光板定位位姿转换到粒子滤波坐标系下为：(" << pose_ref2pf.x << "," << pose_ref2pf.y << "," << pose_ref2pf.theta / 3.1415926 * 180 << "°)" << endl << endl;
						file_blog_ << "反光板定位位姿转换到粒子滤波坐标系下为：(" << pose_ref2pf.x << "," << pose_ref2pf.y << "," << pose_ref2pf.theta / 3.1415926 * 180 << "°)" << endl;
						if (pf_transformed_) {
							pose_pf_new_ = TransformPose(pose_pf_old2new_, pose_pf_);
							cout << "重新设置原点后粒子滤波位姿为：" << "(" << pose_pf_new_.x << "," << pose_pf_new_.y << "," << pose_pf_new_.theta / 3.1415926 * 180 << "°)" << endl;
							file_blog_ << "重新设置原点后粒子滤波位姿为：" << "(" << pose_pf_new_.x << "," << pose_pf_new_.y << "," << pose_pf_new_.theta / 3.1415926 * 180 << "°)" << endl;
						}
						pose_now_stdmsg_ptr_->mutable_position()->set_x(pose_ref2pf.x);
						pose_now_stdmsg_ptr_->mutable_position()->set_x(pose_ref2pf.y);
						pose_now_stdmsg_ptr_->mutable_orentation()->set_yaw(pose_ref2pf.theta);
					}
				}
				else set_frame_permissed_ = true;
			}
			else {
				cout << "当前观测到的反光板少于3个！" << endl;
				set_frame_permissed_ = false;
			}
		}
		cout << "粒子滤波车体位姿为：(" << pose_pf_.x << "," << pose_pf_.y << "," << pose_pf_.theta / 3.1415926 * 180 << "°)" << endl;
		file_blog_ << "粒子滤波位姿为：(" << pose_pf_.x << "," << pose_pf_.y << "," << pose_pf_.theta / 3.1415926 * 180 << "°)" << endl;
		file_blog_ << endl;
		is_update_ = true;
		return;
	}

	void ReflectorSLAM::MatchRef(vector<Ray>& ref_ray_match_curr,
		vector<Mark>& ref_match_in_map,
		vector<Mark>& ref_new_curr) {
		bool match;
		file_blog_ << "所有的ref_curr_为：" << endl;
		for (int i = 0; i < ref_curr_.size(); i++) {
			file_blog_ << "(" << ref_curr_[i].x << "," << ref_curr_[i].y << ")" << endl;
		}
		file_blog_ << "所有的ref_world_为：" << endl;
		for (int i = 0; i < ref_world_.size(); i++) {
			file_blog_ << "(" << ref_world_[i].x << "," << ref_world_[i].y << ")" << endl;
		}
		file_blog_ << "匹配的ref_world_为：" << endl;
		for (int i = 0; i < ref_world_.size(); i++) {
			match = false;
			for (int j = 0; j < ref_in_map_.size(); j++) {
				if (abs(ref_world_[i].x - ref_in_map_[j].x) <= ref_match_thre_
					&& abs(ref_world_[i].y - ref_in_map_[j].y) <= ref_match_thre_) {

					file_blog_ << "(" << ref_world_[i].x << "," << ref_world_[i].y << ")" << endl;
					ref_ray_match_curr.push_back(ref_ray_[i]);
					ref_match_in_map.push_back(ref_in_map_[j]);
					match = true;
				}
			}
			if (!match) ref_new_curr.push_back(ref_curr_[i]);
		}
		file_blog_ << "匹配的ref_ray_match_curr为：" << endl;
		for (int i = 0; i < ref_ray_match_curr.size(); i++) {
			file_blog_ << "("<<ref_ray_match_curr[i].range<< ","<<ref_ray_match_curr[i].theta <<","<< endl;
		}
		file_blog_ << "匹配的ref_match_in_map为：" << endl;
		for (int i = 0; i < ref_ray_match_curr.size(); i++) {
			file_blog_ << "(" << ref_match_in_map[i].x << "," << ref_match_in_map[i].y << ")" << endl;
		}
		return;
	}

	void ReflectorSLAM::MarkTran2Graph() {
		ref_in_graph_.clear();
		double x, y;
		int ratio = graph_size_ / max_dist_;
		for (int i = 0; i < ref_world_.size(); i++) {
			x = (ref_world_[i].x + max_dist_/2) * ratio;
			y = (max_dist_/2 - ref_world_[i].y) * ratio;
			ref_in_graph_.push_back({ x,y });
		}
		return;
	}
	vector<Mark> ReflectorSLAM::transform(const vector<Mark> &mark_input, const Pose &pose) {
		vector<Mark> mark_output;
		Mark mark_temp;
		double x_temp, y_temp;
		for (int i = 0; i < mark_input.size(); i++) {
			x_temp = pose.x + mark_input[i].x*cos(pose.theta) - mark_input[i].y*sin(pose.theta);
			y_temp = pose.y + mark_input[i].x*sin(pose.theta) + mark_input[i].y*cos(pose.theta);
			mark_temp = { x_temp,y_temp };
			mark_output.push_back(mark_temp);
		}
		return mark_output;
	}

	void ReflectorSLAM::GetRefMark(const stdmsg::Laser_Scan& scan) {
		ref_curr_.clear();
		ref_ray_.clear();
		//ref_angle_.clear();
		vector<Ray> ref_ray_temp;
		auto ranges_rssi = scan.ranges_rssi();
		auto ranges = scan.ranges();
		float start_angle = scan.config().angle_min();
		float angle_increment = scan.config().angle_increment();
		float range_temp;
		float x_sum = 0, y_sum = 0, theta_sum = 0;
		float x_mid, y_mid, theta, theta_mid;
		int num_laser = 0;
		double dist_first;
		//cout << "此次扫描" << endl;
		for (int i = 0; i < scan.ranges().size(); i++) {
			//cout << ranges_rssi.Get(i) << " ";
			if (ranges_rssi.Get(i) > min_rssi_thre_ && ranges_rssi.Get(i) < max_rssi_thre_) {
				num_laser++;
				ref_ray_temp.push_back({ ranges.Get(i), start_angle + angle_increment * i });
				/*if (num_laser = 1) dist_first = ranges.Get(i);*/
				/*if (abs(ranges.Get(i) - dist_first) <= 0.4) {*/
					//cout << ranges_rssi.Get(i) << " ";
					/*theta = start_angle + angle_increment * i;
					x_sum += ranges.Get(i)*cos(theta);
					y_sum += ranges.Get(i)*sin(theta);
					theta_sum += theta;*/
					
				//}
				/*else {
					x_mid = x_sum / num_laser;
					y_mid = y_sum / num_laser;
					theta_mid = theta_sum / num_laser;
					Mark ref_temp = { x_mid, y_mid };
					ref_scan_.push_back(ref_temp);
					ref_angle_.push_back(theta_mid);

					x_sum = 0;
					y_sum = 0;
					theta_sum = 0;
					num_laser = 0;

					dist_first = ranges.Get(i);
					cout << ranges_rssi.Get(i) << " ";
					theta = start_angle + angle_increment * i;
					x_sum += ranges.Get(i)*cos(theta);
					y_sum += ranges.Get(i)*sin(theta);
					theta_sum += theta;
					num_laser++;*/
				//}
				
			}
			else {
				if (num_laser >= 1) {
					Ray ref_ray = ComputeCircleCenter(ref_ray_temp);
					double x = ref_ray.range*cos(ref_ray.theta);
					double y = ref_ray.range*sin(ref_ray.theta);
					ref_curr_.push_back({ x, y });
					ref_ray_.push_back(ref_ray);
					//cout << endl;
					/*x_mid = x_sum / num_laser;
					y_mid = y_sum / num_laser;
					theta_mid = theta_sum / num_laser;
					Mark ref_temp = { x_mid, y_mid };
					ref_curr_.push_back(ref_temp);
					ref_angle_.push_back(theta_mid);*/
				}
				/*x_sum = 0;
				y_sum = 0;
				theta_sum = 0;*/
				ref_ray_temp.clear();
				num_laser = 0;
			}
		}
		ref_world_ = transform(ref_curr_, pose_now_);//利用上一帧车体位姿将反光板坐标从车体坐标系转化到世界坐标系
		return;
	}

	Pose ReflectorSLAM::TransformPose(const Pose &reference, const Pose &object) {
		double x, y, theta;
		theta = ThresholdAngle(reference.theta + object.theta);
		x = reference.x + object.x*cos(reference.theta) - object.y*sin(reference.theta);
		y = reference.y + object.x*sin(reference.theta) + object.y*cos(reference.theta);
		return {x,y,theta};
	}

	void ReflectorSLAM::GetTime(string item) {
		QueryPerformanceCounter(&nEndTime_);//停止计时  
		time_ = (double)(nEndTime_.QuadPart - nBeginTime_.QuadPart) / (double)nFreq_.QuadPart;//计算程序执行时间单位为s  
		nBeginTime_ = nEndTime_;
		file_blog_ << item <<"用时："<< time_ * 1000 << " ms" << endl;
	}

	Circle ReflectorSLAM::ComputeCircle(const double& ref_match_angle1,
		const Mark& ref_match_in_map1,
		const double& ref_match_angle2,
		const Mark& ref_match_in_map2) {
		double a, b, R;
		double alpha;
		double x1, y1, x2, y2, x3, y3, dist;
		double m, n;

		double a1, b1, R1, a2, b2, R2;
		double pose_x1, pose_y1, pose_x2, pose_y2;
		x1 = ref_match_in_map1.x;
		y1 = ref_match_in_map1.y;
		x2 = ref_match_in_map2.x;
		y2 = ref_match_in_map2.y;
		alpha = abs(ref_match_angle2 - ref_match_angle1);

		dist = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
		R = dist / (2.0 * abs(sin(alpha)));

		m = (y1 + y2 + (x1 + x2)*(x1 - x2) / (y1 - y2)) / 2;
		n = -(x1 - x2) / (y1 - y2);

		double A, B, C;
		A = 1.0 + n * n;
		B = -2.0 * x1 + 2.0 * m*n - 2.0 * n*y1;
		C = x1 * x1 + m * m + y1 * y1 - 2.0 * m*y1 - R * R;

		a1 = (-B + sqrt(B*B - 4.0 * A*C)) / (2.0 * A);
		b1 = m + n * a1;
		a2 = (-B - sqrt(B*B - 4.0 * A*C)) / (2.0 * A);
		b2 = m + n * a2;
		double abs1 = abs((pose_now_.x - a1)*(pose_now_.x - a1) + (pose_now_.y - b1)*(pose_now_.y - b1) - R * R);
		double abs2 = abs((pose_now_.x - a2)*(pose_now_.x - a2) + (pose_now_.y - b2)*(pose_now_.y - b2) - R * R);
		if (abs1 <= abs2) {
			a = a1;
			b = b1;
		}
		else {
			a = a2;
			b = b2;
		}
		return { a,b,R };
	}

	Mark ReflectorSLAM::ComputeXY(const vector<Mark>& ref_match_curr,
		const vector<Mark>& ref_match_in_map) {
		int num_match = ref_match_curr.size();
		//构造矩阵方程Ax=b
		Eigen::MatrixXd A, b;
		A.resize(num_match*(num_match - 1) / 2, 2);
		b.resize(num_match*(num_match - 1) / 2, 1);
		int k = 0;
		for (int i = 0; i < num_match; i++) {
			for (int j = i + 1; j < num_match; j++) {
				A(k, 0) = -2 * (ref_match_in_map[i].x - ref_match_in_map[j].x);
				A(k, 1) = -2 * (ref_match_in_map[i].y - ref_match_in_map[j].y);
				b(k, 0) = (ref_match_curr[i].x *   ref_match_curr[i].x + ref_match_curr[i].y *   ref_match_curr[i].y
					- ref_match_in_map[i].x * ref_match_in_map[i].x - ref_match_in_map[i].y * ref_match_in_map[i].y)
					- (ref_match_curr[j].x *   ref_match_curr[j].x + ref_match_curr[j].y *   ref_match_curr[j].y
						- ref_match_in_map[j].x * ref_match_in_map[j].x - ref_match_in_map[j].y * ref_match_in_map[j].y);
				k++;
			}
		}

		Eigen::MatrixXd B = A.transpose()*A;
		Eigen::Matrix<double, 2, 1> solution = B.inverse()*A.transpose()*b;
		return { solution(0,0),solution(1,0) };
	}


	void ReflectorSLAM::ComputePoseWithAngleAndDistance(const vector<Mark>& ref_match_curr,
		const vector<Mark>& ref_match_in_map) {
		Mark PoseXY_temp = ComputeXY(ref_match_curr, ref_match_in_map);
		pose_now_.x = PoseXY_temp.x;
		pose_now_.y = PoseXY_temp.y;
		vector<double> ref_match_angle = GetAngleForRef(ref_match_curr);
		ComputeTheta(ref_match_angle, ref_match_in_map);
		file_blog_ << "解矩阵方程三边定位：" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta * 180 / 3.1415926 << "°)" << endl;
	}

	Mark ReflectorSLAM::ComputeXY(const Mark &ref_same, const Circle &circle1, const Circle &circle2) {
		double a1 = circle1.a;
		double b1 = circle1.b;
		double R1 = circle1.R;
		double a2 = circle2.a;
		double b2 = circle2.b;
		double R2 = circle2.R;
		double m = (b1 + b2 + (a1 * a1 - a2 * a2 + R2 * R2 - R1 * R1) / (b1 - b2)) / 2;
		double n = -(a1 - a2) / (b1 - b2);
		double A = 1.0 + n * n;
		double B = -2.0* a1 + 2.0 * m*n - 2.0 * n*b1;
		double C = a1 * a1 + m * m + b1 * b1 - 2.0 * m*b1 - R1 * R1;

		double x1 = (-B + sqrt(B*B - 4.0 * A*C)) / (2.0 * A);
		double y1 = m + n * x1;
		double x2 = (-B - sqrt(B*B - 4.0 * A*C)) / (2.0 * A);
		double y2 = m + n * x2;
		double abs1 = sqrt((x1 - ref_same.x)*(x1 - ref_same.x) + (y1 - ref_same.y)*(y1 - ref_same.y));
		double abs2 = sqrt((x2 - ref_same.x)*(x2 - ref_same.x) + (y2 - ref_same.y)*(y2 - ref_same.y));
		double x, y;
		if (abs1 >= abs2) {
			x = x1;
			y = y1;
		}
		else {
			x = x2;
			y = y2;
		}
		return { x,y };
	}

	vector<double> ReflectorSLAM::GetAngleForRef(const vector<Mark> &ref) {
		vector<double> angle;
		for (int i = 0; i < ref.size(); i++) {
			angle.push_back(atan2(ref[i].y, ref[i].x));
		}
		return angle;
	}

	void ReflectorSLAM::ComputeTheta(const vector<double> &ref_match_angle,
		const vector<Mark> &ref_match_in_map) {
		double theta_sum = 0, theta_plus_sum = 0, theta_minus_sum = 0;
		double alpha1, alpha2;
		int num_plus = 0, num_minus = 0;
		vector<double> theta_all;
		for (int i = 0; i < ref_match_in_map.size(); i++) {
			alpha1 = atan2(ref_match_in_map[i].y - pose_now_.y, ref_match_in_map[i].x - pose_now_.x);
			alpha2 = ref_match_angle[i];
			if (abs(alpha1) <= 3.03) {
				theta_all.push_back(ThresholdAngle(alpha1 - alpha2));
			}
		}
		int num = theta_all.size();
		for (int i = 0; i < theta_all.size(); i++) {
			if (theta_all[i] > 3.03) {
				theta_plus_sum += theta_all[i];
				num_plus++;
				num--;
			}
			else if (theta_all[i] < -3.03) {
				theta_minus_sum += theta_all[i];
				num_minus++;
				num--;
			}
			else {
				theta_sum += theta_all[i];
			}
		}
		if (num_minus >= num && num_minus >= num_plus)  pose_now_.theta = theta_minus_sum / num_minus;
		if (num_plus >= num && num_plus >= num_minus) pose_now_.theta = theta_plus_sum / num_plus;
		if (num >= num_plus && num >= num_minus) pose_now_.theta = theta_sum / num;
		return;
	}

	double ReflectorSLAM::Distance(const Circle &circle1, const Circle &circle2) {
		double a1 = circle1.a;
		double b1 = circle1.b;
		double a2 = circle2.a;
		double b2 = circle2.b;
		return sqrt((a1 - a2)*(a1 - a2) + (b1 - b2)*(b1 - b2));
	}

	void ReflectorSLAM::ComputePoseWithAngle(const vector<double>& ref_match_angle,
		const vector<Mark>& ref_match_in_map) {
		double x_sum = 0, y_sum = 0;
		int count = 0;
		Mark PoseXY_temp;
		Circle circle1, circle2;
		for (int i = 0; i < ref_match_angle.size(); i++) {
			for (int j = i + 1; j < ref_match_angle.size(); j++) {
				if (abs(ref_match_angle[i] - ref_match_angle[j]) >= 0.5) {
					circle1 = ComputeCircle(ref_match_angle[i], ref_match_in_map[i], ref_match_angle[j], ref_match_in_map[j]);
					for (int k = j + 1; k < ref_match_angle.size(); k++) {
						if (abs(ref_match_angle[i] - ref_match_angle[j]) >= 0.5) {
							circle2 = ComputeCircle(ref_match_angle[j], ref_match_in_map[j], ref_match_angle[k], ref_match_in_map[k]);
							if (Distance(circle1, circle2) >= 1) {
								PoseXY_temp = ComputeXY(ref_match_in_map[j], circle1, circle2);
								x_sum += PoseXY_temp.x;
								y_sum += PoseXY_temp.y;
								count++;
							}
						}
					}
				}
			}
		}
		if (count >= 1) {
			pose_now_.x = x_sum / count;
			pose_now_.y = y_sum / count;
			ComputeTheta(ref_match_angle, ref_match_in_map);
			file_blog_ << "三角定位：" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta * 180 / 3.1415926 << "°)" << endl;
		}
	}

	void ReflectorSLAM::ComputePoseWithCeres(const vector<Ray> &ref_ray_match_curr,const vector<Mark> &ref_match_in_map) {
		ceres::Problem location_problem;
		double x = pose_now_.x;
		double y = pose_now_.y;
		double theta = pose_now_.theta;

		for (int i = 0; i < ref_ray_match_curr.size(); i++) {
			auto cost_function = new ceres::AutoDiffCostFunction<LocalizationCost, 3, 1, 1, 1>(new LocalizationCost(ref_ray_match_curr[i],ref_match_in_map[i]));
			location_problem.AddResidualBlock(cost_function, NULL, &x, &y, &theta);
		}
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;
		options.max_num_iterations = 50;
		options.gradient_tolerance = 1e-16;
		options.function_tolerance = 1e-16;
		ceres::Solver::Summary summary;
		Solve(options, &location_problem, &summary);
		pose_now_ = { x,y,theta };
		file_blog_ << "利用ceres三边定位：" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta / 3.14 * 180 << "°)" << endl;
		return;
	}
	
	void ReflectorSLAM::Location() {
		vector<Mark> ref_match_in_map, ref_match_curr,ref_new_curr;
		vector<Ray> ref_ray_match_curr;
		MatchRef(ref_ray_match_curr, ref_match_in_map,ref_new_curr); //匹配反光板
		if (ref_ray_match_curr.size() >= 3) {
			//ComputePoseWithAngleAndDistance(ref_match_curr, ref_match_in_map);
			ComputePoseWithCeres(ref_ray_match_curr,ref_match_in_map);
			Pose pose_laser_vehicle0 = TransformPose(install_position_, pose_now_);
			pose_vehicle_ = ComputeRelativePose(pose_laser_vehicle0, install_position_);
			//ComputePoseWithAngle(ref_match_angle, ref_match_in_map);
			cout << "激光雷达位姿：" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta / 3.14 * 180 << "°)" << endl;
			cout << "车体位姿：" << "(" << pose_vehicle_.x << "," << pose_vehicle_.y << "," << pose_vehicle_.theta / 3.14 * 180 << "°)" << endl;
			if (ref_new_curr.size() > 0) {
				add_ref_permissed_ = true;
				cout << "发现新反光板，可添加进地图！" << endl;
			}
		}
		else {
			cout << "当前反光板匹配数目少于3个，无法进行定位！" << endl << endl;
			add_ref_permissed_ = false;
		}
		return;
	}

	void ReflectorSLAM::CancelDrawMarkInMap() {
		for (int i = 0; i < ref_in_map_in_graph_.size(); i++) {
			setlinecolor(BLACK);
			line(ref_in_map_in_graph_[i].x - 5,
				ref_in_map_in_graph_[i].y + 5,
				ref_in_map_in_graph_[i].x + 5,
				ref_in_map_in_graph_[i].y - 5
			);
		}
		return;
	}

	stdmsg::String ReflectorSLAM::Trans2Current(const stdmsg::String &a) {
		is_operating_ = true;
		stdmsg::String str;
		while (1) {
			if (is_update_) {
				CancelDrawPose();
				CancelDrawMark();
				CancelDrawMarkInMap();

				Pose pose_relative = ComputeRelativePose({ 0,0,0 }, pose_now_);
				pose_now_ = { 0,0,0 };
				pose_vehicle_ = {0,0,0};
				
				ref_in_map_ = transform(ref_in_map_, pose_relative);
				MarkInMapTran2Graph();
				DrawMarkInMap();

				ref_world_ = transform(ref_curr_, pose_now_);
				MarkTran2Graph();
				DrawMark();

				pose_pf_new_ = { 0,0,0 };
				pose_pf_old2new_ = ComputeRelativePose({0,0,0}, pose_pf_);
				pf_transformed_ = true;
				transformed_ = false;
				str.set_str("Transform to Current Frame successfully!");
				break;
			}
		}
		is_operating_ = false;
		return str;
	}

	stdmsg::String ReflectorSLAM::AddFinish(const stdmsg::String& a) {
		stdmsg::String str;
		string s;
		char punctuation;
		double x, y;
		stringstream ss;
		if (is_adding_ref_) {
			ref_in_map_.clear();
			file_ref_read_.open(ref_mark_file_, ios::in);
			while (file_ref_read_.peek() != EOF) {
				getline(file_ref_read_, s);
				ss.str(s);
				ss >> punctuation >> x >> punctuation >> y;
				ref_in_map_.push_back({ x,y });
			}
			file_ref_read_.close();
			MarkInMapTran2Graph();
			str.set_str("Finish to add Reflectors!");
			cout << "添加反光板成功！" << endl;
		}
		is_operating_ = false;
		return str;
	}

	stdmsg::String ReflectorSLAM::SetFrame(const stdmsg::String &a) {
		is_operating_ = true;
		stdmsg::String str;
		if (set_frame_permissed_) {
			while (1) {
				if (is_update_) {
					if (ref_curr_.size() >= 3) {
						ref_in_map_ = ref_curr_;
						MarkInMapTran2Graph();
						file_ref_write_.open(ref_mark_file_, ios::out);
						for (int i = 0; i < ref_in_map_.size(); i++) {
							file_ref_write_ << "(" << ref_in_map_[i].x << "," << ref_in_map_[i].y << ")" << endl;
						}
						file_blog_ << "设置坐标系成功！当前ref_in_map_为：" << endl;
						for (int i = 0; i < ref_in_map_.size(); i++) {
							file_blog_ << "(" << ref_in_map_[i].x << "," << ref_in_map_[i].y << ")" << endl;
						}
						file_ref_write_.close();
						str.set_str("Set frame successfully!");
						cout << "Set frame successfully!" << endl;
						is_set_frame_ = true;
						break;
					}
				}
			}
		}
		is_operating_ = false;
		return str;
	}

	stdmsg::String ReflectorSLAM::AddRef(const stdmsg::String& a) {
		is_operating_ = true;
		stdmsg::String str;
		if (add_ref_permissed_) {
			while (1) {
				if (is_update_) {
					vector<Mark> ref_match_in_map, ref_new_curr;
					vector<Ray> ref_ray_match_curr;
					MatchRef(ref_ray_match_curr, ref_match_in_map, ref_new_curr);
					if (ref_ray_match_curr.size() >= 3) {
						ComputePoseWithCeres(ref_ray_match_curr, ref_match_in_map);
						file_ref_write_.open(ref_mark_file_, ios::app);
						vector<Mark> ref_in_map_new = transform(ref_new_curr, pose_now_);
						for (int i = 0; i < ref_in_map_new.size(); i++) {
							file_ref_write_ << "(" << ref_in_map_new[i].x << "," << ref_in_map_new[i].y << ")" << endl;
						}
						file_ref_write_.close();
						str.set_str("添加成功");
						cout << "添加成功！" << endl;
						break;
					}
					else {
						cout << "匹配的反光板数目不足3个，无法添加反光板！" << endl;
						file_blog_ << "匹配的反光板数目不足3个，无法添加反光板！" << endl;
						is_adding_ref_ = false;
						break;
					}
				}
			}
		}
		return str;
	}

}