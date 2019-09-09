#include "ReflectorSLAM.h"
namespace agv_robot {
	ReflectorSLAM::ReflectorSLAM(const ConfigFile &cfg) {
		is_update_ = false;
		is_initialized_ = false;
		is_operating_ = false;
		transformed_ = false;
		pose_now_ = { 0,0,0 };
		//num_scan_set_ = cfg.value("ReflectorSLAM", "num_scan_set");
		install_position_.x = cfg.value("ReflectorSlAM", "lx");
		install_position_.y = cfg.value("ReflectorSlAM", "ly");
		install_position_.theta = cfg.value("ReflectorSlAM", "ltheta");
		//min_rssi_thre_ = cfg.value("ReflectorSLAM", "min_rssi_thre");
		min_rssi_thre_ = 65;
		max_rssi_thre_ = cfg.value("ReflectorSLAM", "max_rssi_thre");
		//ref_match_thre_ = cfg.value("ReflectorSLAM", "ref_match_thre");
		ref_match_thre_ =2;
		time_t timeTick = time(NULL);
		strftime(file_name_, 100, "../Outlog_%Y%m%d%H%M%S.log", localtime(&timeTick));
		string ref_mark_file = cfg.value("ReflectorSLAM", "file_ref_mark");
		graph_size_ = 600;
		max_dist_ = 40;
		file_blog_.open(file_name_, ios::out);
		file_ref_read_.open(ref_mark_file, ios::in);
		Set("Initialize", &ReflectorSLAM::Initialize, this);
		Set("Trans2Current", &ReflectorSLAM::Trans2Current, this);

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
		MarkInMapTran2Graph();
		initgraph(graph_size_, graph_size_, SHOWCONSOLE);//����ͼ�ν���
		setlinestyle(PS_DASH | PS_ENDCAP_FLAT, 5);
		loadimage(&img_, _T("../agv.jpg"));// ���ļ�����ͼ��
		double time = 0;
		double counts = 0;
		QueryPerformanceFrequency(&nFreq_);
		QueryPerformanceCounter(&nBeginTime_);//��ʼ��ʱ 		
	}

	stdmsg::String ReflectorSLAM::Initialize(const stdmsg::String& a) {
		stdmsg::String str;
		while (1) {
			if (is_update_) {
				is_operating_ = true;
				if (ref_curr_.size() >= 3) {
					vector<double> ref_match_angle;
					vector<Mark>  ref_match_in_map, ref_match_curr;
					MatchRef(ref_match_angle, ref_match_curr, ref_match_in_map); //ƥ�䷴���
					if (ref_match_angle.size() >= 3) {
						file_blog_ << "ƥ�䳬��3��" << endl;
						//ComputePoseWithCeres(ref_match_angle, ref_match_in_map);
						ComputePoseWithAngleAndDistance(ref_match_curr, ref_match_in_map);
						//ComputePoseWithAngle(ref_match_angle, ref_match_in_map);
						is_initialized_ = true;
						str.set_str("��ʼ���ɹ���");
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
		double x, y;
		int ratio = graph_size_ / max_dist_;
		for (int i = 0; i < ref_in_map_.size(); i++) {
			x = (ref_in_map_[i].x + max_dist_ / 2) * ratio;
			y = (max_dist_ / 2 - ref_in_map_[i].y) * ratio;
			ref_in_map_in_graph_.push_back({ x,y });
		}
		return;
	}
	Pose ReflectorSLAM::ComputeRelativePose(const Pose &pose1, const Pose &pose2) {
		double x, y, theta;
		theta = pose2.theta - pose1.theta;
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
	void ReflectorSLAM::Update(vector<Message*> input, vector<Message*> output) {	
		is_update_ = false;
		stdmsg::Laser_Scan scan = *(stdmsg::Laser_Scan*)input[0];
		pose_now_stdmsg_ptr_ = (stdmsg::Pose*)output[0];
		Pose pose_ref2pf;
		Pose pose_pf;
		pose_pf.x = scan.pose().position().x();
		pose_pf.y = scan.pose().position().y();
		pose_pf.theta = scan.pose().orentation().yaw();
		if (!is_operating_) {
			GetRefMark(scan);
			DrawMarkInMap();
			PoseTran2Graph();
			DrawPose();
			
			if (ref_curr_.size() >= 3) {
				if (is_initialized_) {
					Location();
					if (!transformed_) {
						relative_pose_ = ComputeRelativePose(pose_pf, pose_vehicle_);
						transformed_ = true;
					}
				}
				else {
					pose_ref2pf = TransformPose2Vehicle(relative_pose_, pose_vehicle_);
					pose_now_stdmsg_ptr_->mutable_position()->set_x(pose_ref2pf.x);
					pose_now_stdmsg_ptr_->mutable_position()->set_x(pose_ref2pf.y);
					pose_now_stdmsg_ptr_->mutable_orentation()->set_yaw(pose_ref2pf.theta);
				}
			}
			
			CancelDrawPose();
			CancelDrawMark();
		}
		is_update_ = true;
		return;
	}

	void ReflectorSLAM::MatchRef(vector<double>& ref_match_angle,
		vector<Mark>& ref_match_curr,
		vector<Mark>& ref_match_in_map) {
		bool match;
		file_blog_ << "���е�ref_curr_Ϊ��" << endl;
		for (int i = 0; i < ref_curr_.size(); i++) {
			file_blog_ << "(" << ref_curr_[i].x << "," << ref_curr_[i].y << ")" << endl;
		}
		file_blog_ << "���е�ref_world_Ϊ��" << endl;
		for (int i = 0; i < ref_world_.size(); i++) {
			file_blog_ << "(" << ref_world_[i].x << "," << ref_world_[i].y << ")" << endl;
		}
		file_blog_ << "ƥ���ref_world_Ϊ��" << endl;
		for (int i = 0; i < ref_world_.size(); i++) {
			match = false;
			for (int j = 0; j < ref_in_map_.size(); j++) {
				if (abs(ref_world_[i].x - ref_in_map_[j].x) <= ref_match_thre_
					&& abs(ref_world_[i].y - ref_in_map_[j].y) <= ref_match_thre_) {

					file_blog_ << "(" << ref_world_[i].x << "," << ref_world_[i].y << ")" << endl;
					ref_match_angle.push_back(ref_angle_[i]);
					ref_match_curr.push_back(ref_curr_[i]);
					ref_match_in_map.push_back(ref_in_map_[j]);
					match = true;
				}
			}
		}
		file_blog_ << "ƥ���ref_match_angleΪ��" << endl;
		for (int i = 0; i < ref_match_angle.size(); i++) {
			file_blog_ << ref_match_angle[i] << endl;
		}
		file_blog_ << "ƥ���ref_match_in_mapΪ��" << endl;
		for (int i = 0; i < ref_match_angle.size(); i++) {
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
		ref_angle_.clear();
		auto ranges_rssi = scan.ranges_rssi();
		auto ranges = scan.ranges();
		float start_angle = scan.config().angle_min();
		float angle_increment = scan.config().angle_increment();
		float range_temp;
		float x_sum = 0, y_sum = 0, theta_sum = 0;
		float x_mid, y_mid, theta, theta_mid;
		int num_laser = 0;
		double dist_first;
		//cout << "�˴�ɨ��" << endl;
		for (int i = 0; i < scan.ranges().size(); i++) {
			//cout << ranges_rssi.Get(i) << " ";
			if (ranges_rssi.Get(i) > min_rssi_thre_ && ranges_rssi.Get(i) < max_rssi_thre_) {
				num_laser++;
				/*if (num_laser = 1) dist_first = ranges.Get(i);*/
				/*if (abs(ranges.Get(i) - dist_first) <= 0.4) {*/
					//cout << ranges_rssi.Get(i) << " ";
					theta = start_angle + angle_increment * i;
					x_sum += ranges.Get(i)*cos(theta);
					y_sum += ranges.Get(i)*sin(theta);
					theta_sum += theta;
					cout << ranges_rssi.Get(i) << endl;
					
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
					//cout << endl;
					x_mid = x_sum / num_laser;
					y_mid = y_sum / num_laser;
					theta_mid = theta_sum / num_laser;
					Mark ref_temp = { x_mid, y_mid };
					ref_curr_.push_back(ref_temp);
					ref_angle_.push_back(theta_mid);
				}
				x_sum = 0;
				y_sum = 0;
				theta_sum = 0;
				num_laser = 0;
			}
		}
		GetTime("��ȡ����");
		ref_world_ = transform(ref_curr_, pose_now_);//������һ֡����λ�˽����������ӳ�������ϵת������������ϵ
		MarkTran2Graph();
		DrawMark();

		GetTime("�����۲ⷴ���");
		return;
	}

	Pose ReflectorSLAM::TransformPose2Vehicle(const Pose &reference, const Pose &object) {
		double x, y, theta;
		theta = reference.theta + object.theta;
		x = reference.x + object.x*cos(theta) + object.y*sin(theta);
		y = reference.y - object.x*sin(theta) + object.y*cos(theta);
		return {x,y,theta};
	}

	void ReflectorSLAM::GetTime(string item) {
		QueryPerformanceCounter(&nEndTime_);//ֹͣ��ʱ  
		time_ = (double)(nEndTime_.QuadPart - nBeginTime_.QuadPart) / (double)nFreq_.QuadPart;//�������ִ��ʱ�䵥λΪs  
		nBeginTime_ = nEndTime_;
		file_blog_ << item <<"��ʱ��"<< time_ * 1000 << " ms" << endl;
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
		//������󷽳�Ax=b
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
		file_blog_ << "����󷽳����߶�λ��" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta * 180 / 3.1415926 << "��)" << endl;
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
				if (alpha1 - alpha2 > 3.1415926) {
					theta_all.push_back(alpha1 - alpha2 - 2 * 3.1415926);
				}
				else if (alpha1 - alpha2 <= -3.1415926) {
					theta_all.push_back(alpha1 - alpha2 + 2 * 3.1415926);
				}
				else {
					theta_all.push_back(alpha1 - alpha2);
				}
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
			file_blog_ << "���Ƕ�λ��" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta * 180 / 3.1415926 << "��)" << endl;
		}
	}

	void ReflectorSLAM::ComputePoseWithCeres(const vector<double> &ref_match_angle, const vector<Mark> &ref_match_curr,const vector<Mark> &ref_match_in_map) {
		ceres::Problem location_problem;
		double x = pose_now_.x;
		double y = pose_now_.y;
		double theta = pose_now_.theta;

		for (int i = 0; i < ref_match_angle.size(); i++) {
			auto cost_function = new ceres::AutoDiffCostFunction<LocalizationCost, 3, 1, 1, 1>(new LocalizationCost(ref_match_angle[i], ref_match_curr[i],ref_match_in_map[i]));
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
		file_blog_ << "����ceres���߶�λ��" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta / 3.14 * 180 << "��)" << endl;
		return;
	}

	void ReflectorSLAM::Location() {
		vector<Mark> ref_match_in_map, ref_match_curr;
		vector<double> ref_match_angle;
		MatchRef(ref_match_angle, ref_match_curr, ref_match_in_map); //ƥ�䷴���
		GetTime("ƥ�䷴���");
		if (ref_match_angle.size() >= 3) {
			ComputePoseWithAngleAndDistance(ref_match_curr, ref_match_in_map);
			ComputePoseWithCeres(ref_match_angle,ref_match_curr,ref_match_in_map);
			pose_vehicle_ = TransformPose2Vehicle(install_position_, pose_now_);
			//ComputePoseWithAngle(ref_match_angle, ref_match_in_map);
			cout << "AGV��ǰλ�ˣ�" << "(" << pose_now_.x << "," << pose_now_.y << "," << pose_now_.theta / 3.14 * 180 << "��)" << endl;
		}
		else {
			//cout << "��ǰ�����ƥ����Ŀ����3�����޷����ж�λ��" << endl << endl;
			add_ref_permissed_ = false;
			file_blog_ << endl;
		}

		GetTime("����λ��");
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
				double theta = -3.1415926 + atan2(pose_now_.y, pose_now_.x) - pose_now_.theta;
				double dist = sqrt(pose_now_.x*pose_now_.x + pose_now_.y * pose_now_.y);
				pose_now_ = { dist*cos(theta),dist*sin(theta),-pose_now_.theta };

				ref_in_map_ = transform(ref_in_map_, pose_now_);
				CancelDrawMarkInMap();
				MarkInMapTran2Graph();
				DrawMarkInMap();

				ref_world_ = transform(ref_world_, pose_now_);
				MarkTran2Graph();
				DrawMark();

				file_ref_write_.open(file_name_, ios::out);
				for (int i = 0; i < ref_in_map_.size(); i++) {
					file_ref_write_ << "(" << ref_in_map_[i].x << "," << ref_in_map_[i].y << ")" << endl;
				}

				break;
			}
		}
		
		is_operating_ = false;
		str.set_str("Transform to Current Frame successfully!");
		return str;
	}


	
}