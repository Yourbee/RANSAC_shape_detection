// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/console.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>

using std::cout;
using std::endl;
namespace bpo = boost::program_options;
namespace blog = boost::log;
namespace bfs = boost::filesystem;

//wrong
int readTXT(std::string file_path, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	fstream file;
	file.open(file_path, ios::in);
	if (!file)
	{
		cout << "Can't open " << file_path << endl;
		return -1;
	}
	std::string line;
	while (std::getline(file, line))
	{
		std::stringstream ss(line);
		std::vector<std::string> v_str;
		std::string s;
		while (std::getline(ss, s, ' '))
		{
			v_str.push_back(s);
		}
		if (v_str.size() < 6) {
			cout << "格式不支持" << endl;
			return -1;
		}

		cout << std::fixed << std::setprecision(8);
		pcl::PointXYZRGB p;
		p.x = std::stod(v_str[0]);
		p.y = std::stod(v_str[1]);
		p.z = std::stod(v_str[2]);

		p.r = std::stoi(v_str[3]);
		p.g = std::stoi(v_str[4]);
		p.b = std::stoi(v_str[5]);

		cloud.push_back(p);
	}
	cout << cloud[0].x << " "
		<< cloud[0].y << " "
		<< cloud[0].z << " "
		<< cloud[0].r << " "
		<< cloud[0].g << " "
		<< cloud[0].b << " " << endl;
	cout << "File read " << cloud.width * cloud.height << " points" << endl;
	return 0;
}

int move2XOY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	if (0 == cloud->width * cloud->height) return -1;

	float x_min = std::numeric_limits<float>::max();
	float y_min = std::numeric_limits<float>::max();
	float z_min = std::numeric_limits<float>::max();

	float x_max = -std::numeric_limits<float>::max();
	float y_max = -std::numeric_limits<float>::max();
	for (const auto& p : *cloud)
	{
		if (p.x < x_min) x_min = p.x;
		if (p.y < y_min) y_min = p.y;
		if (p.z < z_min) z_min = p.z;

		if (p.x > x_max) x_max = p.x;
		if (p.y > y_max) y_max = p.y;
	}

	float x_mid = (x_min + x_max) / 2;
	float y_mid = (y_min + y_max) / 2;

	for (auto& p : *cloud)
	{
		p.x -= x_mid;
		p.y -= y_mid;
		p.z -= z_min;
	}
	BOOST_LOG_TRIVIAL(trace) << "x_min " << x_min;
	BOOST_LOG_TRIVIAL(trace) << "y_min " << y_min;
	BOOST_LOG_TRIVIAL(trace) << "z_min " << z_min;
	BOOST_LOG_TRIVIAL(trace) << "x_max " << x_max;
	BOOST_LOG_TRIVIAL(trace) << "y_max " << y_max;
	return 0;
}

int showPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr v_cloud)
{
	cout << ">>>" << endl;
	if (0 == v_cloud->width * v_cloud->height)
	{
		cout << "No point in pc." << endl;
		return -1;
	}
	cout << "Pc size " << v_cloud->width * v_cloud->height << endl;

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	std::string cloud_name = "Cloud";
	viewer->addPointCloud<pcl::PointXYZRGB>(v_cloud, cloud_name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
	viewer->spin();

	return 0;
}

int showPoint(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> v_cloud)
{
	cout << ">>>" << endl;
	if (0 == v_cloud.size())
	{
		cout << "Pointcloud std::vector has no member." << endl;
		return -1;
	}

	cout << "Pointcloud std::vector size " << v_cloud.size() << endl;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	for (size_t i = 0; i < v_cloud.size(); ++i)
	{
		cout << "Cloud " << i << " " << v_cloud[i]->width * v_cloud[i]->height << " points." << endl;
		std::string cloud_name = "Cloud " + std::to_string(i);
		viewer->addPointCloud<pcl::PointXYZRGB>(v_cloud[i], cloud_name);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
		
	}
	viewer->spin();

	return 0;
}

int showPointFlat(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> v_cloud)
{
	cout << ">>>" << endl;
	if (0 == v_cloud.size())
	{
		cout << "Pointcloud std::vector has no member." << endl;
		return -1;
	}

	cout << "Pointcloud std::vector size " << v_cloud.size() << endl;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	for (size_t i = 0; i < v_cloud.size(); ++i)
	{
		for (auto& p : *v_cloud[i])
		{
			p.z = 0;
		}

		cout << "Cloud " << i << " " << v_cloud[i]->width * v_cloud[i]->height << " points." << endl;
		std::string cloud_name = "Cloud " + std::to_string(i);
		viewer->addPointCloud<pcl::PointXYZRGB>(v_cloud[i], cloud_name);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);

	}

	pcl::PointXYZ p1, p2, p3, p4;
	p1.x = 0; p1.y = 0; p1.z = 0;
	p2.x = 1; p2.y = 1; p2.z = 1;
	viewer->addLine(p1, p2);

	viewer->spin();

	return 0;
}

int initBoostLog(int level)
{
	switch (level)
	{
	case 1: blog::core::get()->set_filter(blog::trivial::severity >= blog::trivial::trace); break;
	case 2: blog::core::get()->set_filter(blog::trivial::severity >= blog::trivial::debug); break;
	case 3: blog::core::get()->set_filter(blog::trivial::severity >= blog::trivial::info); break;
	case 4: blog::core::get()->set_filter(blog::trivial::severity >= blog::trivial::warning); break;
	case 5: blog::core::get()->set_filter(blog::trivial::severity >= blog::trivial::error); break;
	case 6: blog::core::get()->set_filter(blog::trivial::severity >= blog::trivial::fatal); break;
	default: BOOST_LOG_TRIVIAL(error) << "boost log severity setting wrong"; return -1;  break;
	}
	return 0;
}

size_t calcSize(std::vector<std::vector<cv::Point2f>> p_2d)
{
	size_t num = 0;
	for (auto& p : p_2d)
	{
		num += p.size();
	}
	return num;
}

float ransacHouseAreaCalc(
	PointCloud& pc,
	int const verticalThreshhold,
	RansacShapeDetector::Options& ransacOptions,
	bool flag_show = false)
{
	//void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
	pc.calcNormals(.01f * pc.getScale());

	RansacShapeDetector detector(ransacOptions); // the detector object
	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	size_t n_remained = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
		// returns number of unassigned points
		// the array shapes is filled with pointers to the detected shapes
		// the second element per shapes gives the number of points assigned to that primitive (the support)
		// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
		// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
		// the points of shape i are found in the range
		// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	BOOST_LOG_TRIVIAL(debug) << "面提取结果：";
	for (int i = 0; i < shapes.size(); ++i)
	{
		Vec3f normal(0, 0, 0);
		shapes[i].first->Normal(normal, &normal);
		BOOST_LOG_TRIVIAL(debug) << "shape " << i 
			<< " consists of " << shapes[i].second << " \t"
			<< normal[0] << " " << normal[1] << " " << normal[2];
	}
	BOOST_LOG_TRIVIAL(debug) << "remaining unassigned points: " << n_remained;

	//筛选,阈值：和（0，0，1）角度， 和最小点数
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> v_cloud;
	std::vector<Eigen::Vector4f> v_normals;
	size_t rend = 0;
	size_t n_result = 0;
	BOOST_LOG_TRIVIAL(debug) << "垂直面筛选：";
	for (size_t i = 0; i < shapes.size(); ++i)
	{
		Vec3f z(0, 0, 1);
		float cos_theta = acosf(shapes[i].first->NormalDeviation(z, z)) / M_PI * 180;
		if (abs(cos_theta - 90) < verticalThreshhold 
			&& ransacOptions.m_minSupport < shapes[i].second)
		{
			BOOST_LOG_TRIVIAL(debug) << "shape " << i << " acos_theta " << cos_theta;
			n_result += shapes[i].second;

			int random_r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			int random_g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			int random_b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
			result->reserve(shapes[i].second);
			for (size_t p_i = pc.size() - rend - shapes[i].second; p_i < pc.size() - rend; ++p_i)
			{
				pcl::PointXYZRGB point;
				point.x = pc[p_i][0]; point.y = pc[p_i][1]; point.z = pc[p_i][2];
				point.r = random_r; point.g = random_g; point.b = random_b;
				result->push_back(point);
			}
			v_cloud.push_back(result);
			Vec3f dist, normal;
			shapes[i].first->Normal(dist, &normal);
			Eigen::Vector4f coff(normal[0], normal[1], normal[2], dist[0]);
			if (coff[3] < 0) coff *= -1;
			BOOST_LOG_TRIVIAL(debug) << "coff " << coff.transpose();
			v_normals.push_back(coff);
		}
		rend += shapes[i].second;
	}
	//showPoint(v_cloud);

	//计算相互之间的垂直度
	BOOST_LOG_TRIVIAL(debug) << "相互垂直度损失：";
	std::vector<float> loss_rect(v_cloud.size(), 0);
	for (size_t i = 0; i < v_cloud.size(); ++i)
	{
		for (size_t j = 0; j < v_cloud.size(); ++j)
		{
			if (i == j) continue;

			Eigen::Vector3f coef_i(v_normals[i].head(3));
			Eigen::Vector3f coef_j(v_normals[j].head(3));
			float cos_theta =
				acosf(coef_i.dot(coef_j) / (coef_i.norm() * coef_j.norm())) / M_PI * 180;
			loss_rect[i] += std::min(
				std::min(cos_theta, abs(cos_theta - 90)), 180 - cos_theta) / (v_cloud.size() - 1);
		}
		BOOST_LOG_TRIVIAL(debug) << i << " " << loss_rect[i];
	}
	// 排序
	std::vector<size_t> arg_loss_rect;
	arg_loss_rect.resize(loss_rect.size());
	iota(arg_loss_rect.begin(), arg_loss_rect.end(), 0);  //0 1 2 3 4
	sort(arg_loss_rect.begin(), arg_loss_rect.end(),
		[&loss_rect](size_t i1, size_t i2) {return loss_rect[i1] < loss_rect[i2]; });

	std::string s_arg_loss_rect;
	for (size_t i = 0; i < arg_loss_rect.size(); i++)
	{
		s_arg_loss_rect += std::to_string(arg_loss_rect[i]) + " ";
	}
	BOOST_LOG_TRIVIAL(debug) << s_arg_loss_rect;
	BOOST_LOG_TRIVIAL(debug) << "Min loss normal " << v_normals[arg_loss_rect[0]].head(3).transpose();

	//转到OpenCV格式
	std::vector<std::vector<cv::Point2f>> p_2ds;
	std::vector<cv::Point2f> normals_2d;
	for (size_t i = 0; i < v_cloud.size(); ++i)
	{
		if (loss_rect[i] > 20)
		{
			continue;
		}
		std::vector<cv::Point2f> p_2d;
		for (auto& p : *v_cloud[i])
		{
			p_2d.emplace_back(p.x, p.y);
		}
		p_2ds.push_back(p_2d);
		normals_2d.emplace_back(v_normals[i][0], v_normals[i][1]);

	}
	if (flag_show)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		for (size_t i = 0; i < v_cloud.size(); ++i)
		{
			std::string cloud_name = "Cloud " + std::to_string(i);
			viewer->addPointCloud<pcl::PointXYZRGB>(v_cloud[i], cloud_name);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
		}
		viewer->addCoordinateSystem();
		//viewer->spin();
	}

	//旋转最垂直边到x轴，
	Eigen::Vector4f l_rl = v_normals[arg_loss_rect[0]];
	double theta_2d = acos(-l_rl[1] / sqrt(1.0 * l_rl[0] * l_rl[0] + 1.0 * l_rl[1] * l_rl[1])) / M_PI * 180.0;
	BOOST_LOG_TRIVIAL(debug) << "旋转角度：" << theta_2d;
	theta_2d = l_rl[0] < 0 ? -theta_2d : theta_2d;
	cv::Mat rotation_mat = cv::getRotationMatrix2D(cv::Point2f(0, 0), theta_2d, 1);

	std::vector< std::vector<cv::Point2f>> rotated_points;
	for (auto& p : p_2ds)
	{
		std::vector<cv::Point2f> rotated_point;
		rotated_point.resize(p.size());
		cv::transform(p, rotated_point, rotation_mat);
		rotated_points.push_back(rotated_point);
	}
	std::vector<cv::Point2f> rotated_normals;
	cv::transform(normals_2d, rotated_normals, rotation_mat);

	BOOST_LOG_TRIVIAL(debug) << "最终面法向量：";
	for (auto& n : rotated_normals)
	{
		BOOST_LOG_TRIVIAL(debug) << n.x << " " << n.y;
	}

	//遍历 寻找x y最大最小值
	float x_minus = 0, y_minus = 0, x_plus = 0, y_plus = 0;
	for (auto& rotated_point : rotated_points)
	{
		for (auto& p : rotated_point)
		{
			if (x_minus > p.x) x_minus = p.x;
			else if (x_plus < p.x) x_plus = p.x;

			if (y_minus > p.y) y_minus = p.y;
			else if (y_plus < p.y) y_plus = p.y;
		}
	}

	if (x_plus - 0 < 0.1 || 0 - x_minus < 0.1 || y_plus - 0 < 0.1 || 0 - y_minus < 0.1)
	{
		BOOST_LOG_TRIVIAL(error) << "不完整房屋。";
		return -1;
	}
	float min_area_rect = (x_plus - x_minus) * (y_plus - y_minus);
	BOOST_LOG_TRIVIAL(debug) << "外接面积：" << min_area_rect;

	// 内接面积, xy上最大 最小为边边界
	float x_minus_in = x_minus, y_minus_in = y_minus;
	float x_plus_in = x_plus, y_plus_in = y_plus;
	std::vector<bool> has_walls(4, false);
	enum class Direction : unsigned int {y_minus, x_minus, y_plus, x_plus};
	for (size_t i = 0; i < rotated_normals.size(); ++i)
	{
		std::vector<cv::Point2f>& p = rotated_points[i];
		if (-0.5 > rotated_normals[i].y)
		{
			float temp = (*std::max_element(p.begin(), p.end(),
				[](cv::Point2f& i, cv::Point2f& j) {return i.y < j.y; })).y;
			y_minus_in = temp > y_minus_in ? temp : y_minus_in;
			has_walls[static_cast<int> (Direction::y_minus)] = true;
		}
		else if (0.5 < rotated_normals[i].y)
		{
			float temp = (*std::min_element(p.begin(), p.end(),
				[](cv::Point2f& i, cv::Point2f& j) {return i.y < j.y; })).y;
			y_plus_in = temp < y_plus_in ? temp : y_plus_in;
			has_walls[static_cast<int> (Direction::y_plus)] = true;
		}
		if (-0.5 > rotated_normals[i].x)
		{
			float temp = (*std::max_element(p.begin(), p.end(),
				[](cv::Point2f& i, cv::Point2f& j) {return i.x < j.x; })).x;
			x_minus_in = temp > x_minus_in ? temp : x_minus_in;
			has_walls[static_cast<int> (Direction::x_minus)] = true;
		}
		else if (0.5 < rotated_normals[i].x)
		{
			float temp = (*std::min_element(p.begin(), p.end(),
				[](cv::Point2f& i, cv::Point2f& j) {return i.x < j.x; })).x;
			x_plus_in = temp < x_plus_in ? temp : x_plus_in;
			has_walls[static_cast<int> (Direction::x_plus)] = true;
		}
	}
	float max_area_rect = (x_plus_in - x_minus_in) * (y_plus_in - y_minus_in);
	BOOST_LOG_TRIVIAL(debug) << "内接面积：" << max_area_rect;

	if (0 == std::count(has_walls.begin(), has_walls.end(), false))
	{
		BOOST_LOG_TRIVIAL(debug) << "该房子四面有墙：";
		float const fix = 0.2;
		BOOST_LOG_TRIVIAL(debug) << "补偿值：" << fix;
		max_area_rect = (x_plus_in - x_minus_in + fix) * (y_plus_in - y_minus_in + fix);
		BOOST_LOG_TRIVIAL(debug) << "补充内接面积：" << max_area_rect;
	}

	if (flag_show)
	{
		// 显示带框 二维点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_points_3d(new pcl::PointCloud<pcl::PointXYZRGB>);
		rotated_points_3d->reserve(calcSize(rotated_points));
		for (auto& rotated_point : rotated_points)
		{
			for (auto& p : rotated_point)
			{
				pcl::PointXYZRGB point;
				point.x = p.x; point.y = p.y; point.z = 0;
				point.r = 255; point.g = 255; point.b = 255;
				rotated_points_3d->push_back(point);
			}
		}

		pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("3D Viewer"));

		viewer2->addPointCloud<pcl::PointXYZRGB>(rotated_points_3d, "rotated_points_3d");
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "rotated_points_3d");

		pcl::PointXYZ p1, p2, p3, p4;
		p1.x = x_minus; p1.y = y_minus; p1.z = 0;
		p2.x = x_plus; p2.y = y_minus; p2.z = 0;
		p3.x = x_plus; p3.y = y_plus; p3.z = 0;
		p4.x = x_minus; p4.y = y_plus; p4.z = 0;
		viewer2->addLine(p1, p2, "line1");
		viewer2->addLine(p2, p3, "line2");
		viewer2->addLine(p3, p4, "line3");
		viewer2->addLine(p4, p1, "line4");

		pcl::PointXYZ p1_in, p2_in, p3_in, p4_in;
		p1_in.x = x_minus_in; p1_in.y = y_minus_in; p1_in.z = 0;
		p2_in.x = x_plus_in; p2_in.y = y_minus_in; p2_in.z = 0;
		p3_in.x = x_plus_in; p3_in.y = y_plus_in; p3_in.z = 0;
		p4_in.x = x_minus_in; p4_in.y = y_plus_in; p4_in.z = 0;
		viewer2->addLine(p1_in, p2_in, "line1_in");
		viewer2->addLine(p2_in, p3_in, "line2_in");
		viewer2->addLine(p3_in, p4_in, "line3_in");
		viewer2->addLine(p4_in, p1_in, "line4_in");

		viewer2->addCoordinateSystem();
		viewer2->spin();
	}

	return max_area_rect;
}

boost::regex re("(\\d+)");
boost::match_results<std::string::const_iterator> what1, what2;

template <typename T>
T st2num(const std::string& Text)
{
	std::stringstream ss(Text);
	T result;
	return ss >> result ? result : 0;
}
struct mysort
{
	bool operator ()(bfs::path p_a, bfs::path p_b)
	{
		const std::string a = p_a.filename().string();
		const std::string b = p_b.filename().string();
		boost::regex_search(a.cbegin(), a.cend(), what1, re,
			boost::match_default);
		boost::regex_search(b.cbegin(), b.cend(), what2, re,
			boost::match_default);

		return st2num<int>(what1[1]) < st2num<int>(what2[1]);

	}
};

int main(int argc, char** argv)
{
	blog::add_console_log(std::cout, blog::keywords::format = "[%Severity%] %Message%");

	std::string file;
	int verticalThreshhold;
	bool flag_show;
	int n_calc;
	RansacShapeDetector::Options ransacOptions;
	bpo::options_description opt("all options");
	
	int log_severity;
	opt.add_options()
		// 全名和缩写中间不能有空格
		("file,f", bpo::value<std::string>(&file), "point cloud file or fold path")
		("verticality,v", bpo::value<int>(&verticalThreshhold)->default_value(10), "wall verticality threshold(degree)")
		("distance,d", bpo::value<float>(&ransacOptions.m_epsilon), "distance threshold(default .005f of bounding box width)")
		("resolution,r", bpo::value<float>(&ransacOptions.m_bitmapEpsilon), "bitmap resolution(default .01f of bounding box width)")
		("normal,n", bpo::value<float>(&ransacOptions.m_normalThresh)->default_value(.9f), "cos of the maximal normal deviation")
		("mini,m", bpo::value<unsigned int>(&ransacOptions.m_minSupport), "minimal number of points required for a primitive")
		("probability,p", bpo::value<float>(&ransacOptions.m_probability)->default_value(.01f), "probability which a primitive is overlooked")
		("calc,c", bpo::value<int>(&n_calc)->default_value(7), "number of calculations to take the medium")
		("show,s", bpo::value<bool>(&flag_show)->default_value(true), "flag to show pc result")
		("log,l", bpo::value<int>(&log_severity)->default_value(3), "boost log severity, 1=trace, 2=debug, 3=info, 4=warning, 5=error, 6=fatal")
		("help,h", "计算单栋点云房屋面积");

	bpo::variables_map vm;
	try {
		bpo::store(parse_command_line(argc, argv, opt), vm);
	}
	catch (...) {
		BOOST_LOG_TRIVIAL(error) << "输入的参数中存在未定义的选项！\n";
		return -1;
	}
	bpo::notify(vm);

	initBoostLog(log_severity);

	if (vm.count("help")) {
		BOOST_LOG_TRIVIAL(info) << opt << std::endl;
		return 0;
	}

	if (!vm.count("file")) {
		BOOST_LOG_TRIVIAL(error) << "必须输入 文件/文件夹 名！\n";
		return -1;
	}

	bfs::path p(file);
	std::vector<bfs::path> v_files;
	try
	{
		if (bfs::is_regular_file(p))
		{
			v_files.push_back(p);
		}
		else if (bfs::is_directory(p))
		{
			BOOST_LOG_TRIVIAL(debug) << p << " is a directory.";
			for (auto& x : bfs::directory_iterator(p))
			{
				if (bfs::extension(x) == ".pcd")
				{
					v_files.push_back(x.path());
				}
			}
		}
		else
			cout << p << " exists, but is not a regular file or directory\n";
	}
	catch (const bfs::filesystem_error& ex)
	{
		cout << ex.what() << endl;
		return -1;
	}

	std::sort(v_files.begin(), v_files.end(), mysort());

	for (auto& file : v_files)
	{
		cout << file.string() << " ";
		// 读取点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile <pcl::PointXYZRGB>(file.string(), *cloud) == -1)
		{
			BOOST_LOG_TRIVIAL(error) << "Cloud reading failed." << endl;
			return -1;
		}

		std::uint32_t n_points = cloud->width * cloud->height;
		BOOST_LOG_TRIVIAL(debug) << "Total points: " << n_points << endl;

		move2XOY(cloud);  //移动到原点

		//x, y, z最大最小值，用来计算尺度
		float min_x = std::numeric_limits<float>::max();
		float min_y = std::numeric_limits<float>::max();
		float min_z = std::numeric_limits<float>::max();

		float max_x = -std::numeric_limits<float>::max();
		float max_y = -std::numeric_limits<float>::max();
		float max_z = -std::numeric_limits<float>::max();

		// 转换到ransac库的格式
		PointCloud pc;
		pc.reserve(n_points);
		for (auto& p : *cloud)
		{
			pc.push_back(Point(Vec3f(p.x, p.y, p.z)));

			if (p.x < min_x) min_x = p.x;
			else if (p.x > max_x) max_x = p.x;

			if (p.y < min_y) min_y = p.y;
			else if (p.y > max_y) max_y = p.y;

			if (p.z < min_z) min_z = p.z;
			else if (p.z > max_z) max_z = p.z;
		}
		// set the bbox in pc
		pc.setBBox(Vec3f(min_x, min_y, min_z), Vec3f(max_x, max_y, max_z));
		BOOST_LOG_TRIVIAL(debug) << "scale: " << pc.getScale();

		if (!vm.count("distance")) {
			ransacOptions.m_epsilon = .005f * pc.getScale(); // set distance threshold to .005f of bounding box width
			// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
		}
		if (!vm.count("resolution")) {
			ransacOptions.m_bitmapEpsilon = .01f * pc.getScale(); // set bitmap resolution to .01f of bounding box width
			// NOTE: This threshold is NOT multiplied internally!
		}
		if (!vm.count("mini")) {
			// the minimal number of points required for a primitive
			ransacOptions.m_minSupport = n_points * 0.01f;
		}

		BOOST_LOG_TRIVIAL(debug) << "verticalThreshhold: " << verticalThreshhold;
		BOOST_LOG_TRIVIAL(debug) << "distance lThreshhold: " << ransacOptions.m_epsilon;
		BOOST_LOG_TRIVIAL(debug) << "bitmap resolution: " << ransacOptions.m_bitmapEpsilon;
		BOOST_LOG_TRIVIAL(debug) << "cos of the maximal normal deviation: " << ransacOptions.m_normalThresh;
		BOOST_LOG_TRIVIAL(debug) << "minimal number of points: " << ransacOptions.m_minSupport;
		BOOST_LOG_TRIVIAL(debug) << "probability: " << ransacOptions.m_probability;
		std::vector<float> v_area;
		v_area.reserve(n_calc);
		for (size_t i = 0; i < n_calc; ++i)
		{
			v_area.push_back(ransacHouseAreaCalc(pc, verticalThreshhold, ransacOptions, flag_show));
		}
		sort(v_area.begin(), v_area.end());
		cout << "内面积: "
			<< v_area[n_calc / 2]<< endl;
	}
}
