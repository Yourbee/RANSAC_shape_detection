// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <iostream>

using std::cout;
using std::endl;

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

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << "Usage: main <pc path>" << endl;
		return -1;
	}

	// 读取点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//std::string file = "house1.pcd";
	//std::string file_path = "C:/Users/myqiu/Desktop/pts_data_file/house/" + file;
	std::string file_path = argv[1];
	if (pcl::io::loadPCDFile <pcl::PointXYZRGB>(file_path, *cloud) == -1)
	{
		cout << "Cloud reading failed." << endl;
		return -1;
	}
	std::uint32_t n_points = cloud->width * cloud->height;
	cout << "Total points: " << n_points << endl;

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
	cout << "Ransac added " << pc.size() << " points" << endl;

	// set the bbox in pc
	pc.setBBox(Vec3f(min_x, min_y, min_z), Vec3f(max_x, max_y, max_z));
	cout << "scale " << " " << pc.getScale() << endl;
	//void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
	pc.calcNormals(.01f * pc.getScale());

	//参数
	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .005f * pc.getScale(); // set distance threshold to .01f of bounding box width
		// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .01f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
		// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // ~25 degree, this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = n_points * 0.01f; // this is the minimal numer of points required for a primitive
	ransacOptions.m_probability = .01f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object
	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
		// returns number of unassigned points
		// the array shapes is filled with pointers to the detected shapes
		// the second element per shapes gives the number of points assigned to that primitive (the support)
		// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
		// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
		// the points of shape i are found in the range
		// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	cout << "remaining unassigned points " << remaining << endl;
	for(int i = 0; i < shapes.size(); ++i)
	{
		Vec3f normal(0, 0, 0);
		shapes[i].first->Normal(normal, &normal);
		cout << "shape " << i << " consists of " << shapes[i].second << " \t"
			<< normal[0] << " " << normal[1] << " " << normal[2] << " "
			<< endl;
	}
	cout << endl;

	//筛选,阈值：和（0，0，1）角度， 和最小点数
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> v_cloud;
	std::vector<Eigen::Vector4f> v_normals;
	size_t rend = 0;
	size_t n_result = 0;
	for (size_t i = 0; i < shapes.size(); ++i)
	{
		Vec3f z(0, 0, 1);
		float cos_theta = acosf(shapes[i].first->NormalDeviation(z, z)) / M_PI * 180;
		if ( abs(cos_theta - 90) < 10 && ransacOptions.m_minSupport < shapes[i].second)
		{
			cout << "shape " << i <<" cos_theta " << cos_theta << endl;
			n_result += shapes[i].second;

			int random_r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			int random_g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			int random_b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
			result->reserve(shapes[i].second);
			for (size_t p_i = pc.size() - rend - shapes[i].second; p_i < pc.size() - rend; ++p_i)
			{
				//cout << "p_i " << p_i << endl;
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
			cout << "coff " << coff.transpose() << endl;
			v_normals.push_back(coff);
		}
		rend += shapes[i].second;
	}
	showPoint(v_cloud);

	//计算相互之间的垂直度
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
		cout << i << " " << loss_rect[i] << endl;
	}
	// 排序
	std::vector<size_t> arg_loss_rect;
	arg_loss_rect.resize(loss_rect.size());
	iota(arg_loss_rect.begin(), arg_loss_rect.end(), 0);  //0 1 2 3 4
	sort(arg_loss_rect.begin(), arg_loss_rect.end(),
		[&loss_rect](size_t i1, size_t i2) {return loss_rect[i1] < loss_rect[i2]; });

	for (size_t i = 0; i < arg_loss_rect.size(); i++)
	{
		cout << arg_loss_rect[i] << " ";
	}
	cout << "Min loss normal " << v_normals[arg_loss_rect[0]].head(3).transpose() << endl;
	cout << endl;

	//转到OpenCV格式
	std::vector<cv::Point2f> imagePoints;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	for (size_t i = 0; i < v_cloud.size(); ++i)
	{
		if (loss_rect[i] > 20)
		{
			continue;
		}
		for (auto& p : *v_cloud[i])
		{
			imagePoints.emplace_back(p.x, p.y);
		}
		std::string cloud_name = "Cloud " + std::to_string(i);
		viewer->addPointCloud<pcl::PointXYZRGB>(v_cloud[i], cloud_name);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
	}
	cout << imagePoints.size() << endl;
	viewer->spin();

	//旋转最垂直边到x轴，
	// calculate the least rect loss plane's 2D theta to (0, -1)
	Eigen::Vector4f l_rl = v_normals[arg_loss_rect[0]];
	double theta_2d = acos(-l_rl[1] / sqrt(1.0 * l_rl[0] * l_rl[0] + 1.0 * l_rl[1] * l_rl[1])) / M_PI * 180.0;
	cout << "Rotate degree " << theta_2d << endl;
	theta_2d = l_rl[0] < 0 ? -theta_2d : theta_2d;
	cv::Mat rotation_mat = cv::getRotationMatrix2D(cv::Point2f(0, 0), theta_2d, 1);
	std::vector<cv::Point2f> rotated_points;
	rotated_points.resize(imagePoints.size());
	cv::transform(imagePoints, rotated_points, rotation_mat);
	//拟合矩形
	cv::RotatedRect rect = cv::minAreaRect(rotated_points);
	cv::Point2f P[4];
	rect.points(P);
	//cv::Rect rect = cv::boundingRect(imagePoints);
	//P[0].x = rect.x; P[0].y = rect.y;
	//P[1].x = rect.x + rect.width; P[1].y = rect.y;
	//P[2].x = rect.x + rect.width; P[2].y = rect.y + rect.height;
	//P[3].x = rect.x; P[3].y = rect.y + rect.height;
	//遍历 寻找x y最大最小值
	float x_minus = 0, y_minus = 0, x_plus = 0, y_plus = 0;
	for (auto& p : rotated_points)
	{
		if (x_minus > p.x) x_minus = p.x;
		else if (x_plus < p.x) x_plus = p.x;

		if (y_minus > p.y) y_minus = p.y;
		else if (y_plus < p.y) y_plus = p.y;
	}
	if (x_plus - 0 < 0.1)
	{
		cout << "不完整房屋。" << endl;
		assert(0);
	}
	cout << "面积： " << (x_plus - x_minus) * (y_plus - y_minus) << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_points_3d(new pcl::PointCloud<pcl::PointXYZRGB>);
	rotated_points_3d->reserve(rotated_points.size());
	for (auto& p : rotated_points)
	{
		//cout << "p_i " << p_i << endl;
		pcl::PointXYZRGB point;
		point.x = p.x; point.y = p.y; point.z = 0;
		point.r = 255; point.g = 255; point.b = 255;
		rotated_points_3d->push_back(point);
	}

	pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer2->addPointCloud<pcl::PointXYZRGB>(rotated_points_3d, "rotated_points_3d");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "rotated_points_3d");

	pcl::PointXYZ p1, p2, p3, p4;
	//p1.x = P[0].x; p1.y = P[0].y; p1.z = 0;
	//p2.x = P[1].x; p2.y = P[1].y; p2.z = 0;
	//p3.x = P[2].x; p3.y = P[2].y; p3.z = 0;
	//p4.x = P[3].x; p4.y = P[3].y; p4.z = 0;
	p1.x = x_minus; p1.y = y_minus; p1.z = 0;
	p2.x = x_plus; p2.y = y_minus; p2.z = 0;
	p3.x = x_plus; p3.y = y_plus; p3.z = 0;
	p4.x = x_minus; p4.y = y_plus; p4.z = 0;
	viewer2->addLine(p1, p2, "line1");
	viewer2->addLine(p2, p3, "line2");
	viewer2->addLine(p3, p4, "line3");
	viewer2->addLine(p4, p1, "line4");
	viewer2->addCoordinateSystem();

	viewer2->spin();
}
