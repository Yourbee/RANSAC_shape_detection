// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::string file = "house_unique1.pcd";
	std::string file_path = "C:/Users/myqiu/Desktop/pts_data_file/house/" + file;
	if (pcl::io::loadPCDFile <pcl::PointXYZRGB>(file_path, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	std::uint32_t n_points = cloud->width * cloud->height;
	std::cout << "Total points: " << n_points << std::endl;

	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();

	float max_x = std::numeric_limits<float>::min();
	float max_y = std::numeric_limits<float>::min();
	float max_z = std::numeric_limits<float>::min();

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
	std::cout << "Ransac added " << pc.size() << " points" << std::endl;
	
	// set the bbox in pc
	pc.setBBox(Vec3f(min_x, min_y, min_z), Vec3f(max_x, max_y, max_z));
	std::cout << "scale " << " " << pc.getScale() << std::endl;
	//void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
	pc.calcNormals(.01f * pc.getScale());

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .005f * pc.getScale(); // set distance threshold to .01f of bounding box width
		// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .01f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
		// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // ~25 degree, this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 1000; // this is the minimal numer of points required for a primitive
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

	std::cout << "remaining unassigned points " << remaining << std::endl;
	for(int i = 0; i < shapes.size(); ++i)
	{
		Vec3f normal(0, 0, 0);
		shapes[i].first->Normal(normal, &normal);
		std::cout << "shape " << i << " consists of " << shapes[i].second << " \t"
			<< normal[0] << " " << normal[1] << " " << normal[2] << " "
			<< std::endl;
	}

	std::cout << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
	result->reserve(pc.size());
	size_t rend = 0;
	std::cout << pc[0][0] << " " << std::endl;
	for (size_t i = 0; i < shapes.size(); ++i)
	{
		Vec3f z(0, 0, 1);
		float cos_theta = acosf(shapes[i].first->NormalDeviation(z, z)) / M_PI * 180;
		if ( abs(cos_theta- 90) < 10)
		{
			std::cout << "shape " << i <<" cos_theta " << cos_theta << std::endl;

			int random_r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			int random_g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			int random_b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));

			for (size_t p_i = pc.size() - rend - shapes[i].second; p_i < pc.size() - rend; ++p_i)
			{
				//std::cout << "p_i " << p_i << std::endl;
				pcl::PointXYZRGB point;
				point.x = pc[p_i][0]; point.y = pc[p_i][1]; point.z = pc[p_i][2];
				point.r = random_r; point.g = random_g; point.b = random_b;
				result->push_back(point);
			}
		}
		rend += shapes[i].second;
	}
	std::cout << "result point " << result->width * result->height << std::endl;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addPointCloud<pcl::PointXYZRGB>(result, "result plain");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "result plain");
	viewer->spin();

	for (auto& p : *result)
	{
		p.z = 0;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer2->addPointCloud<pcl::PointXYZRGB>(result, "result plain");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "result plain");
	viewer2->spin();
}
