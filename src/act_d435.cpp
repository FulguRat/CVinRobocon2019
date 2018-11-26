#include "act_d435.h"

ActD435::ActD435() : align(RS2_STREAM_COLOR), 
					 cloudFiltered(new pointCloud)/*, 
					 viewer("Cloud Viewer")*/,
                     stopFlag(false),
                     stopCounter(0)
{

}

ActD435::~ActD435()
{

}

void ActD435::init(void)
{
	// Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16 , 60);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

	//Instruct pipeline to start streaming with the requested configuration
	pipe.start(cfg);

    // Wait for frames from the camera to settle
    for (int i = 0; i < 10; i++) 
    {
        frameSet = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
}

void ActD435::update(void)
{
	clock_t start = clock();

	// Wait for the next set of frames from the camera
    frameSet = pipe.wait_for_frames();

    float totalTime = float(clock() - start) / float(CLOCKS_PER_SEC) * 1000.0f;
	cout << "retrieve time:" << totalTime << "\t processing time:";
    start = clock();

	//Get processed aligned frame
	alignedFrameSet = align.process(frameSet);

	// Trying to get both other and aligned depth frames
	rs2::video_frame colorFrame = alignedFrameSet.first(RS2_STREAM_COLOR);
	rs2::depth_frame alignedDepthFrame = alignedFrameSet.get_depth_frame();

	// rs2::video_frame colorFrame = frameSet.get_color_frame();
	// rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();
	
	// Map Color texture to each point
    pc.map_to(colorFrame);

	// Generate the pointcloud and texture mappings
	points = pc.calculate(alignedDepthFrame);

	pPointCloud cloudByPoints = pointsToPointCloud(points, colorFrame);

	pcl::PassThrough<pointType> pass;
	pass.setInputCloud(cloudByPoints);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 4.0);
	pass.filter(*cloudFiltered);

	// //blocks until the cloud is actually rendered
	// viewer.showCloud(cloudFiltered);

	/*Mat depthImage(480, 640, CV_16UC1, (void*)alignedDepthFrame.get_data());
	imshow("depth", depthImage);

	Mat colorImage(480, 640, CV_8UC3, (void*)colorFrame.get_data());
	imshow("color", colorImage);

	waitKey(1);*/

	totalTime = float(clock() - start) / float(CLOCKS_PER_SEC) * 1000.0f;
	cout << totalTime << " " << endl;

    stopCounter++;
    if (stopCounter >= 100)
    {
        pipe.stop();
        stopFlag = true; 
    }
}

//======================================================
// getColorTexture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<uint8_t, uint8_t, uint8_t> ActD435::getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int xValue = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int yValue = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = xValue * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = yValue * texture.get_stride_in_bytes(); // Get line width in bytes
    int textIndex = (bytes + strides);

    const auto newTexture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int newText1 = newTexture[textIndex];
    int newText2 = newTexture[textIndex + 1];
    int newText3 = newTexture[textIndex + 2];

    return std::tuple<uint8_t, uint8_t, uint8_t>(newText1, newText2, newText3);
}

//===================================================
// pointsToPointCloud
// - Function is utilized to fill a point cloud
// object with depth and RGB data from a single
// frame captured using the Realsense.
//=================================================== 
pPointCloud ActD435::pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color)
{
    // Object Declaration (Point Cloud)
    pPointCloud cloud(new pointCloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>(sp.width() );   
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto textureCoord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = getColorTexture(color, textureCoord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

pPointCloud ActD435::pointsToPointCloud(const rs2::points& points)
{
    pPointCloud cloud(new pointCloud);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto ptr = points.get_vertices();

    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

bool ActD435::isStoped(void)
{
    return stopFlag;
}