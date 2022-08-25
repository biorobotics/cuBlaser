#include <boost/program_options.hpp>

#include <pcl_gen_node.hpp>

// Input methods
#include "input_ximea.hpp"
#include "input_ros.hpp"


ImageToDepthNode::ImageToDepthNode(
    ros::NodeHandle *nh,
    ros::NodeHandle *pnh,
    ImageToDepthNodeConfiguration c) :
    m_nh(*nh),
    m_pnh(*pnh),
    m_output_pub(nh, pnh, c.output_topic, c.frame_id),
    m_output_pub1(nh, pnh, c.vins_pcl_topic, c.frame_id)

{
    m_stopped = false;
    m_properly_configured = true;
    m_config = c;

    // Initialize input source
    switch (m_config.input_source_type)
    {
    case InputSourceType::ROS:
        m_input_source = new SnapshotInputSourceROS(
            nh, pnh, m_config.input_name);
        break;
    
    default:
        ROS_WARN("Unsupported input source");
        m_properly_configured = false;
        break;
    }

    ROS_INFO("Laser width: %d", m_config.detect_laser_width);

    // Try initializing depth triangulation
    try
    {
        m_trig_manager = new TriangulationManager(c.calib_filename);
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Failed to start depth triangulation, check config file");
        m_trig_manager = 0;
    }

    points = new LaserPoints2D();
}

ImageToDepthNode::~ImageToDepthNode()
{
    m_input_source->release();
    delete m_input_source;
    delete m_laser_extractor;
    if (m_trig_manager) delete m_trig_manager;
}

int ImageToDepthNode::start()
{
    bool cb_supported = m_input_source->registerCallback(
        boost::bind(&ImageToDepthNode::imageCallback, this, _1));
    if (!cb_supported)
    {
        ROS_WARN("Input source does not support callbacks");
        return -1;
    }
    m_input_source->start();
    if (!m_input_source->width() || !m_input_source->height()) return -1;

    ROS_INFO("ImageToDepthNode started");

    // Setup laser mask
    cv::Scalar mask1Lo, mask1Hi, mask2Lo, mask2Hi;
    switch (m_config.laser_spectrum_type)
    {
    case RED:
        ROS_INFO("RED Laser");
        mask1Lo = cv::Scalar(0, 120, 30);
        mask1Hi = cv::Scalar(25, 255, 255);
        mask2Lo = cv::Scalar(160, 120, 30);
        mask2Hi = cv::Scalar(180, 255, 255);
        break;
    
    default:
        ROS_INFO("BLUE Laser");
        mask1Lo = cv::Scalar(104, 150, 170);
        mask1Hi = cv::Scalar(135, 255, 255);
        mask2Lo = mask1Lo;
        mask2Hi = mask1Hi;
        break;
    }

    // Initialize laser stripe extraction module
    switch (m_config.stripe_extraction_type)
    {
    case MAXVAL:
        ROS_INFO("Maximum Search Extraction");
        m_laser_extractor = new MaximumSearchLaserExtractor(
            m_input_source->width(), m_input_source->height());
        m_laser_extractor->setLaserMaskingRange(
            mask1Lo, mask1Hi, mask2Lo, mask2Hi);
        break;

    case GGM:
        ROS_INFO("GGM Extraction");
        m_laser_extractor = new GrayGravityLaserExtractor(
            m_input_source->width(), m_input_source->height());
        m_laser_extractor->setLaserMaskingRange(
            mask1Lo, mask1Hi, mask2Lo, mask2Hi);
        dynamic_cast<GrayGravityLaserExtractor *>(
            m_laser_extractor)->setMinLaserPixelStripeWidth(
                2
            );
        dynamic_cast<GrayGravityLaserExtractor *>(
            m_laser_extractor)->setMaxLaserPixelStripeWidth(
                m_config.detect_laser_width
            );
        break;
    
    case STEGERS:
        ROS_INFO("Steger's Extraction");
        m_laser_extractor = new StegersLaserExtractor(
            m_input_source->width(), m_input_source->height(),
            m_config.detect_laser_width);
        m_laser_extractor->setLaserMaskingRange(
            mask1Lo, mask1Hi, mask2Lo, mask2Hi);
        break;
    
    default:
        ROS_WARN("Unsupported laser stripe extractor");
        m_properly_configured = false;
        break;
    }

    ROS_INFO("LaserExtractor initialized");
}

// TODO: try multi-threaded callbacks for multiple images pipelining
void ImageToDepthNode::imageCallback(cv::InputArray frame)
{
    static int seq_counter = 0;
    cv::Mat viz_frame;
    ros::Time frame_ts;
    frame.copyTo(viz_frame);
    
    // Raw image -> 2D sub-pixel laser stripe activations
    LaserPoints2D* pts2D = new LaserPoints2D();
    m_laser_extractor->extract(frame, pts2D);
    // sensor_msgs/PointCloud with u v channel information
    sensor_msgs::PointCloud pcl_chmap;
    sensor_msgs::ChannelFloat32 u_d, v_d;

    // TODO: get actual image timestamp here
    frame_ts = ros::Time::now();

    // Remove detected 2D points close to the boundary
    const int boundary_size = 8;
    for (size_t i=0; i < pts2D->size(); i++)
    {
        cv::Point2f pt(pts2D->x.at(i), pts2D->y.at(i));
        // Crop off boundary points
        if (abs(pt.x - frame.size().width) < boundary_size ||
            abs(pt.y - frame.size().height) < boundary_size ||
            abs(pt.x) < boundary_size ||
            abs(pt.y) < boundary_size)
        {
            continue;
        }
        // Visualize extracted laser stripe activations
        cv::drawMarker(
            viz_frame, pt,
            cv::Scalar(0, 0, 30000),
            cv::MARKER_DIAMOND, 2, 1);
        i += 1;
        // Add to PointCloud u v channel
        u_d.values.push_back(pt.x);
        v_d.values.push_back(pt.y);
    }
    
    cv::imshow("pts2d", viz_frame);
    cv::waitKey(1);

    // 2D sub-pixel laser stripe activations -> 3D point cloud
    PCLRGB pcl_map = m_trig_manager->triangulateImagePoints(pts2D, 0);
    pcl_conversions::toPCL(frame_ts, pcl_map.header.stamp);
    pcl_map.header.seq = seq_counter;

    // m_output_pub.publish(pcl_map);
    m_output_pub.publishFiltered(pcl_map.makeShared());

    // Special output type for vins_refactor: sensor_msgs/PointCloud w/ ch
    pcl_chmap.channels.push_back(u_d);
    pcl_chmap.channels.push_back(v_d);
    for (auto it = pcl_map.begin(); it != pcl_map.end(); it++)
    {
        geometry_msgs::Point32 p;
        p.x = it->x;
        p.y = it->y;
        p.z = it->z;
        pcl_chmap.points.push_back(p);
    }
    pcl_chmap.header.stamp = frame_ts;
    pcl_chmap.header.seq = seq_counter;
    m_output_pub1.publish(pcl_chmap);
    seq_counter++;
}

static void parse_args(
    int argc, char **argv, ImageToDepthNodeConfiguration &config)
{
    using namespace std;
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("laser_width", po::value<int>(), "Width of laser stripe")
        ("laser_color", po::value<string>(), "Color of laser stripe")
        ("extraction_method", po::value<string>(), "Extraction method")
        ("input_source_type", po::value<string>(), "Input source type")
        ("input_name", po::value<string>(), "Option for selected input source")
        ("output_topic", po::value<string>(), "Output point cloud topic")
        ("vins_pcl_topic", po::value<string>(), "sensor_msgs/PointCloud topic")
        ("frame_id", po::value<string>(), "Point cloud frame name")
        ("calib_filename", po::value<string>(), "Calibration file path");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(0);
    }

    if (vm.count("laser_width"))
    {
        config.detect_laser_width = vm["laser_width"].as<int>();
    }

    if (vm.count("laser_color"))
    {
        if (vm["laser_color"].as<string>() == "RED")
        {
            config.laser_spectrum_type = RED;
        }
        else
        {
            config.laser_spectrum_type = BLUE;
        }
    }

    if (vm.count("extraction_method"))
    {
        if (vm["extraction_method"].as<string>() == "STEGERS")
        {
            config.stripe_extraction_type = STEGERS;
        }
        else if (vm["extraction_method"].as<string>() == "MAXVAL")
        {
            config.stripe_extraction_type = MAXVAL;
        }
        else
        {
            config.stripe_extraction_type = GGM;
        }
    }

    if (vm.count("input_source_type"))
    {
        string typ = vm["input_source_type"].as<string>();
        if (typ == "ROS")
        {
            config.input_source_type = ROS;
        }
        else if (typ == "V4L_RGB565")
        {
            config.input_source_type = V4L_RGB565;
        }
        else if (typ == "XIMEA")
        {
            config.input_source_type = XIMEA;
        }
    }

    if (vm.count("input_name"))
    {
        config.input_name = vm["input_name"].as<string>();
    }

    if (vm.count("output_topic"))
    {
        config.output_topic = vm["output_topic"].as<string>();
    }

    if (vm.count("vins_pcl_topic"))
    {
        config.vins_pcl_topic = vm["vins_pcl_topic"].as<string>();
    }

    if (vm.count("frame_id"))
    {
        config.frame_id = vm["frame_id"].as<string>();
    }

    if (vm.count("calib_filename"))
    {
        config.calib_filename = vm["calib_filename"].as<string>();
    }
}

int main(int argc, char **argv)
{
    // Options default
    ImageToDepthNodeConfiguration config {
        .input_source_type = ROS,
        .input_name = "/blaser_camera/image_lexp",
        .output_topic = "/blaser_pcl",
        .vins_pcl_topic = "/blaser_pcl_ch",
        .frame_id = "blaser_camera",
        .calib_filename = "",
        .stripe_extraction_type = STEGERS,
        .laser_spectrum_type = BLUE,
        .detect_laser_width = 6,
    };

    parse_args(argc, argv, config);

    ros::init(argc, argv, "image_to_depth_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh;


    ImageToDepthNode image_to_depth_node(&nh, &pnh, config);

    ROS_INFO("Initialized ImageToDepthNode");

    image_to_depth_node.start();

    ros::spin();

}