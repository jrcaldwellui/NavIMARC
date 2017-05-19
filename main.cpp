int main(int argc, char *argv[]) try
{
    //Initialize Node and handles
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    //Get Serial Parameters
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    int serial_baudrate;
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);
    
    //Get Scanner Parameters
    int rotation_speed;
    nh_private.param<int>("rotation_speed", rotation_speed, 5);
    
    //Get frame id Parameters
    std::string frame_id;
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    
    //Setup Publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("pc2", 1000);
    
    //Create Sweep Driver Object
    sweep::sweep device{serial_port.c_str()};
    
    //Send Rotation Speed
    device.set_motor_speed(rotation_speed);
    
    ROS_INFO("expected rotation frequency: %d (Hz)", rotation_speed);
    
    //initialize all other nodes
    SlamGMapping::startLiveSlam();
    
    
    //Start Scan
    device.start_scanning();
    
    while (ros::ok())
    {
        //Grab Full Scan
        const sweep::scan scan = device.get_scan();
        
        publish_scan(&scan_pub, &scan, frame_id);
        
        
        
        ros::spinOnce();
    }
    
    //Stop Scanning & Destroy Driver
    device.stop_scanning();
}

catch (const sweep::device_error& e) {
    ROS_ERROR_STREAM("Error: " << e.what() << std::endl);
}



