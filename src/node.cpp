 #include"Inuitive.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "m32_ros");
    ros::NodeHandle n;

    Inuitive dev(n);
    
    ros::spin();

    return dev.Stop();
}

