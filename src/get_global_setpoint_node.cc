/**
 * @file get_global_setpoint_node.cc
 * @brief Node to capture GPS coordinates and calculate relative ENU position
 * 
 * Functionality:
 * 1. Capture GPS coordinates (lat, lon, alt) when drone powers on
 * 2. Capture GPS coordinates when user presses a key (ENTER)
 * 3. Calculate relative position vector in ENU frame (meters)
 * 4. Save both coordinates and relative vector to XML file
 */

#include <ros/ros.h>
#include <ros/package.h>  
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

class GlobalSetpointRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    
    // GPS data storage
    sensor_msgs::NavSatFix initial_gps_;
    sensor_msgs::NavSatFix target_gps_;
    bool initial_captured_ = false;
    bool target_captured_ = false;
    bool gps_received_ = false;
    
    // Output file path
    std::string output_file_;
    
    // Earth parameters
    const double EARTH_RADIUS = 6378137.0; // WGS84 equatorial radius in meters
    
public:
    GlobalSetpointRecorder() : nh_("~") {
        // Get output file path from parameter, default to resc-pilot/launch folder
        std::string package_path = ros::package::getPath("px4_utils_land");
        std::string relative_path = "launch/global_setpoints.yaml";
        std::string default_path = package_path + "/" + relative_path;
        nh_.param<std::string>("output_file", output_file_, default_path);

        // Read target GPS coordinates from parameters
        double target_lat, target_lon, target_alt;
        if (nh_.getParam("target_latitude", target_lat) && 
            nh_.getParam("target_longitude", target_lon) && 
            nh_.getParam("target_altitude", target_alt)) {
            
            target_gps_.latitude = target_lat;
            target_gps_.longitude = target_lon;
            target_gps_.altitude = target_alt;
            target_gps_.status.status = 0; // Set valid status
            target_captured_ = true;
            
            ROS_INFO("Target GPS loaded from parameters:");
            ROS_INFO("  Latitude:  %.8f", target_gps_.latitude);
            ROS_INFO("  Longitude: %.8f", target_gps_.longitude);
            ROS_INFO("  Altitude:  %.3f m", target_gps_.altitude);
        }

        // Subscribe to GPS topic
        gps_sub_ = nh_.subscribe("/mavros/global_position/global", 10, 
                                 &GlobalSetpointRecorder::gpsCallback, this);
        
        ROS_INFO("Global Setpoint Recorder Node Started");
        ROS_INFO("Output file: %s", output_file_.c_str());
        ROS_INFO("Waiting for GPS data...");
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!gps_received_) {
            ROS_INFO("GPS data received!");
            gps_received_ = true;
        }
        
        // Capture initial GPS coordinates on first valid GPS message
        if (!initial_captured_ && msg->status.status >= 0) {
            initial_gps_ = *msg;
            initial_captured_ = true;
            ROS_INFO("Initial GPS captured:");
            ROS_INFO("  Latitude:  %.8f", initial_gps_.latitude);
            ROS_INFO("  Longitude: %.8f", initial_gps_.longitude);
            ROS_INFO("  Altitude:  %.3f m", initial_gps_.altitude);
            ROS_INFO("\nPress ENTER to capture target GPS coordinates...");
        }
        
        // Update current GPS for manual capture
        if (initial_captured_ && !target_captured_) {
            target_gps_ = *msg;
        }
    }
    
    /**
     * @brief Convert GPS coordinates to ENU relative position
     * @param ref Reference GPS coordinate (initial position)
     * @param target Target GPS coordinate
     * @return Vector3 with ENU coordinates (East, North, Up) in meters
     */
    geometry_msgs::Vector3 gpsToENU(const sensor_msgs::NavSatFix& ref, 
                                     const sensor_msgs::NavSatFix& target) {
        geometry_msgs::Vector3 enu;
        
        // Convert degrees to radians
        double lat1 = ref.latitude * M_PI / 180.0;
        double lon1 = ref.longitude * M_PI / 180.0;
        double lat2 = target.latitude * M_PI / 180.0;
        double lon2 = target.longitude * M_PI / 180.0;
        
        // Calculate differences
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        
        // Calculate ENU coordinates
        // East: delta_longitude * R * cos(latitude)
        enu.x = dlon * EARTH_RADIUS * std::cos((lat1 + lat2) / 2.0);
        
        // North: delta_latitude * R
        enu.y = dlat * EARTH_RADIUS;
        
        // Up: delta_altitude
        enu.z = target.altitude - ref.altitude + 2.0; // Adding 2.0 to account for altitude offset
        
        return enu;
    }
    
    /**
     * @brief Calculate Euclidean distance between two GPS points
     */
    double calculateDistance(const sensor_msgs::NavSatFix& ref, 
                            const sensor_msgs::NavSatFix& target) {
        geometry_msgs::Vector3 enu = gpsToENU(ref, target);
        return std::sqrt(enu.x * enu.x + enu.y * enu.y + enu.z * enu.z);
    }
    
    /**
     * @brief Save GPS coordinates and ENU vector to YAML file
     */
    void saveToYAML() {
        if (!initial_captured_ || !target_captured_) {
            ROS_ERROR("Cannot save: Both GPS coordinates must be captured first!");
            return;
        }
        
        // Calculate ENU relative position
        geometry_msgs::Vector3 enu_vector = gpsToENU(initial_gps_, target_gps_);
        double distance = calculateDistance(initial_gps_, target_gps_);
        
        // Open file for writing
        std::ofstream file(output_file_);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", output_file_.c_str());
            return;
        }
        
        // Write YAML content
        file << "# GPS Coordinates captured by get_global_setpoint node\n";
        file << "# Generated at: " << ros::Time::now() << "\n\n";
        
        file << "# Initial GPS coordinate (power-on position)\n";
        file << "initial:\n";
        file << "  latitude: " << std::fixed << std::setprecision(8) 
             << initial_gps_.latitude << "\n";
        file << "  longitude: " << std::fixed << std::setprecision(8) 
             << initial_gps_.longitude << "\n";
        file << "  altitude: " << std::fixed << std::setprecision(3) 
             << initial_gps_.altitude << "\n\n";
        
        file << "# Target GPS coordinate (manually captured)\n";
        file << "target:\n";
        file << "  latitude: " << std::fixed << std::setprecision(8) 
             << target_gps_.latitude << "\n";
        file << "  longitude: " << std::fixed << std::setprecision(8) 
             << target_gps_.longitude << "\n";
        file << "  altitude: " << std::fixed << std::setprecision(3) 
             << target_gps_.altitude << "\n\n";
        
        file << "# Relative position vector in ENU frame (meters)\n";
        file << "# Target position relative to Initial position\n";
        file << "enu_relative:\n";
        file << "  east: " << std::fixed << std::setprecision(3) 
             << enu_vector.x << "\n";
        file << "  north: " << std::fixed << std::setprecision(3) 
             << enu_vector.y << "\n";
        file << "  up: " << std::fixed << std::setprecision(3) 
             << enu_vector.z << "\n";
        file << "  distance: " << std::fixed << std::setprecision(3) 
             << distance << "\n";
        
        file.close();
        
        ROS_INFO("\n=== Data saved to: %s ===", output_file_.c_str());
        ROS_INFO("Initial GPS: (%.8f, %.8f, %.3f)", 
                 initial_gps_.latitude, initial_gps_.longitude, initial_gps_.altitude);
        ROS_INFO("Target GPS:  (%.8f, %.8f, %.3f)", 
                 target_gps_.latitude, target_gps_.longitude, target_gps_.altitude);
        ROS_INFO("ENU Relative Position (m): East=%.3f, North=%.3f, Up=%.3f", 
                 enu_vector.x, enu_vector.y, enu_vector.z);
        ROS_INFO("Total Distance: %.3f meters", distance);
    }
    
    /**
     * @brief Main loop to handle keyboard input
     */
    void run() {
        // Wait for initial GPS capture
        ros::Rate rate(10);
        while (ros::ok() && !initial_captured_) {
            ros::spinOnce();
            rate.sleep();
        }
        
        if (!ros::ok()) {
            return;
        }
        
        // If target GPS is already loaded from parameters, process immediately
        if (target_captured_) {
            ROS_INFO("Using target GPS from parameters - processing immediately...");
            
            // Calculate and display relative position
            geometry_msgs::Vector3 enu = gpsToENU(initial_gps_, target_gps_);
            double dist = calculateDistance(initial_gps_, target_gps_);
            
            ROS_INFO("\nRelative Position (ENU):");
            ROS_INFO("  East:  %.3f m", enu.x);
            ROS_INFO("  North: %.3f m", enu.y);
            ROS_INFO("  Up:    %.3f m", enu.z);
            ROS_INFO("  Distance: %.3f m", dist);
            
            // Save to file
            saveToYAML();
            
            ROS_INFO("Processing complete. Node shutting down...");
            return;
        }
        

    }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_global_setpoint_node");
    
    GlobalSetpointRecorder recorder;
    recorder.run();
    
    return 0;
}
