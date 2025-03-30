// #include <octomap_visibility/octomap_visibility.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Line;

struct ElectricalComponent { std::string id; std::string ipto_id; std::string description; Eigen::Vector3d cornerMin; Eigen::Vector3d cornerMax; 
ElectricalComponent(std::string id, std::string ipto_id, std::string description, Eigen::Vector3d cornerMin, Eigen::Vector3d cornerMax) : id(id), ipto_id(ipto_id), description(description), cornerMin(cornerMin), cornerMax(cornerMax) {}; };


std::vector<ElectricalComponent> vecElectricalComponents;
ros::Publisher publisherCuboidBBoxLines;

void loadYAMLParameters(const std::string& yaml_filename, std::vector<ElectricalComponent>& vecElectricalComponents) {
  	
	std::cout << std::endl;
	std::cout << "<-------------------------------------------------------------------------->" << std::endl;
	std::cout << "<----------- load YAML file (with source points for Raycasting) ----------->" << std::endl;
	std::cout << "<-------------------------------------------------------------------------->" << std::endl;
  
	YAML::Node yamlNode = YAML::LoadFile(yaml_filename);
	
	std::cout << "YAML file " << yaml_filename << " loaded successfully with a size of " << yamlNode.size() << ". The following BBoxes (Electrical Components) were loaded:" << std::endl;

	vecElectricalComponents.clear();
	for ( YAML::const_iterator it = yamlNode.begin(); it != yamlNode.end(); ++it ) {
		
		YAML::Node component_node = *it;  		

		YAML::Node component_id_node = component_node["id"]; // id, ipto_id, description, pointMin, pointMax are a "mapping" so [] is used
		YAML::Node component_ipto_id_node = component_node["ipto_id"];
		YAML::Node component_description_node = component_node["description"];
		YAML::Node component_pointA_node = component_node["pointA"];
		YAML::Node component_pointB_node = component_node["pointB"];
		
		const std::string component_id = component_id_node.as<std::string>();
		const std::string component_ipto_id = component_ipto_id_node.as<std::string>();
		const std::string component_description = component_description_node.as<std::string>();
		const double component_pointA_x = component_pointA_node["x"].as<double>(); // x, y, z are a "mapping" so [] is used
	  	const double component_pointA_y = component_pointA_node["y"].as<double>();
		const double component_pointA_z = component_pointA_node["z"].as<double>();
		const double component_pointB_x = component_pointB_node["x"].as<double>();
		const double component_pointB_y = component_pointB_node["y"].as<double>();
		const double component_pointB_z = component_pointB_node["z"].as<double>();
		
		

		// Because corners can be entered in any order, determine "Low" and "High" corners
		const Eigen::Vector3d component_cornerMin = Eigen::Vector3d( std::min(component_pointA_x, component_pointB_x), std::min(component_pointA_y, component_pointB_y), std::min(component_pointA_z, component_pointB_z) );
		const Eigen::Vector3d component_cornerMax = Eigen::Vector3d( std::max(component_pointA_x, component_pointB_x), std::max(component_pointA_y, component_pointB_y), std::max(component_pointA_z, component_pointB_z) );
	  


		
		ElectricalComponent electrical_component(component_id, component_ipto_id, component_description, component_cornerMin, component_cornerMax);
		
		
		
		// Add to vector of ElectricalComponents (for Raycasting)
		vecElectricalComponents.push_back(electrical_component);
		std::cout << "- id \"" << component_id << "\" and ipto_id \"" << component_ipto_id << "\" and description \"" << component_description << "\" and coords (min): [" << component_cornerMin.x() << " " << component_cornerMin.y() << " " << component_cornerMin.z() << "] and coords (max): [" << component_cornerMax.x() << " " << component_cornerMax.y() << " " << component_cornerMax.z() << "]." << std::endl;
	}
	
	
}

const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCuboidLines(const Eigen::Vector3d &cornerMin, const Eigen::Vector3d &cornerMax) {

	// test p = octomap(); push p; p=p'; push(p) if it is the same

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vecOfCuboidLines;


	// Old comment, does not hold anymore: Because corners can be entered in any order, determine mins and maxs
	const double minX = cornerMin.x(); // std::min(cornerA.x(), cornerB.x());
	const double maxX = cornerMax.x(); // std::max(cornerA.x(), cornerB.x());
	const double minY = cornerMin.y(); // std::min(cornerA.y(), cornerB.y());
	const double maxY = cornerMax.y(); // std::max(cornerA.y(), cornerB.y());
	const double minZ = cornerMin.z(); // std::min(cornerA.z(), cornerB.z());
	const double maxZ = cornerMax.z(); // std::max(cornerA.z(), cornerB.z());


	// Push back all lines that consist the bottom face of the cuboid (always use the minimum coords possible for the start point of the line segment)
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, minY, minZ), Eigen::Vector3d(maxX, minY, minZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(maxX, minY, minZ), Eigen::Vector3d(maxX, maxY, minZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, maxY, minZ), Eigen::Vector3d(maxX, maxY, minZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, minY, minZ), Eigen::Vector3d(minX, maxY, minZ) ));

	// Do the same for the top face of the cuboid
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, minY, maxZ), Eigen::Vector3d(maxX, minY, maxZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(maxX, minY, maxZ), Eigen::Vector3d(maxX, maxY, maxZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, maxY, maxZ), Eigen::Vector3d(maxX, maxY, maxZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, minY, maxZ), Eigen::Vector3d(minX, maxY, maxZ) ));

	// Finally, do the same for the 4 vertical lines of the cuboid
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, minY, minZ), Eigen::Vector3d(minX, minY, maxZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(maxX, minY, minZ), Eigen::Vector3d(maxX, minY, maxZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(maxX, maxY, minZ), Eigen::Vector3d(maxX, maxY, maxZ) ));
	vecOfCuboidLines.push_back(Line( Eigen::Vector3d(minX, maxY, minZ), Eigen::Vector3d(minX, maxY, maxZ) ));


	return vecOfCuboidLines;

  }

void addLineToMarkerArray(const Line& line, visualization_msgs::MarkerArray& markerArray) {

    const float radius = 0.02;

	// We have constructed startPoint with lower coords than endPoint
	geometry_msgs::Point startPoint;
	startPoint.x = line.first.x();
	startPoint.y = line.first.y();
	startPoint.z = line.first.z();
	
	geometry_msgs::Point endPoint;
	endPoint.x = line.second.x();
	endPoint.y = line.second.y();
	endPoint.z = line.second.z();

	visualization_msgs::Marker edge;
	edge.header.frame_id = "map";
	edge.header.stamp = ros::Time::now();
	edge.ns = "bbox_edges";
	edge.id = markerArray.markers.size();
	edge.type = visualization_msgs::Marker::CYLINDER;
	edge.action = visualization_msgs::Marker::ADD;
	edge.color.r = 0.0;  // Green
	edge.color.g = 1.0;
	edge.color.b = 0.0;
	edge.color.a = 1.0;
	
	// Midpoint position
	edge.pose.position.x = (startPoint.x + endPoint.x) / 2;
	edge.pose.position.y = (startPoint.y + endPoint.y) / 2;
	edge.pose.position.z = (startPoint.z + endPoint.z) / 2;
	
	// Eigen rotation (Z-axis to edge direction)
	Eigen::Vector3d dir(endPoint.x - startPoint.x, endPoint.y - startPoint.y, endPoint.z - startPoint.z);
	edge.scale.x = radius * 2; // Diameter
	edge.scale.y = radius * 2; // Diameter
	edge.scale.z = dir.lpNorm<Eigen::Infinity>();  // Height = edge length
	dir.normalize();
	Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir);
	edge.pose.orientation.x = q.x();
	edge.pose.orientation.y = q.y();
	edge.pose.orientation.z = q.z();
	edge.pose.orientation.w = q.w();

	markerArray.markers.push_back(edge); // Add to array
}

void publishCuboidBBoxes(const ros::Publisher& publisherCuboidBBoxLines, const std::string& ipto_id, const unsigned int face_number) {

	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker clearMarker;
    clearMarker.action = visualization_msgs::Marker::DELETEALL;
    clearMarker.ns = "bbox_edges";  // Same namespace used in function addLineToMarkerArray()
    markerArray.markers.push_back(clearMarker);

	std::vector<Line> vecOfAllCuboidBBoxLines;
	for ( std::vector<ElectricalComponent>::const_iterator it = vecElectricalComponents.begin(); it != vecElectricalComponents.end(); ++it) {
		const ElectricalComponent electrical_component = *it;

		if ( ipto_id == "ALL" || ipto_id == electrical_component.ipto_id ) {
			const std::vector<Line> vecCuboidBBoxLines = getCuboidLines(electrical_component.cornerMin, electrical_component.cornerMax);
	
			for (std::vector<Line>::const_iterator line = vecCuboidBBoxLines.begin(); line != vecCuboidBBoxLines.end(); ++line) {
				addLineToMarkerArray(*line, markerArray);
			}
	
			std::cout << "Added 12 cylindrical edges in the MarkerArray." << std::endl;
		}
	}

	
	
	// ===== Publish ALL edges in ONE message =====
    publisherCuboidBBoxLines.publish(markerArray);
}

void splitString(const std::string& text, std::string& ipto_id, unsigned int face_number) {
	const char target = '_';

	// Find the first '_'
	size_t first_pos = text.find(target);
	if (first_pos == std::string::npos) {
		ipto_id = "";
		face_number = 0;
	} else {
		// Find the second '_' (start searching after the first one)
		size_t second_pos = text.find(target, first_pos + 1);
		if (second_pos == std::string::npos) {
			ipto_id = text;
			face_number = 0;
		} else {
			// Extract and print the substring before the second '_'
			ipto_id = text.substr(0, second_pos);
			
			const std::string remaining_text = text.substr(second_pos + 1);
			if ( remaining_text == "F1" || remaining_text == "F2" || remaining_text == "F3" || remaining_text == "F4" ) {
				face_number = remaining_text[1] - '0';
			} else
				face_number = 0;
		}
	}

	ROS_INFO("The requested ipto_id is: %s, face_number is: %d", ipto_id.c_str(), face_number);
	return;
}



void stringCallback(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("Received: %s", msg->data.c_str()); // bush_1b_F3
	
	std::string ipto_id;
	unsigned int face_number;
	splitString(msg->data, ipto_id, face_number);


    publishCuboidBBoxes(publisherCuboidBBoxLines, ipto_id, face_number);
}

//   void publishCuboidFaceSources(const std::vector<Eigen::Vector3d>& vecOfCuboidFaceSources) {

// 	cuboidFaceSources_publisher = pnh_.advertise<visualization_msgs::Marker>("cuboid_bbox_face_sources", 1, true);



// 	const unsigned long colorValue = 1641198; // blue examples: red=15600153, green=65280, blue=1641198
// 	/*if (!getParam("color", colorValue)) {
// 		ROS_INFO("VectorVisualization with name '%s' did not find a 'color' parameter. Using default.", name_.c_str());
// 	}*/
  
// 	Eigen::Vector3f colorVector;
//   	grid_map::colorValueToVector(colorValue, colorVector);

// 	std_msgs::ColorRGBA colorMsg;

//     colorMsg.r = colorVector.x();
//     colorMsg.g = colorVector.y();
//     colorMsg.b = colorVector.z();
//     colorMsg.a = 1;



// 	// Marker to be published.
//   	visualization_msgs::Marker marker;

// 	// Set marker info.
// 	marker.header.frame_id = m_gridMap->getFrameId();
// 	marker.header.stamp.fromNSec(m_gridMap->getTimestamp());

// 	marker.ns = "bbox_visualization";
// 	marker.id = 0;

// 	marker.type = visualization_msgs::Marker::CUBE_LIST;
// 	marker.action = visualization_msgs::Marker::ADD;
// 	marker.lifetime = ros::Duration();
	
// 	marker.scale.x = 0.1;
// 	marker.scale.y = 0.1;
// 	marker.scale.z = 0.1;




// 	// Clear points.
// 	marker.points.clear();
// 	marker.colors.clear();
// 	for (std::vector<Eigen::Vector3d>::const_iterator it = vecOfCuboidFaceSources.begin(); it != vecOfCuboidFaceSources.end(); ++it) {

// 		geometry_msgs::Point cuboidFaceSource;
// 		cuboidFaceSource.x = it->x();
// 		cuboidFaceSource.y = it->y();
// 		cuboidFaceSource.z = it->z();
// 		marker.points.push_back(cuboidFaceSource);
		
// 		marker.colors.push_back(colorMsg); // Each vertex needs a color.

// 	}

// 	cuboidFaceSources_publisher.publish(marker);



// }


    
    
    
    
    
    
    













int main(int argc, char **argv) {

	ros::init(argc, argv, "bbox_publisher");
	ros::NodeHandle nh, pnh("~");
	nh.setParam("frame_id", "map");
	
	ros::Subscriber sub = nh.subscribe("ipto_id_bbox", 1, stringCallback);
	
	const std::string fname_YAML = "/home/tetix/Dropbox/ENORASI_tmp/octomap_visibility_requirements/segmentation_2D_e=0.12_Z4-65.yaml";
	loadYAMLParameters(fname_YAML, vecElectricalComponents);
	
	publisherCuboidBBoxLines = nh.advertise<visualization_msgs::MarkerArray>("cuboid_bbox_lines", 1, true);

	//publishCuboidBBoxes(publisherCuboidBBoxLines, "ALL");


	std::cout << "done. Entering loop..." << std::endl << std::endl;
	ros::Rate r(1);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;



	// for (unsigned int i = 4; i <= 4; i++) {
			// 	const unsigned char face_type = i;
				
			// 	Eigen::Vector3d face_center;
			// 	std::vector<Eigen::Vector3d> vecOfCuboidFaceSources;
			// 	octVis.determineCuboidFaceSources(electrical_component.cornerMin, electrical_component.cornerMax, face_type, face_size, face_center, vecOfCuboidFaceSources);
			// 	octVis.publishCuboidFaceSources(vecOfCuboidFaceSources);
			// }


}

























