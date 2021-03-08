//
// Created by Kanishka Ganguly on 7/23/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//


#pragma once

#include "../include/imports.hpp"

/**
 *  @class Box robot_utils.hpp "include/robot_utils.hpp"
 *  @brief Helper class to store collision bounding boxes, based on link state of robot.
 *  Makes it easy to store all seven links of robot as "boxes" and keep track of voxels the box passes through.
 *  Also used to store obstacle models.
 */
class Box {
private:
  /** @brief Name of the link represented by this bounding box */
  std::string name;
  /** @brief Store canonical locations of each point, for left and right faces */
  std::array<Eigen::Vector4f, 4> left_face_canonical;
  std::array<Eigen::Vector4f, 4> right_face_canonical;
  /** @brief The nodes in voxel grid occupied by this bounding box */
  std::vector<Eigen::Vector3i> nodes_hit;

public:
  /** @brief Set name */
  inline void SetName(const std::string &name) { this->name = name; }

  /** @brief Set nodes this box hits in voxel grid */
  inline void SetNodesHit(const std::vector<Eigen::Vector3i> &nodes_hit) { this->nodes_hit = nodes_hit; }

  /** @brief Set face canonical point coordinates */
  inline void SetCanonical(const std::string &face, const std::array<Eigen::Vector4f, 4> &values) {
	  if (face.compare("left") == 0) {
		  this->left_face_canonical = values;
	  } else if (face.compare("right") == 0) {
		  this->right_face_canonical = values;
	  }
  }

  /** @brief Get name */
  inline std::string GetName() const { return this->name; }

  /** @brief Get nodes this box hits in voxel grid */
  inline std::vector<Eigen::Vector3i> GetNodesHit() { return this->nodes_hit; }

  /** @brief Get face canonical point coordinates */
  inline std::array<Eigen::Vector4f, 4> GetCanonical(const std::string &face) {
	  if (face.compare("left") == 0) {
		  return this->left_face_canonical;
	  } else if (face.compare("right") == 0) {
		  return this->right_face_canonical;
	  }
  }

  /** @brief Equality operator */
  bool operator==(const Box &b) const {
	  return this->GetName().compare(b.GetName()) == 0;
  }

  /** @overload */
  bool operator==(const std::string &b) const {
	  return this->GetName().compare(b) == 0;
  }

  /** @brief Less than operator */
  bool operator<(const Box &b) const {
	  return this->GetName() < b.GetName();
  }

  /** @overload */
  bool operator<(const std::string &b) const {
	  return this->GetName() < b;
  }

  /** @brief Greater than operator */
  bool operator>(const Box &b) const {
	  return this->GetName() > b.GetName();
  }

  /** @overload */
  bool operator>(const std::string &b) const {
	  return this->GetName() > b;
  }

  /** @brief Stream output operator */
  std::ostream &operator<<(std::ostream &os) const {
	  os << this->GetName();
	  return os;
  }
};

/**
 *  @class RobotUtils robot_utils.hpp "include/robot_utils.hpp"
 *  @brief Utility class for performing computation on robot state obtained from VoxelGridPlugin
 *	Contains helper functions for pose conversions, string to links conversions, and checking for
 *	state changes in robot.
 */
class RobotUtils {
  typedef std::pair<std::string, std::pair<int, float>> mapping_var;
private:
  /** @brief Loaded XML document from config file */
  tinyxml2::XMLDocument doc;

  /**
   * @brief Mapping from world axis convention to link axis convention
   * Loaded once from XML file, queried as needed
   */
  std::array<mapping_var, 21> world_link_mapping;

public:
  /**
   * @brief Constructor for RobotUtils
   */
  RobotUtils() {
	  std::string pkg_path = ros::package::getPath("voxel_grid_plugin");
	  std::string xml_path = pkg_path + "/config/link_mapping.xml";
	  this->doc.LoadFile(xml_path.c_str());
	  this->ParseMapping();
  }

  /**
   * @brief Construct mapping from XML file, stored statically for future lookups
   */
  void ParseMapping() {
	  int iter = 0;
	  // Get root element
	  tinyxml2::XMLElement *root = this->doc.FirstChildElement("links");
	  // Loop through all children
	  for (tinyxml2::XMLElement *e = root->FirstChildElement("link"); e != NULL; e = e->NextSiblingElement("link")) {
		  // Get name attribute
		  const char *link_name_ch = e->Attribute("name");
		  std::string link_name(link_name_ch);
		  // Fetch all three child data elements (mappings)
		  tinyxml2::XMLElement *map_0 = e->FirstChildElement();
		  tinyxml2::XMLElement *map_1 = map_0->NextSiblingElement();
		  tinyxml2::XMLElement *map_2 = map_1->NextSiblingElement();

		  float val = -100;
		  // Fetch float value of element
		  map_0->QueryFloatText(&val);
		  // Add data to array
		  world_link_mapping[iter] = std::make_pair(link_name, std::make_pair(0, val == 10 ? -0.0f : val));
		  // Increment array insertion index
		  iter += 1;

		  map_1->QueryFloatText(&val);
		  world_link_mapping[iter] = std::make_pair(link_name, std::make_pair(1, val == 10 ? -0.0f : val));
		  iter += 1;

		  map_2->QueryFloatText(&val);
		  world_link_mapping[iter] = std::make_pair(link_name, std::make_pair(2, val == 10 ? -0.0f : val));
		  iter += 1;
	  }
  }

  /**
   * @brief	Used to update XML elements, primarily to be used for
   * 			updating initial pose data in Gazebo model SDF before adding to world.
   * @param xml_path The path to the SDF file
   * @param data The data used for updating
   * @return The XML as string after updating
   */
  std::string SetInitialPoseFromFile(std::string xml_path, std::string data) {
	  tinyxml2::XMLDocument xml_document; // Construct XML document
	  // Load XML from file
	  xml_document.LoadFile(xml_path.c_str());
	  tinyxml2::XMLElement *sdf_element = xml_document.FirstChildElement("sdf"); // Root sdf element
	  tinyxml2::XMLElement *model_element = sdf_element->FirstChildElement("model"); // Model element under root sdf
	  tinyxml2::XMLElement *pose_element = model_element->FirstChildElement("pose"); // Pose element under model
	  if (pose_element) {
		  pose_element->SetText(data.c_str()); // Do the update
	  }
	  // Convert updated XML back to string
	  tinyxml2::XMLPrinter printer;
	  xml_document.Print(&printer);
	  return std::string(printer.CStr());
  }

  /**
   * @brief	Used to update XML elements, primarily to be used for
   * 			updating initial pose data in Gazebo model SDF before adding to world.
   * @param xml_path The path to the SDF file
   * @param data The data used for updating
   * @return The XML as string after updating
   */
  std::string SetInitialPoseFromString(std::string xml_string, std::string data) {
	  tinyxml2::XMLDocument xml_document; // Construct XML document
	  // Load XML from file
	  xml_document.Parse(xml_string.c_str());
	  tinyxml2::XMLElement *sdf_element = xml_document.FirstChildElement("sdf"); // Root sdf element
	  tinyxml2::XMLElement *model_element = sdf_element->FirstChildElement("model"); // Model element under root sdf
	  tinyxml2::XMLElement *pose_element = model_element->FirstChildElement("pose"); // Pose element under model
	  if (pose_element) {
		  pose_element->SetText(data.c_str()); // Do the update
	  }
	  // Convert updated XML back to string
	  tinyxml2::XMLPrinter printer;
	  xml_document.Print(&printer);
	  return std::string(printer.CStr());
  }

  /**
   * @brief	Used to update XML elements, primarily to be used for
   * 			updating initial pose data in Gazebo model SDF before adding to world.
   * @param xml_path The path to the SDF file
   * @param data The data used for updating
   * @return The XML as string after updating
   */
  std::string SetModelNameFromFile(std::string xml_path, std::string data) {
	  tinyxml2::XMLDocument xml_document; // Construct XML document
	  // Load XML from file
	  xml_document.LoadFile(xml_path.c_str());
	  tinyxml2::XMLElement *sdf_element = xml_document.FirstChildElement("sdf"); // Root sdf element
	  tinyxml2::XMLElement *model_element = sdf_element->FirstChildElement("model"); // Model element under root sdf
	  if (model_element) {
		  model_element->DeleteAttribute("name"); // Delete attribute
		  model_element->SetAttribute("name", data.c_str()); // Do the update
	  }
	  // Convert updated XML back to string
	  tinyxml2::XMLPrinter printer;
	  xml_document.Print(&printer);
	  return std::string(printer.CStr());
  }

  /**
   * @brief	Used to update XML elements, primarily to be used for
   * 			updating initial pose data in Gazebo model SDF before adding to world.
   * @param xml_path The path to the SDF file
   * @param data The data used for updating
   * @return The XML as string after updating
   */
  std::string SetModelNameFromString(std::string xml_string, std::string data) {
	  tinyxml2::XMLDocument xml_document; // Construct XML document
	  // Load XML from file
	  xml_document.Parse(xml_string.c_str());
	  tinyxml2::XMLElement *sdf_element = xml_document.FirstChildElement("sdf"); // Root sdf element
	  tinyxml2::XMLElement *model_element = sdf_element->FirstChildElement("model"); // Model element under root sdf
	  if (model_element) {
		  model_element->DeleteAttribute("name"); // Delete attribute
		  model_element->SetAttribute("name", data.c_str()); // Do the update
	  }
	  // Convert updated XML back to string
	  tinyxml2::XMLPrinter printer;
	  xml_document.Print(&printer);
	  return std::string(printer.CStr());
  }

  /**
   * @brief	Used to update XML elements, primarily to be used for
   * 			updating sensor and collision name in Gazebo model SDF before adding to world.
* 				These need to be unique for collision checking.
   * @param xml_path The path to the SDF file
   * @param data The data used for updating
   * @return The XML as string after updating
   */
  std::string SetSensorDataFromString(std::string xml_string, std::string model_name) {
	  tinyxml2::XMLDocument xml_document; // Construct XML document
	  // Load XML from file
	  xml_document.Parse(xml_string.c_str());
	  // SDF --> Model --> Link --> Sensor --> contact --> collision + topic
	  //						  --> Collision
	  tinyxml2::XMLElement *sdf_element = xml_document.FirstChildElement("sdf"); // Root sdf element
	  tinyxml2::XMLElement *model_element = sdf_element->FirstChildElement("model"); // Child model element
	  tinyxml2::XMLElement *link_element = model_element->FirstChildElement("link"); // Child link element
	  tinyxml2::XMLElement *sensor_element = link_element->FirstChildElement("sensor"); // Child sensor element
	  tinyxml2::XMLElement *collision_element = link_element->FirstChildElement("collision"); // Child collision element
	  // Collision element needs to be the same in both top level and under sensor element
	  // Otherwise Gazebo does not recognize it
	  if (collision_element) {
		  collision_element->DeleteAttribute("name"); // Delete attribute
		  std::string collision_name = model_name + "_collision";
		  collision_element->SetAttribute("name", collision_name.c_str()); // Do the update
	  } else {
		  spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Did not find COLLISION element in SDF");
	  }
	  if (sensor_element) {
		  sensor_element->DeleteAttribute("name"); // Delete attribute
		  std::string sensor_name = model_name + "_sensor";
		  sensor_element->SetAttribute("name", sensor_name.c_str()); // Do the update

		  tinyxml2::XMLElement *contact_element = sensor_element->FirstChildElement("contact"); // Contact element under sensor
		  if (contact_element) {
			  tinyxml2::XMLElement *collision_child_element = contact_element->FirstChildElement("collision"); // Contact element under sensor
			  tinyxml2::XMLElement *topic_element = contact_element->FirstChildElement("topic"); // Contact element under sensor
			  if (collision_child_element) {
				  std::string collision_child_name = model_name + "_collision";
				  collision_child_element->SetText(collision_child_name.c_str()); // Do the update
			  } else {
				  spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Did not find COLLISION child element in SDF");
			  }
			  if (topic_element) {
				  std::string topic_name = "/" + model_name + "_collision_topic";
				  topic_element->SetText(topic_name.c_str()); // Do the update
			  } else {
				  spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Did not find TOPIC element in SDF");
			  }
		  } else {
			  spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Did not find CONTACT element in SDF");
		  }
	  } else {
		  spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Did not find SENSOR element in SDF");
	  }
	  // Convert updated XML back to string
	  tinyxml2::XMLPrinter printer;
	  xml_document.Print(&printer);
	  return std::string(printer.CStr());
  }

  /**
   * @brief Reads a model SDF file to string
   * @param file_path The path to the SDF file
   * @return The string containing the SDF data
   */
  std::string ReadSDF(const std::string &file_path) {
	  std::ifstream sdf_in;
	  sdf_in.open(file_path);
	  std::stringstream sdf_stream;
	  sdf_stream << sdf_in.rdbuf();
	  return sdf_stream.str();
  }

  /**
   * @brief Get mapping of axis for given link
   * @param link_name Name of link whose mapping needs to be looked up
   * @param axis The mapped axis
   * @return The mapped axis as float, used to encode direction as sign
   */
  inline float GetMapping(std::string link_name, int axis) {
	  for (const auto &map : this->world_link_mapping) {
		  if (link_name.compare(map.first) == 0) {
			  if (map.second.first == axis) {
				  return map.second.second;
			  }
		  }
	  }
  }

  /**
   * @brief Get shared pointer to box that has specified name
	* @param name - Name of box to return
   * @return Shared pointer to box
   */
  static std::shared_ptr<Box> GetBoxStateByName(const std::string &name, const std::vector<std::shared_ptr<Box>> &box_state) {
	  for (const auto &b : box_state) {
		  if (name.compare(b->GetName()) == 0) {
			  return b;
		  }
	  }
	  return nullptr;
  }

  /**
   * @brief Used to compare two sets, in order to check which one has more elements than the other.
   * This will be used to synchronize the two sets.
   * @tparam T Any template parameter that std::set can accept
   * @param set1 The first set
   * @param set2 The second set
   * @param diff The elements that are in set1 but not in set 2
   */
  template<typename T>
  static void CompareSets(const std::set<T> &set1, const std::set<T> &set2, std::set<T> &diff) {
	  diff.clear();
	  std::set_difference(set1.begin(), set1.end(), set2.begin(), set2.end(),
						  std::inserter(diff, diff.begin()));
  }

  /**
   * @brief Used to compare two sets, in order to check which one has more elements than the other.
   * @param set1 First set to check
   * @param set2 Second set to check
   * @param diff Returns the elements that are in set1 but not in set2
   */
  template<typename T>
  static void CompareSets(const std::vector<std::shared_ptr<Box>> &set1,
						  const T &set2,
						  std::vector<std::string> &diff) {
	  diff.clear();
	  for (const auto &i : set1) {
		  auto it = std::find(set2.begin(), set2.end(), i->GetName());
		  if (it == std::end(set2)) {
			  diff.push_back(i->GetName());
		  }
	  }
  }

  /** @overload */
  template<typename T>
  static void CompareSets(const T &set1,
						  const std::vector<std::shared_ptr<Box>> &set2,
						  std::vector<std::string> &diff) {
	  diff.clear();
	  for (const std::string &i : set1) {
		  auto it = std::find_if(set2.begin(), set2.end(), [&](std::shared_ptr<Box> const &b) { return *b == i; });
		  if (it == std::end(set2)) {
			  diff.push_back(i);
		  }
	  }
  }

  /**
   * @brief This updates a set based on the elements in another set. Both insertion and removal are supported.
   * @param local_list The set to update
   * @param diff The set from which to get elements
   * @param add Boolean switching between insertion or deletion
   */
  template<typename T>
  static void UpdateList(std::set<T> &local_list, const std::set<T> &diff, const bool &add = true) {
	  for (auto i : diff) {
		  if (add) {
			  local_list.insert(i);
		  } else {
			  local_list.erase(i);
		  }
	  }
  }

  /** @overload */
  static void UpdateList(std::vector<std::shared_ptr<Box>> &box_list, const std::vector<std::string> &diff, const bool &add = true) {
	  for (auto i:diff) {
		  if (add) {
			  std::shared_ptr<Box> box_ptr = std::make_shared<Box>();
			  box_ptr->SetName(i);
			  box_list.push_back(box_ptr);
		  } else {
			  auto it = std::find_if(box_list.begin(), box_list.end(), [&](std::shared_ptr<Box> const &b) { return *b == i; });
			  if (it != box_list.end()) {
				  box_list.erase(it);
			  }
		  }
	  }
  }

  /**
   * @brief Given a Gazebo link bounding box, generates canonical positions (at time=0) for left and right faces
   * of the bounding box. This is used to transform the faces before raycasting based upon rotations obtained from Gazebo.
   * This function also remaps the axis conventions based on hardcoded values from a configuration file.
   * @param left_face - Array of four Eigen vectors to hold the four points of left face
   * @param right_face - Array of four Eigen vectors to hold the four points of right face
   */
  void GetCanonicalPositionsWithAxisMapping(const std::string &link_name,
											const gazebo::math::Box &cbox,
											std::array<Eigen::Vector4f, 4> &left_face,
											std::array<Eigen::Vector4f, 4> &right_face) {
	  // Make vector of (axis, dimension) pairs
	  std::vector<std::pair<int, double>> box_size = {
		  std::make_pair(0, cbox.GetXLength()),
		  std::make_pair(1, cbox.GetYLength()),
		  std::make_pair(2, cbox.GetZLength())
	  };
	  // Custom comparator function for sorting axis by dimension
	  auto sortLongestAxis = [](const std::pair<int, double> &l,
								const std::pair<int, double> &r) -> bool {
		return l.second >= r.second;
	  };
	  std::sort(box_size.begin(), box_size.end(), sortLongestAxis);
	  std::pair<int, double> offset = box_size[0];        // Offset from box center to face center
	  std::pair<int, double> face_offset_w = box_size[1]; // Offset from face center along width
	  std::pair<int, double> face_offset_h = box_size[2]; // Offset from face center along height

	  // Map world axes to corresponding link axes, based on lookup table
	  float axis_0 = GetMapping(link_name, 0); // Get axis mapping from link name
	  float axis_1 = GetMapping(link_name, 1); // Get axis mapping from link name
	  float axis_2 = GetMapping(link_name, 2); // Get axis mapping from link name

	  offset.first = static_cast<int>(std::abs(axis_0)); // Replace the axis with mapped value
	  offset.second = std::signbit(axis_0) ? -offset.second : offset.second; // Replace the sign, if negative

	  face_offset_w.first = static_cast<int>(std::abs(axis_1)); // Replace the axis with mapped value
	  face_offset_w.second = std::signbit(axis_1) ? -face_offset_w.second : face_offset_w.second; // Replace the sign, if negative

	  face_offset_h.first = static_cast<int>(std::abs(axis_2)); // Replace the axis with mapped value
	  face_offset_h.second = std::signbit(axis_2) ? -face_offset_h.second : face_offset_h.second; // Replace the sign, if negative

	  //======================== Get left face corners for canonical positions =========================//
	  // Define four corner of face in clockwise order
	  Eigen::Vector4f left_face_tl = Eigen::Vector4f::Ones(),
		  left_face_tr = Eigen::Vector4f::Ones(),
		  left_face_br = Eigen::Vector4f::Ones(),
		  left_face_bl = Eigen::Vector4f::Ones();
	  // Offset of center of box along longest axis, this gets you to center of the face
	  // Offset of center of face to get four corners
	  // Top left
	  left_face_tl(offset.first) = -offset.second / 2;
	  left_face_tl(face_offset_w.first) = face_offset_w.second / 2;
	  left_face_tl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Top right
	  left_face_tr(offset.first) = -offset.second / 2;
	  left_face_tr(face_offset_w.first) = face_offset_w.second / 2;
	  left_face_tr(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom right
	  left_face_br(offset.first) = -offset.second / 2;
	  left_face_br(face_offset_w.first) = -face_offset_w.second / 2;
	  left_face_br(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom left
	  left_face_bl(offset.first) = -offset.second / 2;
	  left_face_bl(face_offset_w.first) = -face_offset_w.second / 2;
	  left_face_bl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Set left face canonical points
	  left_face = {left_face_tl, left_face_tr, left_face_br, left_face_bl};
	  //======================== Get right face corners for canonical positions ========================//
	  // Define four corner of face in clockwise order
	  Eigen::Vector4f right_face_tl = Eigen::Vector4f::Ones(),
		  right_face_tr = Eigen::Vector4f::Ones(),
		  right_face_br = Eigen::Vector4f::Ones(),
		  right_face_bl = Eigen::Vector4f::Ones();

	  // Offset of center of box along longest axis, this gets you to center of the face
	  // Offset of center of face to get four corners
	  // Top left
	  right_face_tl(offset.first) = offset.second / 2;
	  right_face_tl(face_offset_w.first) = face_offset_w.second / 2;
	  right_face_tl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Top right
	  right_face_tr(offset.first) = offset.second / 2;
	  right_face_tr(face_offset_w.first) = face_offset_w.second / 2;
	  right_face_tr(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom right
	  right_face_br(offset.first) = offset.second / 2;
	  right_face_br(face_offset_w.first) = -face_offset_w.second / 2;
	  right_face_br(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom left
	  right_face_bl(offset.first) = offset.second / 2;
	  right_face_bl(face_offset_w.first) = -face_offset_w.second / 2;
	  right_face_bl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Set right face canonical points
	  right_face = {right_face_tl, right_face_tr, right_face_br, right_face_bl};
  };

  /**
   * @brief Given a Gazebo link bounding box, generates canonical positions (at time=0) for left and right faces
   * of the bounding box. This is used to transform the faces before raycasting based upon rotations obtained from Gazebo.
   * @param left_face - Array of four Eigen vectors to hold the four points of left face
   * @param right_face - Array of four Eigen vectors to hold the four points of right face
   */
  void GetCanonicalPositionsNoAxisMapping(const std::string &link_name,
										  const gazebo::math::Box &cbox,
										  std::array<Eigen::Vector4f, 4> &left_face,
										  std::array<Eigen::Vector4f, 4> &right_face) {
	  // Make vector of (axis, dimension) pairs
	  std::vector<std::pair<int, double>> box_size = {
		  std::make_pair(0, cbox.GetXLength()),
		  std::make_pair(1, cbox.GetYLength()),
		  std::make_pair(2, cbox.GetZLength())
	  };
	  // Custom comparator function for sorting axis by dimension
	  auto sortLongestAxis = [](const std::pair<int, double> &l,
								const std::pair<int, double> &r) -> bool {
		return l.second >= r.second;
	  };
	  std::sort(box_size.begin(), box_size.end(), sortLongestAxis);
	  std::pair<int, double> offset = box_size[0];        // Offset from box center to face center
	  std::pair<int, double> face_offset_w = box_size[1]; // Offset from face center along width
	  std::pair<int, double> face_offset_h = box_size[2]; // Offset from face center along height

	  //======================== Get left face corners for canonical positions =========================//
	  // Define four corner of face in clockwise order
	  Eigen::Vector4f left_face_tl = Eigen::Vector4f::Ones(),
		  left_face_tr = Eigen::Vector4f::Ones(),
		  left_face_br = Eigen::Vector4f::Ones(),
		  left_face_bl = Eigen::Vector4f::Ones();
	  // Offset of center of box along longest axis, this gets you to center of the face
	  // Offset of center of face to get four corners
	  // Top left
	  left_face_tl(offset.first) = -offset.second / 2;
	  left_face_tl(face_offset_w.first) = face_offset_w.second / 2;
	  left_face_tl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Top right
	  left_face_tr(offset.first) = -offset.second / 2;
	  left_face_tr(face_offset_w.first) = face_offset_w.second / 2;
	  left_face_tr(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom right
	  left_face_br(offset.first) = -offset.second / 2;
	  left_face_br(face_offset_w.first) = -face_offset_w.second / 2;
	  left_face_br(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom left
	  left_face_bl(offset.first) = -offset.second / 2;
	  left_face_bl(face_offset_w.first) = -face_offset_w.second / 2;
	  left_face_bl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Set left face canonical points
	  left_face = {left_face_tl, left_face_tr, left_face_br, left_face_bl};
	  //======================== Get right face corners for canonical positions ========================//
	  // Define four corner of face in clockwise order
	  Eigen::Vector4f right_face_tl = Eigen::Vector4f::Ones(),
		  right_face_tr = Eigen::Vector4f::Ones(),
		  right_face_br = Eigen::Vector4f::Ones(),
		  right_face_bl = Eigen::Vector4f::Ones();

	  // Offset of center of box along longest axis, this gets you to center of the face
	  // Offset of center of face to get four corners
	  // Top left
	  right_face_tl(offset.first) = offset.second / 2;
	  right_face_tl(face_offset_w.first) = face_offset_w.second / 2;
	  right_face_tl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Top right
	  right_face_tr(offset.first) = offset.second / 2;
	  right_face_tr(face_offset_w.first) = face_offset_w.second / 2;
	  right_face_tr(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom right
	  right_face_br(offset.first) = offset.second / 2;
	  right_face_br(face_offset_w.first) = -face_offset_w.second / 2;
	  right_face_br(face_offset_h.first) = face_offset_h.second / 2;
	  // Bottom left
	  right_face_bl(offset.first) = offset.second / 2;
	  right_face_bl(face_offset_w.first) = -face_offset_w.second / 2;
	  right_face_bl(face_offset_h.first) = -face_offset_h.second / 2;
	  // Set right face canonical points
	  right_face = {right_face_tl, right_face_tr, right_face_br, right_face_bl};
  };

  /**
   * @brief Transforms the canonical positions of the four corners of each box face to their world coordinates,
   * based on current pose from Gazebo.
   * @param rotation The current rotation of the link/model from Gazebo
   * @param position The current position of the link/model from Gazebo
   * @param transformed_left_points The output array for storing the left face points
   * @param transformed_right_points The output array for storing the right face points
   */
  void TransformCanonicalToWorld(const gazebo::math::Quaternion &rotation,
								 const gazebo::math::Vector3 &position,
								 const std::array<Eigen::Vector4f, 4> &left_canonical,
								 const std::array<Eigen::Vector4f, 4> &right_canonical,
								 std::array<Eigen::Vector4f, 4> &transformed_left_points,
								 std::array<Eigen::Vector4f, 4> &transformed_right_points) {
	  using namespace math_helper::eigen_conversions;
	  using namespace math_helper::gazebo_conversions;
	  using namespace math_helper::gazebo_eigen_conversions;

	  //================ Transform from box to world =========================================================//
	  // Transformation matrix from box coordinates system to world coordinate system
	  Eigen::Matrix4f T_box_to_world = Eigen::Matrix4f::Zero();
	  Eigen::Matrix3f box_rot = GazeboQuaternionToEigenQuaternion(rotation).toRotationMatrix();
	  Eigen::Vector3f box_pos = GazeboVec3ToEigenVec3(position);
	  PoseToMatrix4(box_pos, box_rot, T_box_to_world);

	  //================= Transform left face canonical points to world ======================================//
	  // Transform each canonical face corner to world
	  Eigen::Vector4f P_left_face_tl_to_world, P_left_face_tr_to_world, P_left_face_br_to_world, P_left_face_bl_to_world;
	  P_left_face_tl_to_world = T_box_to_world * left_canonical.at(0);
	  P_left_face_tr_to_world = T_box_to_world * left_canonical.at(1);
	  P_left_face_br_to_world = T_box_to_world * left_canonical.at(2);
	  P_left_face_bl_to_world = T_box_to_world * left_canonical.at(3);
	  transformed_left_points = {P_left_face_tl_to_world, P_left_face_tr_to_world, P_left_face_br_to_world, P_left_face_bl_to_world};

	  //================= Transform right face points to box =================================================//
	  // Transform each canonical face corner to world
	  Eigen::Vector4f P_right_face_tl_to_world, P_right_face_tr_to_world, P_right_face_br_to_world, P_right_face_bl_to_world;
	  P_right_face_tl_to_world = T_box_to_world * right_canonical.at(0);
	  P_right_face_tr_to_world = T_box_to_world * right_canonical.at(1);
	  P_right_face_br_to_world = T_box_to_world * right_canonical.at(2);
	  P_right_face_bl_to_world = T_box_to_world * right_canonical.at(3);
	  transformed_right_points = {P_right_face_tl_to_world, P_right_face_tr_to_world, P_right_face_br_to_world, P_right_face_bl_to_world};
  }
};

