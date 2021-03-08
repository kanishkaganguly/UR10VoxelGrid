//
// Created by Kanishka Ganguly on 7/31/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

#include <ros/package.h>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/dup_filter_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

/**
 *  @class Logger logger.hpp "include/logger.hpp"
 *  @brief Helper class to set up spdlog for global logging.
 *  Creates single logger with multiple sinks,
 *  namely the duplicate filtering and the basic file sink.
 *  The logger is then registered for use by all other files in the project.
 */
class Logger {
public:
	/**
	 * @brief Constructor for logging with spdlog
	 */
	Logger(std::string name) {
		// Generate timestamp for log messages
		std::time_t curr_time = std::time(nullptr);
		std::stringstream ss;
		ss << std::put_time(std::localtime(&curr_time), "%m-%d-%Y_%H-%M-%S");
		// Fetch filepath from ROS
		std::string pkg_path = ros::package::getPath("voxel_grid_plugin");
		std::string log_name = pkg_path + "/logs/log_" + name + "_" + ss.str() + ".txt";

		// Create sinks for logging
		std::vector <spdlog::sink_ptr> sinks;
		// This one removes duplicate messages within 5 seconds, and logs to stdout
		auto dup_filter = std::make_shared<spdlog::sinks::dup_filter_sink_mt>(std::chrono::seconds(5));
		dup_filter->add_sink(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
		sinks.push_back(dup_filter);
		// This one logs to file
		sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_name));
		// Combine everything into one logger
		auto combined_logger = std::make_shared<spdlog::logger>("file_logger", begin(sinks), end(sinks));
		// Register logger for global use
		spdlog::register_logger(combined_logger);
	}
};