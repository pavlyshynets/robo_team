cmake_minimum_required(VERSION 3.8)
project(serial_communication)

if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(custom_interfaces REQUIRED)

    # Create the executable
    add_executable(serial_node 
	      src/serial_handler.cpp
	      )

      # Specify dependencies
      ament_target_dependencies(serial_node
	        rclcpp
		  custom_interfaces
		  )

	  # Link with libserial
	  target_link_libraries(serial_node
		    serial
		    )

	    # Install the executable
	    install(TARGETS
		      serial_node
		        DESTINATION lib/${PROJECT_NAME}
			)

		# Install launch and config files
		install(DIRECTORY
			    config
			      DESTINATION share/${PROJECT_NAME}
			      )

		      ament_package()
