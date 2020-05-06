# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(ndt_omp_CONFIG_INCLUDED)
  return()
endif()
set(ndt_omp_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(ndt_omp_SOURCE_PREFIX /home/jxl/jxl_ws/src/ndt_omp)
  set(ndt_omp_DEVEL_PREFIX /home/jxl/jxl_ws/src/ndt_omp/build/devel)
  set(ndt_omp_INSTALL_PREFIX "")
  set(ndt_omp_PREFIX ${ndt_omp_DEVEL_PREFIX})
else()
  set(ndt_omp_SOURCE_PREFIX "")
  set(ndt_omp_DEVEL_PREFIX "")
  set(ndt_omp_INSTALL_PREFIX /usr/local)
  set(ndt_omp_PREFIX ${ndt_omp_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'ndt_omp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(ndt_omp_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/jxl/jxl_ws/src/ndt_omp/include " STREQUAL " ")
  set(ndt_omp_INCLUDE_DIRS "")
  set(_include_dirs "/home/jxl/jxl_ws/src/ndt_omp/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'koide <koide@aisl.cs.tut.ac.jp>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${ndt_omp_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'ndt_omp' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'ndt_omp' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/jxl/jxl_ws/src/ndt_omp/${idir}'.  ${_report}")
    endif()
    _list_append_unique(ndt_omp_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "ndt_omp")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND ndt_omp_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND ndt_omp_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND ndt_omp_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND ndt_omp_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/jxl/jxl_ws/src/ndt_omp/build/devel/lib;/home/jxl/jxl_ws/devel/lib;/home/jxl/third_softwares/Autoware/ros/install/ymc/lib;/home/jxl/third_softwares/Autoware/ros/install/xsens_driver/lib;/home/jxl/third_softwares/Autoware/ros/install/lattice_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/waypoint_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/waypoint_maker/lib;/home/jxl/third_softwares/Autoware/ros/install/way_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/trafficlight_recognizer/lib;/home/jxl/third_softwares/Autoware/ros/install/op_utilities/lib;/home/jxl/third_softwares/Autoware/ros/install/op_simulation_package/lib;/home/jxl/third_softwares/Autoware/ros/install/op_local_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/op_global_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_kf_contour_track/lib;/home/jxl/third_softwares/Autoware/ros/install/op_ros_helpers/lib;/home/jxl/third_softwares/Autoware/ros/install/lane_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/ff_waypoint_follower/lib;/home/jxl/third_softwares/Autoware/ros/install/dp_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/waypoint_follower/lib;/home/jxl/third_softwares/Autoware/ros/install/vlg22c_cam/lib;/home/jxl/third_softwares/Autoware/ros/install/vision_ssd_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/vision_segment_enet_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/vision_lane_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/vision_dpm_ttic_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/vision_darknet_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/vision_beyond_track/lib;/home/jxl/third_softwares/Autoware/ros/install/vehicle_socket/lib;/home/jxl/third_softwares/Autoware/ros/install/vehicle_model/lib;/home/jxl/third_softwares/Autoware/ros/install/vehicle_gazebo_simulation_launcher/lib;/home/jxl/third_softwares/Autoware/ros/install/vehicle_gazebo_simulation_interface/lib;/home/jxl/third_softwares/Autoware/ros/install/vehicle_description/lib;/home/jxl/third_softwares/Autoware/ros/install/op_simu/lib;/home/jxl/third_softwares/Autoware/ros/install/op_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/op_utility/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_euclidean_cluster_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/vector_map_server/lib;/home/jxl/third_softwares/Autoware/ros/install/road_occupancy_processor/lib;/home/jxl/third_softwares/Autoware/ros/install/costmap_generator/lib;/home/jxl/third_softwares/Autoware/ros/install/object_map/lib;/home/jxl/third_softwares/Autoware/ros/install/naive_motion_predict/lib;/home/jxl/third_softwares/Autoware/ros/install/map_file/lib;/home/jxl/third_softwares/Autoware/ros/install/libvectormap/lib;/home/jxl/third_softwares/Autoware/ros/install/imm_ukf_pda_track/lib;/home/jxl/third_softwares/Autoware/ros/install/decision_maker/lib;/home/jxl/third_softwares/Autoware/ros/install/vector_map/lib;/home/jxl/third_softwares/Autoware/ros/install/vector_map_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/vectacam/lib;/home/jxl/third_softwares/Autoware/ros/install/udon_socket/lib;/home/jxl/third_softwares/Autoware/ros/install/tablet_socket/lib;/home/jxl/third_softwares/Autoware/ros/install/runtime_manager/lib;/home/jxl/third_softwares/Autoware/ros/install/mqtt_socket/lib;/home/jxl/third_softwares/Autoware/ros/install/tablet_socket_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/state_machine_lib/lib;/home/jxl/third_softwares/Autoware/ros/install/sound_player/lib;/home/jxl/third_softwares/Autoware/ros/install/sick_lms5xx/lib;/home/jxl/third_softwares/Autoware/ros/install/sick_ldmrs_tools/lib;/home/jxl/third_softwares/Autoware/ros/install/sick_ldmrs_driver/lib;/home/jxl/third_softwares/Autoware/ros/install/sick_ldmrs_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/sick_ldmrs_description/lib;/home/jxl/third_softwares/Autoware/ros/install/rslidar_driver/lib;/home/jxl/third_softwares/Autoware/ros/install/rslidar_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/points2image/lib;/home/jxl/third_softwares/Autoware/ros/install/rosinterface/lib;/home/jxl/third_softwares/Autoware/ros/install/rosbag_controller/lib;/home/jxl/third_softwares/Autoware/ros/install/roi_object_filter/lib;/home/jxl/third_softwares/Autoware/ros/install/range_vision_fusion/lib;/home/jxl/third_softwares/Autoware/ros/install/pos_db/lib;/home/jxl/third_softwares/Autoware/ros/install/points_preprocessor/lib;/home/jxl/third_softwares/Autoware/ros/install/points_downsampler/lib;/home/jxl/third_softwares/Autoware/ros/install/pixel_cloud_fusion/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_localizer/lib;/home/jxl/third_softwares/Autoware/ros/install/pcl_omp_registration/lib;/home/jxl/third_softwares/Autoware/ros/install/pc2_downsampler/lib;/home/jxl/third_softwares/Autoware/ros/install/ouster_ros/lib;/home/jxl/third_softwares/Autoware/ros/install/oculus_socket/lib;/home/jxl/third_softwares/Autoware/ros/install/obj_db/lib;/home/jxl/third_softwares/Autoware/ros/install/nmea_navsat/lib;/home/jxl/third_softwares/Autoware/ros/install/ndt_tku/lib;/home/jxl/third_softwares/Autoware/ros/install/ndt_gpu/lib;/home/jxl/third_softwares/Autoware/ros/install/ndt_cpu/lib;/home/jxl/third_softwares/Autoware/ros/install/multi_lidar_calibrator/lib;/home/jxl/third_softwares/Autoware/ros/install/microstrain_driver/lib;/home/jxl/third_softwares/Autoware/ros/install/memsic_imu/lib;/home/jxl/third_softwares/Autoware/ros/install/marker_downsampler/lib;/home/jxl/third_softwares/Autoware/ros/install/map_tools/lib;/home/jxl/third_softwares/Autoware/ros/install/map_tf_generator/lib;/home/jxl/third_softwares/Autoware/ros/install/log_tools/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_shape_estimation/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_point_pillars/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_naive_l_shape_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_fake_perception/lib;/home/jxl/third_softwares/Autoware/ros/install/lidar_apollo_cnn_seg_detect/lib;/home/jxl/third_softwares/Autoware/ros/install/libdpm_ttic/lib;/home/jxl/third_softwares/Autoware/ros/install/lgsvl_simulator_bridge/lib;/home/jxl/third_softwares/Autoware/ros/install/lgsvl_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/kvaser/lib;/home/jxl/third_softwares/Autoware/ros/install/kitti_launch/lib;/home/jxl/third_softwares/Autoware/ros/install/kitti_player/lib;/home/jxl/third_softwares/Autoware/ros/install/kitti_box_publisher/lib;/home/jxl/third_softwares/Autoware/ros/install/javad_navsat_driver/lib;/home/jxl/third_softwares/Autoware/ros/install/integrated_viewer/lib;/home/jxl/third_softwares/Autoware/ros/install/image_processor/lib;/home/jxl/third_softwares/Autoware/ros/install/hokuyo/lib;/home/jxl/third_softwares/Autoware/ros/install/graph_tools/lib;/home/jxl/third_softwares/Autoware/ros/install/gnss_localizer/lib;/home/jxl/third_softwares/Autoware/ros/install/gnss/lib;/home/jxl/third_softwares/Autoware/ros/install/glviewer/lib;/home/jxl/third_softwares/Autoware/ros/install/gazebo_world_description/lib;/home/jxl/third_softwares/Autoware/ros/install/gazebo_imu_description/lib;/home/jxl/third_softwares/Autoware/ros/install/gazebo_camera_description/lib;/home/jxl/third_softwares/Autoware/ros/install/garmin/lib;/home/jxl/third_softwares/Autoware/ros/install/freespace_planner/lib;/home/jxl/third_softwares/Autoware/ros/install/fastvirtualscan/lib;/home/jxl/third_softwares/Autoware/ros/install/detected_objects_visualizer/lib;/home/jxl/third_softwares/Autoware/ros/install/decision_maker_panel/lib;/home/jxl/third_softwares/Autoware/ros/install/dbw_mkz_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/data_preprocessor/lib;/home/jxl/third_softwares/Autoware/ros/install/custom_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/calibration_publisher/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_health_checker/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_system_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_rviz_plugins/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_pointgrey_drivers/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_driveworks_interface/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_connector/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_camera_lidar_calibrator/lib;/home/jxl/third_softwares/Autoware/ros/install/astar_search/lib;/home/jxl/third_softwares/Autoware/ros/install/as/lib;/home/jxl/third_softwares/Autoware/ros/install/amathutils_lib/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_launcher_rviz/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_launcher/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_driveworks_gmsl_interface/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_config_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_can_msgs/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_build_flags/lib;/home/jxl/third_softwares/Autoware/ros/install/autoware_bag_tools/lib;/home/jxl/third_softwares/Autoware/ros/install/adi_driver/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(ndt_omp_LIBRARY_DIRS ${lib_path})
      list(APPEND ndt_omp_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'ndt_omp'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND ndt_omp_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(ndt_omp_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${ndt_omp_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 ndt_omp_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${ndt_omp_dep}_FOUND)
      find_package(${ndt_omp_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${ndt_omp_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(ndt_omp_INCLUDE_DIRS ${${ndt_omp_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(ndt_omp_LIBRARIES ${ndt_omp_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${ndt_omp_dep}_LIBRARIES})
  _list_append_deduplicate(ndt_omp_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(ndt_omp_LIBRARIES ${ndt_omp_LIBRARIES})

  _list_append_unique(ndt_omp_LIBRARY_DIRS ${${ndt_omp_dep}_LIBRARY_DIRS})
  list(APPEND ndt_omp_EXPORTED_TARGETS ${${ndt_omp_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${ndt_omp_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
