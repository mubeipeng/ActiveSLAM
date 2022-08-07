# find all source and header files

file(GLOB SRCS RELATIVE "${PROJECT_SOURCE_DIR}/src/isam" "*.cpp")
#file(GLOB HEADERS "${PROJECT_SOURCE_DIR}/isamlib/isam/" "*.h")

# isamlib
add_library(isamlib ${SRCS})
#    isam/Anchor.h isam/Cholesky.h isam/ChowLiuTree.h isam/covariance.h isam/Covariances.h isam/Element.h isam/Factor.h isam/glc.h
#    isam/GLCReparam.h isam/Graph.h isam/isam.h isam/Jacobian.h isam/Node.h isam/Noise.h isam/numericalDiff.h isam/OptimizationInterface.h
#    isam/Optimizer.h isam/OrderedSparseMatrix.h isam/Point2d.h isam/Point2d.h isam/Pose2d.h isam/Properties.h isam/robust.h isam/Rot3d.h
#    isam/slam2d.h isam/slam3d.h isam/Slam.h isam/slam_monocular.h isam/slam_stereo.h isam/SparseMatrix.h isam/SparseSystem.h
#    isam/SparseVector.h isam/util.h)
#set_property(TARGET isamlib PROPERTY OUTPUT_NAME isam)

# simple way of finding CHOLMOD
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDES})
target_link_libraries(isamlib ${CHOLMOD_LIBRARIES})

# install library
#install(TARGETS isamlib
#  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#)
# install header files
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/isam/ DESTINATION include/isam
#          FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

