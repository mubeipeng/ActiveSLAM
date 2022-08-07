# Make sure CMake can find all of the header and source files
include_directories(AprilTags .)

# Generate the library
#add_library(apriltags
#        Edge.cc AprilTags/Edge.h FloatImage.cc AprilTags/FloatImage.h
#        Gaussian.cc AprilTags/Gaussian.h GLine2D.cc AprilTags/GLine2D.h
#        GLineSegment2D.cc AprilTags/GLineSegment2D.h GrayModel.cc AprilTags/GrayModel.h
#        AprilTags/Gridder.h Homography33.cc aprilTags/Homography33.h AprilTags/MathUtil.h
#        MathUtil.cc AprilTags/pch.h Quad.cc AprilTags/Quad.h
#        Segment.cc AprilTags/Segment.h AprilTags/Tag16h5.h AprilTags/Tag16h5_other.h
#        AprilTags/Tag25h7.h aprilTags/Tag25h9.h AprilTags/Tag36h9.h AprilTags/Tag36h11.h
#        AprilTags/Tag36h11_other.h TagDetection.cc AprilTags/TagDetection.h TagDetector.cc
#        AprilTags/TagDetector.h TagFamily.cc AprilTags/TagFamily.h UnionFindSimple.cc
#        AprilTags/UnionFindSimple.h AprilTags/XYWeight.h
#        )

file(GLOB SRCS RELATIVE "${PROJECT_SOURCE_DIR}/src/apriltags" "*.cc")
add_library(apriltags ${SRCS}
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Edge.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/FloatImage.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Gaussian.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/GLine2D.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/GLineSegment2D.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/GrayModel.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Gridder.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Homography33.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/MathUtil.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/pch.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Quad.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Segment.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Tag36h11.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/TagDetection.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/TagFamily.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/UnionFindSimple.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/XYWeight.h
#    ${PROJECT_SOURCE_DIR}/include/AprilTags/Tag36h11.h
)

# Link AprilTags library with OpenCV
LINK_LIBRARIES(apriltags ${OpenCV_LIBRARIES})



