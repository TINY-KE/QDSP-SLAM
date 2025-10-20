

# SET(CMAKE_BUILD_TYPE Debug)

add_library(Relationship SHARED
Relationship.cpp
)

target_link_libraries(Relationship
        ${PCL_LIBRARIES}
        Config
        )


# target_link_libraries(Relationship
#         ORB_SLAM2
#         )
