
if(WIN32)
file(GLOB SOURCE ./*.cpp ../include/*.h)
elseif(UNIX AND NOT APPLE)
file(GLOB SOURCE ./*.cpp)
endif()

add_library(myslam ${SOURCE})

target_link_libraries(myslam ${THIRD_PARTY_LIBS})