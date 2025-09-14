configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")


set(kinova_SRC
  kinova.cpp
)

set(kinova_HDR
  kinova.h
)

add_library(mc_kinova SHARED ${kinova_SRC} ${kinova_HDR})
add_library(mc_kinova::kinova ALIAS mc_kinova)
set_target_properties(mc_kinova PROPERTIES EXPORT_NAME kinova)
target_include_directories(mc_kinova PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)
target_include_directories(mc_kinova INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(mc_kinova PUBLIC mc_rtc::mc_rbdyn)
target_compile_definitions(mc_kinova PRIVATE -DMC_KINOVA_EXPORTS)

install(FILES ${kinova_HDR} DESTINATION include/mc_kinova)
install(TARGETS mc_kinova
  EXPORT mc_kinovaTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_robot(kinova module.cpp "")
target_link_libraries(kinova PRIVATE mc_kinova::kinova)

add_dependencies(kinova generate-kinova-urdf)
