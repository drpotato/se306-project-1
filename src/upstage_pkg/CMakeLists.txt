cmake_minimum_required(VERSION 2.8.3)
project(upstage_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  msg_pkg
)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS X11 GL GLU
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(UpstageLibrary
  src/Actor.cpp
  src/Context.cpp
  src/Keyboard.cpp
  src/Resource.cpp
  src/ResourceLoader.cpp
  src/ResourceManager.cpp
  src/ROSComm.cpp
  src/UpstageEnvironment.cpp
  src/geom/Mesh.cpp
  src/geom/Model.cpp
  src/maths/Quaternion.cpp
  src/renderer/Renderer.cpp
  src/renderer/Texture.cpp
  src/renderer/opengl/glloader.cpp
  src/resourceloaders/ResourceLoaderBMP.cpp
  src/resourceloaders/ResourceLoaderGenericXML.cpp
  src/resourceloaders/ResourceLoaderStageWorld.cpp
  src/resourceloaders/ResourceLoaderUpstageEnv.cpp
  src/xml/XML.cpp
)


add_executable(Upstage src/UpstageMain.cpp)

add_dependencies(UpstageLibrary msg_pkg_generate_messages_cpp)

target_link_libraries(
  Upstage
  UpstageLibrary
  X11
  GL
  GLU
  ${catkin_LIBRARIES}
)