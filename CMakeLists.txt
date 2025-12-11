cmake_minimum_required(VERSION 3.10)
project(BulletSims)

set(CMAKE_CXX_STANDARD 14)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Bullet3 Submodule
# We only need to build the libraries we use.

# Bullet's CMake is a bit heavy, so ltes disable demos/extras we don't need to speed up bullet build.
set(BUILD_BULLET3 ON CACHE BOOL "Build Bullet3" FORCE)
set(BUILD_PYBULLET OFF CACHE BOOL "Build PyBullet" FORCE)
set(BUILD_ENET OFF CACHE BOOL "Build ENet" FORCE)
set(BUILD_CLSOCKET OFF CACHE BOOL "Build CLSocket" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "Build CPU Demos" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "Build OpenGL3 Demos" FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "Build Bullet2 Demos" FORCE)
set(BUILD_EXTRAS ON CACHE BOOL "Build Extras" FORCE) # Needed for Serialize/etc
set(BUILD_UNIT_TESTS OFF CACHE BOOL "Build Unit Tests" FORCE)

# Linux fix
if(UNIX AND NOT APPLE)
    add_definitions(-D_LINUX)
endif()

add_subdirectory(external/bullet3)

# Find OpenGL
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(X11 REQUIRED)
# find_package(GLEW REQUIRED) # Use bundled GLEW instead



# Include Directories
include_directories(
    external/bullet3/src
    external/bullet3/examples
    external/bullet3/examples/ThirdPartyLibs
    external/bullet3/examples/ThirdPartyLibs/glad
)

# --- Helper Library ---
# This library contains the common boilerplate code from Bullet examples
# that we want to reuse across our own examples.

set(BULLET_HELPER_SOURCES
    external/bullet3/examples/ExampleBrowser/OpenGLGuiHelper.cpp
    external/bullet3/examples/ExampleBrowser/GL_ShapeDrawer.cpp
    external/bullet3/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp
    external/bullet3/examples/Utils/b3Clock.cpp
    external/bullet3/examples/Utils/b3ResourcePath.cpp
    external/bullet3/examples/ThirdPartyLibs/stb_image/stb_image.cpp
    external/bullet3/examples/ThirdPartyLibs/stb_image/stb_image_write.cpp
    external/bullet3/examples/ThirdPartyLibs/glad/gl.c
    external/bullet3/examples/OpenGLWindow/SimpleOpenGL3App.cpp
    external/bullet3/examples/OpenGLWindow/SimpleCamera.cpp
    external/bullet3/examples/OpenGLWindow/GLInstancingRenderer.cpp
    external/bullet3/examples/OpenGLWindow/GLPrimitiveRenderer.cpp
    external/bullet3/examples/OpenGLWindow/GLRenderToTexture.cpp
    external/bullet3/examples/OpenGLWindow/LoadShader.cpp
    external/bullet3/examples/OpenGLWindow/TwFonts.cpp
    external/bullet3/examples/OpenGLWindow/OpenSans.cpp
    external/bullet3/examples/OpenGLWindow/fontstash.cpp
    external/bullet3/examples/OpenGLWindow/opengl_fontstashcallbacks.cpp
    external/bullet3/examples/OpenGLWindow/X11OpenGLWindow.cpp
    # Add other common helpers here as needed
)

add_library(bullet_helpers STATIC ${BULLET_HELPER_SOURCES})
target_link_libraries(bullet_helpers
    BulletDynamics
    BulletCollision
    LinearMath
    Bullet3Common
    OpenGL::GL
    Threads::Threads
    ${X11_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# --- Example 1: Basic Simulation ---
add_executable(example_basic src/example_basic/main.cpp)
target_link_libraries(example_basic
    bullet_helpers
    BulletDynamics
    BulletCollision
    LinearMath
    Bullet3Common
    OpenGL::GL
    Threads::Threads
    ${X11_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# --- Example 2: URDF Simulation ---
# This one needs extra importers
set(URDF_HELPER_SOURCES
    external/bullet3/examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
    external/bullet3/examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
    external/bullet3/examples/Importers/ImportURDFDemo/UrdfParser.cpp
    external/bullet3/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
    external/bullet3/examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
    external/bullet3/examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
    external/bullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
    external/bullet3/examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
    external/bullet3/examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
    external/bullet3/examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
    external/bullet3/examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
)

add_library(urdf_helpers STATIC ${URDF_HELPER_SOURCES})
target_link_libraries(urdf_helpers
    bullet_helpers
    Bullet3Common
    BulletCollision
    BulletDynamics
    LinearMath
)

add_executable(example_urdf src/example_urdf/main.cpp)
target_link_libraries(example_urdf
    urdf_helpers
    bullet_helpers
    BulletDynamics
    BulletCollision
    LinearMath
    Bullet3Common
    OpenGL::GL
    Threads::Threads
    ${X11_LIBRARIES}
    ${CMAKE_DL_LIBS}
)
