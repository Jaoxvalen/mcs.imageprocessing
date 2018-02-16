cmake_minimum_required( VERSION 3.5.0 )
project( proc_imag C CXX )

set( CMAKE_BUILD_TYPE Debug )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS_DEBUG "-g" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )



if( WIN32 )

    message( "GENERATING MAKEFILES/PROJECTS FOR WINDOWS" )
    set( "OpenCV_DIR" "C:/opencv/build" )
    find_package( OpenCV REQUIRED )
    find_package( Threads REQUIRED )

elseif( UNIX AND NOT APPLE )

    message( "GENERATING MAKEFILES/PROJECTS FOR LINUX" )


    find_package( Threads REQUIRED )
    find_package( PkgConfig REQUIRED )
    find_package( OpenCV REQUIRED )

    include_directories ( 
                            /usr/local/include
                            ${OpenCV_INCLUDE_DIRS} 
                        )
    link_directories( /usr/local/lib )

elseif( APPLE )

    message( "GENERATING MAKEFILES/PROJECTS FOR MACOS" )

endif()

add_subdirectory( t1 )