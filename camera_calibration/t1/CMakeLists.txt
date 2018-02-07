


if( WIN32 )

elseif( UNIX AND NOT APPLE )

    message( "Camera calibration" )
    add_executable( main.out 
                    main.cpp
                    )

    target_link_libraries( main.out
                           ${X11_LIBRARIES} 
                           ${CMAKE_THREAD_LIBS_INIT}
                           ${OpenCV_LIBS}
                           dl )


elseif( APPLE )

endif()