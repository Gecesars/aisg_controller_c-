cmake_minimum_required(VERSION 3.20)

project(antenna_tilt_controller VERSION 0.1.0 LANGUAGES CXX)

option(ATC_BUILD_TESTS "Build the protocol and controller tests" ON)
option(ATC_BUILD_GUI "Build the Qt 6 desktop application" ON)

add_library(atc_core
    src/adb.cpp
    src/aisg.cpp
    src/aisg3.cpp
    src/controller.cpp
    src/domain.cpp
    src/hdlc.cpp
    src/posix_serial_transport.cpp
    src/simulated_transport.cpp
)

add_library(atc::core ALIAS atc_core)

target_compile_features(atc_core PUBLIC cxx_std_20)
target_include_directories(atc_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_options(atc_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wshadow>
)

find_package(Threads REQUIRED)
target_link_libraries(atc_core PUBLIC Threads::Threads)

if(ATC_BUILD_GUI)
    find_package(Qt6 6.4 COMPONENTS Core Gui Widgets QUIET)

    # A locally extracted qt6-base-dev can be supplied explicitly for
    # development environments where installing system packages is not an
    # option. Normal developer systems take the find_package branch above.
    if(NOT Qt6_FOUND)
        set(ATC_QT_DEV_ROOT ""
            CACHE PATH "Root containing locally extracted Qt 6 development headers")
        if(NOT ATC_QT_DEV_ROOT)
            message(FATAL_ERROR
                "Qt 6 development files were not found. Install qt6-base-dev, "
                "set CMAKE_PREFIX_PATH, or pass -DATC_QT_DEV_ROOT=/path/to/root.")
        endif()
        set(_qt_include "${ATC_QT_DEV_ROOT}/usr/include/x86_64-linux-gnu/qt6")
        find_file(_qt_core_library libQt6Core.so.6 PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu NO_DEFAULT_PATH)
        find_file(_qt_gui_library libQt6Gui.so.6 PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu NO_DEFAULT_PATH)
        find_file(_qt_widgets_library libQt6Widgets.so.6 PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu NO_DEFAULT_PATH)
        if(NOT EXISTS "${_qt_include}/QtWidgets/QApplication" OR
           NOT _qt_core_library OR NOT _qt_gui_library OR NOT _qt_widgets_library)
            message(FATAL_ERROR
                "Qt 6 development files were not found. Install qt6-base-dev, "
                "set CMAKE_PREFIX_PATH, or provide ATC_QT_DEV_ROOT.")
        endif()

        add_library(Qt6::Core SHARED IMPORTED)
        set_target_properties(Qt6::Core PROPERTIES
            IMPORTED_LOCATION "${_qt_core_library}"
            INTERFACE_INCLUDE_DIRECTORIES "${_qt_include};${_qt_include}/QtCore"
            INTERFACE_POSITION_INDEPENDENT_CODE ON
            INTERFACE_COMPILE_DEFINITIONS "QT_CORE_LIB")
        add_library(Qt6::Gui SHARED IMPORTED)
        set_target_properties(Qt6::Gui PROPERTIES
            IMPORTED_LOCATION "${_qt_gui_library}"
            INTERFACE_INCLUDE_DIRECTORIES "${_qt_include};${_qt_include}/QtGui"
            INTERFACE_POSITION_INDEPENDENT_CODE ON
            INTERFACE_COMPILE_DEFINITIONS "QT_GUI_LIB"
            INTERFACE_LINK_LIBRARIES "Qt6::Core")
        add_library(Qt6::Widgets SHARED IMPORTED)
        set_target_properties(Qt6::Widgets PROPERTIES
            IMPORTED_LOCATION "${_qt_widgets_library}"
            INTERFACE_INCLUDE_DIRECTORIES "${_qt_include};${_qt_include}/QtWidgets"
            INTERFACE_POSITION_INDEPENDENT_CODE ON
            INTERFACE_COMPILE_DEFINITIONS "QT_WIDGETS_LIB"
            INTERFACE_LINK_LIBRARIES "Qt6::Gui;Qt6::Core")
    endif()

    add_executable(antenna-tilt-controller
        src/gui/backend.cpp
        src/gui/main.cpp
        src/gui/main_window.cpp
    )
    target_compile_features(antenna-tilt-controller PRIVATE cxx_std_20)
    target_link_libraries(antenna-tilt-controller PRIVATE atc::core Qt6::Widgets)
    target_compile_options(antenna-tilt-controller PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-fPIC;-Wall;-Wextra;-Wpedantic;-Wconversion;-Wshadow>
    )
    target_link_options(antenna-tilt-controller PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-pie>
    )
    install(TARGETS antenna-tilt-controller RUNTIME DESTINATION bin)
    install(FILES resources/antenna-tilt-controller.desktop DESTINATION share/applications)
    install(FILES resources/antenna-tilt-controller.svg DESTINATION share/icons/hicolor/scalable/apps)
    install(FILES README.md THIRD_PARTY.md SECURITY.md LICENSE docs/COMPATIBILITY.md
        DESTINATION share/doc/antenna-tilt-controller)
    install(FILES licenses/ret_aisg2-MIT.txt DESTINATION share/doc/antenna-tilt-controller/licenses)
endif()

if(ATC_BUILD_TESTS)
    enable_testing()
    add_executable(atc_core_tests tests/test_main.cpp)
    target_compile_features(atc_core_tests PRIVATE cxx_std_20)
    target_link_libraries(atc_core_tests PRIVATE atc::core)
    add_test(NAME atc_core_tests COMMAND atc_core_tests)
endif()

set(CPACK_PACKAGE_NAME "antenna-tilt-controller")
set(CPACK_PACKAGE_VENDOR "ATC Open Controller")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Controlador AISG/RET/TMA para Linux")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_GENERATOR "TGZ")
set(CPACK_PACKAGE_FILE_NAME
    "antenna-tilt-controller-${PROJECT_VERSION}-linux-${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)
