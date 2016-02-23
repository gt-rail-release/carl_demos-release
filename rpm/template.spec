Name:           ros-indigo-carl-demos
Version:        0.0.9
Release:        0%{?dist}
Summary:        ROS carl_demos package

Group:          Development/Libraries
License:        BSD
URL:            http://ros.org/wiki/carl_demos
Source0:        %{name}-%{version}.tar.gz

Requires:       espeak-devel
Requires:       ros-indigo-actionlib
Requires:       ros-indigo-actionlib-msgs
Requires:       ros-indigo-carl-dynamixel
Requires:       ros-indigo-carl-moveit
Requires:       ros-indigo-carl-navigation
Requires:       ros-indigo-carl-safety
Requires:       ros-indigo-geometry-msgs
Requires:       ros-indigo-interactive-marker-proxy
Requires:       ros-indigo-message-runtime
Requires:       ros-indigo-rail-manipulation-msgs
Requires:       ros-indigo-rail-user-queue-manager
Requires:       ros-indigo-roscpp
Requires:       ros-indigo-tf2-web-republisher
Requires:       ros-indigo-web-video-server
BuildRequires:  espeak-devel
BuildRequires:  ros-indigo-actionlib
BuildRequires:  ros-indigo-actionlib-msgs
BuildRequires:  ros-indigo-carl-dynamixel
BuildRequires:  ros-indigo-carl-moveit
BuildRequires:  ros-indigo-carl-navigation
BuildRequires:  ros-indigo-catkin
BuildRequires:  ros-indigo-geometry-msgs
BuildRequires:  ros-indigo-message-generation
BuildRequires:  ros-indigo-rail-manipulation-msgs
BuildRequires:  ros-indigo-roscpp

%description
Demo Applications for CARL

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/indigo" \
        -DCMAKE_PREFIX_PATH="/opt/ros/indigo" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/indigo

%changelog
* Tue Feb 23 2016 Russell Toris <rctoris@wpi.edu> - 0.0.9-0
- Autogenerated by Bloom

* Tue Aug 18 2015 Russell Toris <rctoris@wpi.edu> - 0.0.8-0
- Autogenerated by Bloom

* Fri May 08 2015 Russell Toris <rctoris@wpi.edu> - 0.0.7-0
- Autogenerated by Bloom

* Wed Apr 29 2015 Russell Toris <rctoris@wpi.edu> - 0.0.6-0
- Autogenerated by Bloom

* Fri Feb 06 2015 Russell Toris <rctoris@wpi.edu> - 0.0.5-0
- Autogenerated by Bloom

* Thu Dec 18 2014 Russell Toris <rctoris@wpi.edu> - 0.0.4-0
- Autogenerated by Bloom

